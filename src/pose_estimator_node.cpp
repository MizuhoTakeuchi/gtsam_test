// GTSAM-based pose estimator fusing GPS and LiDAR measurements
// Optimization runs at 500ms cycle using ISAM2 incremental solver

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <mutex>
#include <vector>

using gtsam::symbol_shorthand::X;  // Pose3 variables: X(0), X(1), ...

// ---------------------------------------------------------------------------
// Utility: convert ROS PoseWithCovariance to GTSAM Pose3 + 6x6 noise model
// ---------------------------------------------------------------------------

// ROS covariance order : [tx, ty, tz, rx, ry, rz]
// GTSAM tangent space   : [rx, ry, rz, tx, ty, tz]
// We need to swap the 3x3 blocks accordingly.

static gtsam::Pose3 rosPoseToGtsam(
    const geometry_msgs::msg::Pose & p)
{
  gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
      p.orientation.w, p.orientation.x,
      p.orientation.y, p.orientation.z);
  gtsam::Point3 trans(p.position.x, p.position.y, p.position.z);
  return gtsam::Pose3(rot, trans);
}

static gtsam::SharedNoiseModel rosCovarianceToGtsamNoise(
    const std::array<double, 36> & cov_ros)
{
  // Build 6x6 matrix in GTSAM order [rot, trans]
  Eigen::Matrix<double, 6, 6> cov_gtsam;

  // ROS 6x6 layout (row-major, indices 0-5 = tx,ty,tz,rx,ry,rz):
  //   [TT  TR]      GTSAM wants  [RR  RT]
  //   [RT  RR]                    [TR  TT]
  Eigen::Matrix<double, 6, 6> cov_ros_mat;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      cov_ros_mat(i, j) = cov_ros[i * 6 + j];

  // Reorder: swap top-left 3x3 (TT) with bottom-right 3x3 (RR)
  cov_gtsam.block<3, 3>(0, 0) = cov_ros_mat.block<3, 3>(3, 3); // RR
  cov_gtsam.block<3, 3>(0, 3) = cov_ros_mat.block<3, 3>(3, 0); // RT
  cov_gtsam.block<3, 3>(3, 0) = cov_ros_mat.block<3, 3>(0, 3); // TR
  cov_gtsam.block<3, 3>(3, 3) = cov_ros_mat.block<3, 3>(0, 0); // TT

  return gtsam::noiseModel::Gaussian::Covariance(cov_gtsam);
}

static geometry_msgs::msg::PoseStamped gtsamPoseToRos(
    const gtsam::Pose3 & pose, const rclcpp::Time & stamp,
    const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  gtsam::Point3 t = pose.translation();
  msg.pose.position.x = t.x();
  msg.pose.position.y = t.y();
  msg.pose.position.z = t.z();

  gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();

  return msg;
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class PoseEstimatorNode : public rclcpp::Node
{
public:
  PoseEstimatorNode()
  : Node("pose_estimator")
  {
    // Parameters
    const int period_ms =
        this->declare_parameter("optimization_period_ms", 500);
    between_noise_trans_ =
        this->declare_parameter("between_noise_translation", 1.0);
    between_noise_rot_ =
        this->declare_parameter("between_noise_rotation", 0.1);

    // ISAM2
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 1;
    isam2_ = std::make_unique<gtsam::ISAM2>(params);

    // Subscribers (QoS: best-effort to match typical sensor drivers)
    rclcpp::QoS sensor_qos(10);
    sensor_qos.best_effort();

    gps_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/gps/pose_with_covariance", sensor_qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          gps_buffer_.push_back(*msg);
        });

    lidar_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/lidar/pose_with_covariance", sensor_qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          lidar_buffer_.push_back(*msg);
        });

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/estimated_path", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Optimization timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&PoseEstimatorNode::optimizationCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "PoseEstimator started (period=%dms)", period_ms);
  }

private:
  // -------------------------------------------------------
  // Core optimization routine (called every 500ms)
  // -------------------------------------------------------
  void optimizationCallback()
  {
    // Grab buffered measurements
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> gps_meas;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> lidar_meas;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      gps_meas.swap(gps_buffer_);
      lidar_meas.swap(lidar_buffer_);
    }

    if (gps_meas.empty() && lidar_meas.empty()) {
      return;  // Nothing to do
    }

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    const gtsam::Key current_key = X(pose_index_);

    // ------- Compute initial estimate for the new pose -------
    // Use the average of available measurements as initial guess
    gtsam::Pose3 initial_estimate;
    if (!lidar_meas.empty()) {
      initial_estimate = rosPoseToGtsam(lidar_meas.back().pose.pose);
    } else if (!gps_meas.empty()) {
      initial_estimate = rosPoseToGtsam(gps_meas.back().pose.pose);
    } else if (pose_index_ > 0) {
      initial_estimate = latest_pose_;
    }

    // ------- First pose: add a prior -------
    if (pose_index_ == 0) {
      // Use the first available measurement's covariance as prior noise
      gtsam::SharedNoiseModel prior_noise;
      if (!lidar_meas.empty()) {
        prior_noise =
            rosCovarianceToGtsamNoise(lidar_meas.front().pose.covariance);
      } else {
        prior_noise =
            rosCovarianceToGtsamNoise(gps_meas.front().pose.covariance);
      }
      new_factors.addPrior(current_key, initial_estimate, prior_noise);
    }

    // ------- BetweenFactor from previous pose -------
    if (pose_index_ > 0) {
      // Constant-pose motion model (identity transform with tunable noise)
      auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) <<
              between_noise_rot_, between_noise_rot_, between_noise_rot_,
              between_noise_trans_, between_noise_trans_, between_noise_trans_
          ).finished());

      // Use delta from latest_pose to initial_estimate as between measurement
      gtsam::Pose3 delta = latest_pose_.between(initial_estimate);
      new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(pose_index_ - 1), current_key, delta, between_noise);
    }

    // ------- GPS measurement factors (PriorFactor on this pose) -------
    for (const auto & m : gps_meas) {
      gtsam::Pose3 measured = rosPoseToGtsam(m.pose.pose);
      gtsam::SharedNoiseModel noise =
          rosCovarianceToGtsamNoise(m.pose.covariance);
      new_factors.addPrior(current_key, measured, noise);
    }

    // ------- LiDAR measurement factors (PriorFactor on this pose) -------
    for (const auto & m : lidar_meas) {
      gtsam::Pose3 measured = rosPoseToGtsam(m.pose.pose);
      gtsam::SharedNoiseModel noise =
          rosCovarianceToGtsamNoise(m.pose.covariance);
      new_factors.addPrior(current_key, measured, noise);
    }

    // ------- Insert initial value & run ISAM2 update -------
    new_values.insert(current_key, initial_estimate);
    isam2_->update(new_factors, new_values);

    // (Optional) run additional ISAM2 iterations for convergence
    isam2_->update();

    // ------- Extract result -------
    gtsam::Values result = isam2_->calculateEstimate();
    latest_pose_ = result.at<gtsam::Pose3>(current_key);

    // ------- Publish -------
    rclcpp::Time now = this->now();

    // PoseStamped
    auto pose_msg = gtsamPoseToRos(latest_pose_, now, "map");
    pose_pub_->publish(pose_msg);

    // Path (append)
    path_.header.stamp = now;
    path_.header.frame_id = "map";
    path_.poses.push_back(pose_msg);
    path_pub_->publish(path_);

    // TF: map -> base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose_msg.pose.position.x;
    tf.transform.translation.y = pose_msg.pose.position.y;
    tf.transform.translation.z = pose_msg.pose.position.z;
    tf.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    RCLCPP_INFO(this->get_logger(),
                "[%zu] Optimized pose: (%.2f, %.2f, %.2f)  "
                "GPS=%zu LiDAR=%zu factors",
                pose_index_,
                latest_pose_.translation().x(),
                latest_pose_.translation().y(),
                latest_pose_.translation().z(),
                gps_meas.size(), lidar_meas.size());

    ++pose_index_;
  }

  // -------------------------------------------------------
  // Members
  // -------------------------------------------------------
  std::mutex mtx_;

  // Measurement buffers
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> gps_buffer_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> lidar_buffer_;

  // GTSAM
  std::unique_ptr<gtsam::ISAM2> isam2_;
  size_t pose_index_ = 0;
  gtsam::Pose3 latest_pose_;

  // Parameters
  double between_noise_trans_ = 1.0;
  double between_noise_rot_ = 0.1;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Path accumulation
  nav_msgs::msg::Path path_;
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
