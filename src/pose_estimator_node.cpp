// GTSAM-based pose estimator fusing GPS and LiDAR measurements
// Event-driven node creation: each sensor measurement timestamp gets its own pose node
// BetweenFactor noise scales with sqrt(dt) (random walk model)

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

#include <algorithm>
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
// Stamped measurement: unified representation for GPS and LiDAR
// ---------------------------------------------------------------------------

struct StampedMeasurement {
  rclcpp::Time stamp;
  enum class Source { GPS, LIDAR } source;
  gtsam::Pose3 pose;
  gtsam::SharedNoiseModel noise;
};

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
    min_dt_for_new_node_ =
        this->declare_parameter("min_dt_for_new_node", 0.001);

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
                "PoseEstimator started (period=%dms, min_dt=%.4fs)",
                period_ms, min_dt_for_new_node_);
  }

private:
  // -------------------------------------------------------
  // Core optimization routine (called periodically)
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

    // ---- Convert all measurements to StampedMeasurement, sort by time ----
    std::vector<StampedMeasurement> measurements;
    measurements.reserve(gps_meas.size() + lidar_meas.size());

    for (const auto & m : gps_meas) {
      measurements.push_back({
        m.header.stamp,
        StampedMeasurement::Source::GPS,
        rosPoseToGtsam(m.pose.pose),
        rosCovarianceToGtsamNoise(m.pose.covariance)
      });
    }
    for (const auto & m : lidar_meas) {
      measurements.push_back({
        m.header.stamp,
        StampedMeasurement::Source::LIDAR,
        rosPoseToGtsam(m.pose.pose),
        rosCovarianceToGtsamNoise(m.pose.covariance)
      });
    }

    std::sort(measurements.begin(), measurements.end(),
        [](const StampedMeasurement & a, const StampedMeasurement & b) {
          return a.stamp < b.stamp;
        });

    // ---- Build factor graph incrementally ----
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;

    // Previous node's initial estimate (for BetweenFactor delta computation)
    gtsam::Pose3 prev_estimate = latest_pose_;

    // The key we attach measurement PriorFactors to
    gtsam::Key attach_key;
    bool have_node = (pose_index_ > 0);
    if (have_node) {
      attach_key = X(pose_index_ - 1);
    }

    // Track new nodes for path publishing
    struct NewNode { gtsam::Key key; rclcpp::Time stamp; };
    std::vector<NewNode> new_nodes;
    rclcpp::Time latest_stamp;
    size_t gps_count = 0;
    size_t lidar_count = 0;

    for (const auto & meas : measurements) {
      bool need_new_node = false;

      if (!have_node) {
        // No node exists yet — must create the first one
        need_new_node = true;
      } else {
        double dt = (meas.stamp - prev_stamp_).seconds();
        if (dt < 0.0) {
          RCLCPP_WARN(this->get_logger(),
              "Timestamp went backwards (dt=%.4fs), skipping measurement", dt);
          continue;
        }
        if (dt >= min_dt_for_new_node_) {
          need_new_node = true;
        }
      }

      if (need_new_node) {
        gtsam::Key new_key = X(pose_index_);
        new_values.insert(new_key, meas.pose);

        if (!have_node) {
          // First node ever: anchor prior
          new_factors.addPrior(new_key, meas.pose, meas.noise);
        } else {
          // BetweenFactor with sqrt(dt)-scaled noise (random walk model)
          double dt = (meas.stamp - prev_stamp_).seconds();
          double dt_sqrt = std::sqrt(std::max(dt, 1e-6));
          auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(6) <<
                  between_noise_rot_ * dt_sqrt,
                  between_noise_rot_ * dt_sqrt,
                  between_noise_rot_ * dt_sqrt,
                  between_noise_trans_ * dt_sqrt,
                  between_noise_trans_ * dt_sqrt,
                  between_noise_trans_ * dt_sqrt
              ).finished());

          gtsam::Pose3 delta = prev_estimate.between(meas.pose);
          new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
              attach_key, new_key, delta, between_noise);
        }

        attach_key = new_key;
        have_node = true;
        prev_stamp_ = meas.stamp;
        prev_estimate = meas.pose;
        new_nodes.push_back({new_key, meas.stamp});
        ++pose_index_;
      }

      // Measurement PriorFactor on the current node
      new_factors.addPrior(attach_key, meas.pose, meas.noise);
      latest_stamp = meas.stamp;

      if (meas.source == StampedMeasurement::Source::GPS) {
        ++gps_count;
      } else {
        ++lidar_count;
      }
    }

    // All measurements may have been skipped (backward timestamps)
    if (new_factors.empty()) {
      return;
    }

    // ---- Run ISAM2 update (single batch) ----
    isam2_->update(new_factors, new_values);
    isam2_->update();  // extra iteration for convergence

    // ---- Extract result ----
    gtsam::Values result = isam2_->calculateEstimate();
    latest_pose_ = result.at<gtsam::Pose3>(attach_key);

    // ---- Publish ----
    // Use measurement timestamp (not wall clock)
    auto pose_msg = gtsamPoseToRos(latest_pose_, latest_stamp, "map");
    pose_pub_->publish(pose_msg);

    // Path: append all newly created nodes
    for (const auto & node : new_nodes) {
      gtsam::Pose3 p = result.at<gtsam::Pose3>(node.key);
      path_.poses.push_back(gtsamPoseToRos(p, node.stamp, "map"));
    }
    path_.header.stamp = latest_stamp;
    path_.header.frame_id = "map";
    path_pub_->publish(path_);

    // TF: map -> base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = latest_stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose_msg.pose.position.x;
    tf.transform.translation.y = pose_msg.pose.position.y;
    tf.transform.translation.z = pose_msg.pose.position.z;
    tf.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    RCLCPP_INFO(this->get_logger(),
                "[total_nodes=%zu] Optimized pose: (%.2f, %.2f, %.2f)  "
                "GPS=%zu LiDAR=%zu new_nodes=%zu",
                pose_index_,
                latest_pose_.translation().x(),
                latest_pose_.translation().y(),
                latest_pose_.translation().z(),
                gps_count, lidar_count, new_nodes.size());
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
  rclcpp::Time prev_stamp_;  // timestamp of the most recently created node

  // Parameters
  double between_noise_trans_ = 1.0;   // noise density (sigma per sqrt-second)
  double between_noise_rot_ = 0.1;     // noise density (sigma per sqrt-second)
  double min_dt_for_new_node_ = 0.001; // minimum dt (seconds) to create a new node

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
