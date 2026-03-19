// GTSAM-based pose estimator with selective factor removal and re-optimization
//
// Architecture:
//   Normal mode:  ISAM2 incremental optimization at 500ms (fast)
//   Removal mode: When "/remove_factors" receives "GPS" or "LIDAR",
//                 removes all measurement factors of that type, batch
//                 re-optimizes with LevenbergMarquardt, and resets ISAM2.
//
// The shadow graph tracks all factors with metadata, enabling selective
// removal and full graph reconstruction at any time.
//
// BetweenFactor noise scales with sqrt(dt) (random walk model).

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <mutex>
#include <optional>
#include <vector>
#include <map>

using gtsam::symbol_shorthand::X;  // Pose3 variables: X(0), X(1), ...

// ---------------------------------------------------------------------------
// Utility: ROS <-> GTSAM conversions
// ---------------------------------------------------------------------------
// ROS covariance order : [tx, ty, tz, rx, ry, rz]
// GTSAM tangent space   : [rx, ry, rz, tx, ty, tz]

static gtsam::Pose3 rosPoseToGtsam(const geometry_msgs::msg::Pose & p)
{
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(
          p.orientation.w, p.orientation.x,
          p.orientation.y, p.orientation.z),
      gtsam::Point3(p.position.x, p.position.y, p.position.z));
}

static gtsam::SharedNoiseModel rosCovarianceToGtsamNoise(
    const std::array<double, 36> & cov_ros)
{
  Eigen::Matrix<double, 6, 6> cov_ros_mat;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      cov_ros_mat(i, j) = cov_ros[i * 6 + j];

  // Reorder: swap translation/rotation blocks
  Eigen::Matrix<double, 6, 6> cov_gtsam;
  cov_gtsam.block<3, 3>(0, 0) = cov_ros_mat.block<3, 3>(3, 3);  // RR
  cov_gtsam.block<3, 3>(0, 3) = cov_ros_mat.block<3, 3>(3, 0);  // RT
  cov_gtsam.block<3, 3>(3, 0) = cov_ros_mat.block<3, 3>(0, 3);  // TR
  cov_gtsam.block<3, 3>(3, 3) = cov_ros_mat.block<3, 3>(0, 0);  // TT
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
// Factor metadata for selective removal
// ---------------------------------------------------------------------------

struct FactorRecord {
  enum class Type {
    ANCHOR_PRIOR,       // Initial prior on first node (never removed)
    BETWEEN,            // Between consecutive poses (never removed)
    GPS_MEASUREMENT,    // GPS PriorFactor (removable)
    LIDAR_MEASUREMENT   // LiDAR PriorFactor (removable)
  };

  Type type;
  rclcpp::Time stamp;
  gtsam::Key primary_key;
};

// ---------------------------------------------------------------------------
// Internal stamped measurement
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
    retention_duration_s_ =
        this->declare_parameter("retention_duration_s", 300.0);

    // ISAM2
    isam2_params_.relinearizeThreshold = 0.1;
    isam2_params_.relinearizeSkip = 1;
    isam2_ = std::make_unique<gtsam::ISAM2>(isam2_params_);

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

    // Factor removal trigger: publish "GPS" or "LIDAR" to remove those factors
    removal_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/remove_factors", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          pending_removal_ = msg->data;
        });

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/estimated_path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Optimization timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&PoseEstimatorNode::optimizationCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "PoseEstimator started (period=%dms, retention=%.0fs)",
                period_ms, retention_duration_s_);
  }

private:
  // -----------------------------------------------------------
  // Main optimization loop (called at 500ms)
  // -----------------------------------------------------------
  void optimizationCallback()
  {
    // --- Grab buffered data under lock ---
    std::optional<std::string> removal;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> gps_meas;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> lidar_meas;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      removal = pending_removal_;
      pending_removal_.reset();
      gps_meas.swap(gps_buffer_);
      lidar_meas.swap(lidar_buffer_);
    }

    // --- Handle factor removal before processing new data ---
    if (removal.has_value()) {
      handleFactorRemoval(*removal);
    }

    if (gps_meas.empty() && lidar_meas.empty()) {
      return;
    }

    // --- Convert to StampedMeasurement, sort by time ---
    std::vector<StampedMeasurement> measurements;
    measurements.reserve(gps_meas.size() + lidar_meas.size());

    for (const auto & m : gps_meas) {
      measurements.push_back({
          m.header.stamp, StampedMeasurement::Source::GPS,
          rosPoseToGtsam(m.pose.pose),
          rosCovarianceToGtsamNoise(m.pose.covariance)});
    }
    for (const auto & m : lidar_meas) {
      measurements.push_back({
          m.header.stamp, StampedMeasurement::Source::LIDAR,
          rosPoseToGtsam(m.pose.pose),
          rosCovarianceToGtsamNoise(m.pose.covariance)});
    }

    std::sort(measurements.begin(), measurements.end(),
        [](const StampedMeasurement & a, const StampedMeasurement & b) {
          return a.stamp < b.stamp;
        });

    // --- Build incremental factor graph ---
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    std::vector<FactorRecord> new_records;

    gtsam::Pose3 prev_estimate = latest_pose_;
    gtsam::Key attach_key =
        (pose_index_ > 0) ? X(pose_index_ - 1) : gtsam::Key(0);
    bool have_node = (pose_index_ > 0);

    struct NewNode { gtsam::Key key; rclcpp::Time stamp; };
    std::vector<NewNode> new_nodes;
    rclcpp::Time latest_stamp;
    size_t gps_count = 0, lidar_count = 0;

    for (const auto & meas : measurements) {
      bool need_new_node = false;

      if (!have_node) {
        need_new_node = true;
      } else {
        double dt = (meas.stamp - prev_stamp_).seconds();
        if (dt < 0.0) {
          RCLCPP_WARN(this->get_logger(),
              "Backward timestamp (dt=%.4fs), skipping", dt);
          continue;
        }
        if (dt >= min_dt_for_new_node_) {
          need_new_node = true;
        }
      }

      if (need_new_node) {
        gtsam::Key new_key = X(pose_index_);

        if (!have_node) {
          // Anchor prior on first node (never removed)
          new_factors.addPrior(new_key, meas.pose, meas.noise);
          new_records.push_back(
              {FactorRecord::Type::ANCHOR_PRIOR, meas.stamp, new_key});
        } else {
          // BetweenFactor with sqrt(dt)-scaled noise (random walk)
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
          new_records.push_back(
              {FactorRecord::Type::BETWEEN, meas.stamp, new_key});
        }

        new_values.insert(new_key, meas.pose);
        attach_key = new_key;
        have_node = true;
        prev_stamp_ = meas.stamp;
        prev_estimate = meas.pose;
        key_timestamps_[new_key] = meas.stamp;
        new_nodes.push_back({new_key, meas.stamp});
        ++pose_index_;
      }

      // Measurement PriorFactor on the current node
      auto factor_type = (meas.source == StampedMeasurement::Source::GPS)
          ? FactorRecord::Type::GPS_MEASUREMENT
          : FactorRecord::Type::LIDAR_MEASUREMENT;
      new_factors.addPrior(attach_key, meas.pose, meas.noise);
      new_records.push_back({factor_type, meas.stamp, attach_key});
      latest_stamp = meas.stamp;

      if (meas.source == StampedMeasurement::Source::GPS) ++gps_count;
      else ++lidar_count;
    }

    if (new_factors.empty()) {
      return;
    }

    // --- ISAM2 incremental update ---
    isam2_->update(new_factors, new_values);
    isam2_->update();  // extra iteration for convergence

    // --- Track in shadow graph ---
    shadow_graph_.push_back(new_factors);
    shadow_values_.insert(new_values);
    factor_records_.insert(
        factor_records_.end(), new_records.begin(), new_records.end());

    // --- Extract result ---
    gtsam::Values result = isam2_->calculateEstimate();
    latest_pose_ = result.at<gtsam::Pose3>(attach_key);

    // --- Publish pose + TF ---
    publishPoseAndTF(latest_stamp);

    // --- Append new nodes to path ---
    for (const auto & node : new_nodes) {
      gtsam::Pose3 p = result.at<gtsam::Pose3>(node.key);
      path_.poses.push_back(gtsamPoseToRos(p, node.stamp, "map"));
    }
    path_.header.stamp = latest_stamp;
    path_.header.frame_id = "map";
    path_pub_->publish(path_);

    RCLCPP_INFO(this->get_logger(),
                "[nodes=%zu factors=%zu] pose=(%.2f, %.2f, %.2f) "
                "GPS=%zu LiDAR=%zu new_nodes=%zu",
                pose_index_, shadow_graph_.size(),
                latest_pose_.translation().x(),
                latest_pose_.translation().y(),
                latest_pose_.translation().z(),
                gps_count, lidar_count, new_nodes.size());
  }

  // -----------------------------------------------------------
  // Factor removal and batch re-optimization
  // -----------------------------------------------------------
  void handleFactorRemoval(const std::string & source)
  {
    FactorRecord::Type target;
    if (source == "GPS") {
      target = FactorRecord::Type::GPS_MEASUREMENT;
    } else if (source == "LIDAR") {
      target = FactorRecord::Type::LIDAR_MEASUREMENT;
    } else {
      RCLCPP_WARN(this->get_logger(),
          "Unknown removal target: '%s' (use \"GPS\" or \"LIDAR\")",
          source.c_str());
      return;
    }

    if (pose_index_ == 0) {
      RCLCPP_WARN(this->get_logger(), "No graph to modify yet");
      return;
    }

    // Build filtered graph: keep everything except the target type
    gtsam::NonlinearFactorGraph filtered_graph;
    std::vector<FactorRecord> filtered_records;
    size_t removed_count = 0;

    for (size_t i = 0; i < factor_records_.size(); ++i) {
      if (factor_records_[i].type == target) {
        ++removed_count;
        continue;
      }
      filtered_records.push_back(factor_records_[i]);
      filtered_graph.add(shadow_graph_[i]);
    }

    if (removed_count == 0) {
      RCLCPP_INFO(this->get_logger(),
          "No %s factors found to remove", source.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Removing %zu %s factors, batch re-optimizing %zu remaining factors...",
        removed_count, source.c_str(), filtered_graph.size());

    // Use current ISAM2 estimate as initial values for LM
    gtsam::Values current_estimate = isam2_->calculateEstimate();
    gtsam::LevenbergMarquardtOptimizer optimizer(
        filtered_graph, current_estimate);
    gtsam::Values optimized = optimizer.optimize();

    // Reset ISAM2 with the filtered graph and optimized values
    isam2_ = std::make_unique<gtsam::ISAM2>(isam2_params_);
    isam2_->update(filtered_graph, optimized);
    isam2_->update();  // extra iteration

    // Update shadow state
    shadow_graph_ = filtered_graph;
    shadow_values_ = optimized;
    factor_records_ = filtered_records;

    // Update latest pose
    latest_pose_ = optimized.at<gtsam::Pose3>(X(pose_index_ - 1));

    // Rebuild and publish path
    rebuildPath(optimized);

    RCLCPP_INFO(this->get_logger(),
        "Re-optimization complete: %zu factors remaining, "
        "pose=(%.2f, %.2f, %.2f)",
        filtered_graph.size(),
        latest_pose_.translation().x(),
        latest_pose_.translation().y(),
        latest_pose_.translation().z());
  }

  // -----------------------------------------------------------
  // Rebuild path from all pose nodes
  // -----------------------------------------------------------
  void rebuildPath(const gtsam::Values & values)
  {
    path_.poses.clear();

    // Collect and sort keys by timestamp
    std::vector<std::pair<gtsam::Key, rclcpp::Time>> sorted_keys(
        key_timestamps_.begin(), key_timestamps_.end());
    std::sort(sorted_keys.begin(), sorted_keys.end(),
        [](const auto & a, const auto & b) {
          return a.second < b.second;
        });

    for (const auto & [key, stamp] : sorted_keys) {
      if (values.exists(key)) {
        path_.poses.push_back(
            gtsamPoseToRos(values.at<gtsam::Pose3>(key), stamp, "map"));
      }
    }

    if (!path_.poses.empty()) {
      path_.header.stamp = path_.poses.back().header.stamp;
      path_.header.frame_id = "map";
      path_pub_->publish(path_);
    }
  }

  // -----------------------------------------------------------
  // Publish current pose estimate and TF
  // -----------------------------------------------------------
  void publishPoseAndTF(const rclcpp::Time & stamp)
  {
    auto pose_msg = gtsamPoseToRos(latest_pose_, stamp, "map");
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose_msg.pose.position.x;
    tf.transform.translation.y = pose_msg.pose.position.y;
    tf.transform.translation.z = pose_msg.pose.position.z;
    tf.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  // -----------------------------------------------------------
  // Members
  // -----------------------------------------------------------
  std::mutex mtx_;

  // Sensor buffers (protected by mtx_)
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> gps_buffer_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> lidar_buffer_;
  std::optional<std::string> pending_removal_;

  // GTSAM core
  gtsam::ISAM2Params isam2_params_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  size_t pose_index_ = 0;
  gtsam::Pose3 latest_pose_;
  rclcpp::Time prev_stamp_;

  // Shadow graph: full history for batch rebuild on factor removal
  gtsam::NonlinearFactorGraph shadow_graph_;
  gtsam::Values shadow_values_;
  std::vector<FactorRecord> factor_records_;
  std::map<gtsam::Key, rclcpp::Time> key_timestamps_;

  // Parameters
  double between_noise_trans_ = 1.0;
  double between_noise_rot_ = 0.1;
  double min_dt_for_new_node_ = 0.001;
  double retention_duration_s_ = 300.0;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr removal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

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
