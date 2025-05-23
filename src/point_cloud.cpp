#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class DepthToPointCloudNode : public rclcpp::Node {
public:
  DepthToPointCloudNode()
  : Node("depth_to_pointcloud_node")
  {
    // QoS設定
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    // message_filters::Subscriberはshared_ptrで管理し、コンストラクタで初期化
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/depth/image_raw", qos.get_rmw_qos_profile());

    info_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
      this, "/depth/camera_info", qos.get_rmw_qos_profile());

    // TimeSynchronizerを作成し、コールバックを登録
    sync_ = std::make_shared<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(*depth_sub_, *info_sub_, 10);
    sync_->registerCallback(std::bind(&DepthToPointCloudNode::callback, this,
                                     std::placeholders::_1, std::placeholders::_2));

    // PointCloud2パブリッシャー作成
    // pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points", qos);
    // rclcpp::QoS qos(rclcpp::KeepLast(10));
qos.reliable();  // 明示的にRELIABLEにする

pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points", qos);


    // 静的TFブロードキャスター
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_tf();
  }

private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Depth and CameraInfo received");

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat &depth = cv_ptr->image;

    float fx = info_msg->k[0];
    float fy = info_msg->k[4];
    float cx = info_msg->k[2];
    float cy = info_msg->k[5];

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = depth_msg->header;
    cloud_msg.header.frame_id = "camera_link";
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;

    std::vector<std::array<float, 3>> points;
    for (int v = 0; v < depth.rows; ++v) {
      for (int u = 0; u < depth.cols; ++u) {
        uint16_t d = depth.at<uint16_t>(v, u);
        if (d == 0) continue;

        float z = static_cast<float>(d) * 0.001f;  // mm→mに変換
        float x = (u - cx) * z / fx;
        float y = (v - cy) * z / fy;

        points.push_back({x, y, z});
      }
    }

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto &pt : points) {
      *iter_x = pt[0]; ++iter_x;
      *iter_y = pt[1]; ++iter_y;
      *iter_z = pt[2]; ++iter_z;
    }

    cloud_msg.width = static_cast<uint32_t>(points.size());
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;

    pub_->publish(cloud_msg);
  }

  void publish_static_tf()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "camera_link";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> info_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthToPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



