#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <limits>
#include <vector>

class Map3DTo2D : public rclcpp::Node {
public:
  Map3DTo2D() : Node("map_3d_to_2d") {

    resolution_ = this->declare_parameter("resolution", 0.05);
    z_min_ = this->declare_parameter("z_min", -0.5);
    z_max_ = this->declare_parameter("z_max", 1.5);
    threshold_ = this->declare_parameter("occupancy_threshold", 2);
    frame_id_ = this->declare_parameter("frame_id", "map");
    padding_ = this->declare_parameter("padding", 1.0);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/saved_map", 10,
        std::bind(&Map3DTo2D::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map2d", 10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Z filtering
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);

    pcl::PointCloud<pcl::PointXYZ> filtered;
    pass.filter(filtered);

    if (filtered.empty()) {
      RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty");
      return;
    }

    // -------------------------------
    // Compute bounds (only once)
    // -------------------------------
    if (!map_initialized_) {

      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();

      for (const auto &pt : filtered.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
          continue;

        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
      }

      // padding
      min_x -= padding_;
      max_x += padding_;
      min_y -= padding_;
      max_y += padding_;

      origin_x_ = min_x;
      origin_y_ = min_y;

      width_ = static_cast<int>((max_x - min_x) / resolution_);
      height_ = static_cast<int>((max_y - min_y) / resolution_);

      if (width_ <= 0 || height_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid map size");
        return;
      }

      map_initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "Map initialized: %d x %d", width_,
                  height_);
    }

    // -------------------------------
    // Build grid
    // -------------------------------
    std::vector<int> grid(width_ * height_, 0);

    for (const auto &pt : filtered.points) {
      int x = static_cast<int>((pt.x - origin_x_) / resolution_);
      int y = static_cast<int>((pt.y - origin_y_) / resolution_);

      if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid[y * width_ + x]++;
      }
    }

    // -------------------------------
    // Fill OccupancyGrid
    // -------------------------------
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = now();
    map.header.frame_id = frame_id_;

    map.info.resolution = resolution_;
    map.info.width = width_;
    map.info.height = height_;
    map.info.origin.position.x = origin_x_;
    map.info.origin.position.y = origin_y_;

    // initialize as unknown
    map.data.assign(width_ * height_, -1);

    for (size_t i = 0; i < grid.size(); i++) {
      if (grid[i] >= threshold_) {
        map.data[i] = 100; // occupied
      }
    }

    pub_->publish(map);
  }

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

  // Parameters
  float resolution_;
  float z_min_, z_max_;
  int threshold_;
  std::string frame_id_;
  float padding_;

  // Map state
  bool map_initialized_ = false;
  int width_, height_;
  float origin_x_, origin_y_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map3DTo2D>());
  rclcpp::shutdown();
  return 0;
}
