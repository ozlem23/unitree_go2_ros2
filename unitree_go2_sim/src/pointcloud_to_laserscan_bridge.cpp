#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <vector>
#include <limits>

class PointCloudToLaserScanBridge : public rclcpp::Node
{
public:
    PointCloudToLaserScanBridge() : Node("pc2_to_laserscan_bridge")
    {
        // Girdi olarak 3D LiDAR (Velodyne) verisini dinliyoruz
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                            .best_effort()
                            .durability_volatile();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points/points", 
            sensor_qos,
            std::bind(&PointCloudToLaserScanBridge::listener_callback, this, std::placeholders::_1));

        // Çıktı olarak 2D Lazer (/scan) basıyoruz (RViz ve SLAM uyumlu QoS)
        auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                            .reliable()                  
                            .transient_local();          

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", scan_qos);
        RCLCPP_INFO(this->get_logger(), "🚀 LiDAR Köprüsü Matematiksel Katman Filtresi ile Yeniden Ayağa Kalktı.");
    }

private:
void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    if (cloud_msg->data.empty()) return;

    auto laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    
    laser_msg->header.stamp = cloud_msg->header.stamp; 
    // Sabit velodyne yerine, robot sallansa bile izdüşümü düzgün kalması için base_link yapıyoruz
    laser_msg->header.frame_id = "base_link"; 

    laser_msg->angle_min = -M_PI;
    laser_msg->angle_max = M_PI;
    laser_msg->angle_increment = 0.007; 
    
    laser_msg->scan_time = 0.033;
    int num_readings = std::floor((laser_msg->angle_max - laser_msg->angle_min) / laser_msg->angle_increment) + 1;
    laser_msg->time_increment = laser_msg->scan_time / static_cast<float>(num_readings);

    laser_msg->range_min = 0.35; 
    laser_msg->range_max = 30.0;

    // Başlangıçta tüm diziyi sonsuz (boşluk) yapıyoruz
    laser_msg->ranges.resize(num_readings, std::numeric_limits<float>::infinity()); 

    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : cloud_msg->fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
        if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1) return;

    for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i) {
        size_t pixel_offset = i * cloud_msg->point_step;
        
        float x = *reinterpret_cast<const float*>(&cloud_msg->data[pixel_offset + x_offset]);
        float y = *reinterpret_cast<const float*>(&cloud_msg->data[pixel_offset + y_offset]);
        float z = *reinterpret_cast<const float*>(&cloud_msg->data[pixel_offset + z_offset]);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

        float R = std::sqrt(x*x + y*y + z*z);
        if (R < laser_msg->range_min) continue; 

        float vertical_angle = std::asin(z / R);

        // Şüphen doğrultusunda burayı bıçak gibi daralttık (Robot sarsılsa da sadece tam yatayı alır)
        if (vertical_angle < -0.02 || vertical_angle > 0.02) {
            continue; 
        }

        float range_val = std::sqrt(x*x + y*y);

        if (range_val >= laser_msg->range_min && range_val <= laser_msg->range_max) {
            float angle = std::atan2(y, x);
            int index = std::floor((angle - laser_msg->angle_min) / laser_msg->angle_increment);

            if (index >= 0 && index < num_readings) {
                if (range_val < laser_msg->ranges[index]) {
                    laser_msg->ranges[index] = range_val;
                }
            }
        }
    }

    // ❌ O TEHLİKELİ ARDIŞIK DOĞRULAMA DÖNGÜSÜNÜ TAMAMEN KALDIRDIK!
    // ROS ve SLAM standartlarına göre boşluklar .inf kalmalı ki SLAM orayı "boş alan" olarak haritaya işleyebilsen.

    publisher_->publish(*laser_msg);
}

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToLaserScanBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}