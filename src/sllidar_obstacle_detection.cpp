#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"


#define DEGREES(X_RAD)   (X_RAD * (180.0/M_PI))

static double step2deg(int step_val, float rad_per_step) {
    double deg_per_step = DEGREES(rad_per_step);
    double res_deg = (step_val * deg_per_step);
    if (res_deg >= 180.0) {
        res_deg = res_deg-360;
    }
    return (-res_deg);
}

class RPLidarObstacleDetection: public rclcpp::Node 
{
    public:
        RPLidarObstacleDetection(): Node("lidar_obstacle_detection") {
            this->declare_parameter("thresh_dist", 0.1);
            this->declare_parameter("fov_angle_min", -30.0);
            this->declare_parameter("fov_angle_max", 30.0);

            thresh_dist = this->get_parameter("thresh_dist").as_double();
            fov_ang_min = this->get_parameter("fov_angle_min").as_double();
            fov_ang_max = this->get_parameter("fov_angle_max").as_double();

            this->subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                10,
                std::bind(&RPLidarObstacleDetection::laser_scan_callback, this, std::placeholders::_1)
            );

            this->publisher_ = this->create_publisher<std_msgs::msg::Bool>(
                "obstacle_detected",
                10
            );
        }
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        double thresh_dist;
        double fov_ang_min;
        double fov_ang_max;

        void laser_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
            std_msgs::msg::Bool obstacle_msg;
            obstacle_msg.data = false;
            
            for (size_t i=0; i<(laser_scan->ranges.size()); i++) {
                double cur_deg = step2deg(i, laser_scan->angle_increment);
                double cur_dist = (laser_scan->ranges[i]);

                if ((cur_deg >= fov_ang_min) && (cur_deg <= fov_ang_max) && (cur_dist <= thresh_dist)) {
                    RCLCPP_INFO(get_logger(), "Obstacle Detected at %.2fdeg, %.2fm", cur_deg, laser_scan->ranges[i]);
                    obstacle_msg.data = true;
                    break;
                }
            }

            if (obstacle_msg.data == false) {
                RCLCPP_INFO(get_logger(), "Path clear, No Obstacle :)");
            }

            this->publisher_->publish(obstacle_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPLidarObstacleDetection>());
    rclcpp::shutdown();
    return 0;
}