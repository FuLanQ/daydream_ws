#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <vector>
#include <cstring>
#include <thread>
#include <cmath>
#include <chrono>

// ROS 2 消息类型
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

// 接收单片机上传的数据包结构体 (严格1字节对齐)
#pragma pack(push, 1)
struct RobotData {
    float accel_x; float accel_y; float accel_z;
    float gyro_x;  float gyro_y;  float gyro_z;
    float lf_speed; float rf_speed; float lb_speed; float rb_speed;
};
#pragma pack(pop)

class SerialTestNode : public rclcpp::Node {
public:
    SerialTestNode() : Node("daydream_base_node"), io_context_(), serial_(io_context_) {
        // =====================================
        // 1. 声明并读取 Launch 文件参数
        // =====================================
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 921600);
        this->declare_parameter<bool>("publish_tf", false);
        this->declare_parameter<int>("cmd_timeout_ms", 500);
        this->declare_parameter<double>("max_linear_vel", 1.5);
        this->declare_parameter<double>("max_angular_vel", 3.0);
        
        // 这里的 min_wheel_vel 保留参数读取，防止你的老 launch 文件报错，但后续计算中不再使用
        this->declare_parameter<double>("min_wheel_vel", 500.0); 

        port_name_       = this->get_parameter("port").as_string();
        baudrate_        = this->get_parameter("baudrate").as_int();
        publish_tf_      = this->get_parameter("publish_tf").as_bool();
        cmd_timeout_ms_  = this->get_parameter("cmd_timeout_ms").as_int();
        max_linear_vel_  = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        min_wheel_vel_   = this->get_parameter("min_wheel_vel").as_double(); 
        
        // =====================================
        // 2. 注册话题发布者和订阅者
        // =====================================
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 50);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", 50);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 50, std::bind(&SerialTestNode::cmdVelCallback, this, std::placeholders::_1));

        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        // =====================================
        // 3. 安全守护：命令超时断电定时器
        // =====================================
        timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&SerialTestNode::checkTimeoutData, this));

        last_cmd_time_ = this->now();
        last_time_ = this->now();

        // =====================================
        // 4. 初始化与打开串口
        // =====================================
        try {
            serial_.open(port_name_);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            // ⭐ 提示文本修改：告知开发者死区已剥离
            RCLCPP_INFO(this->get_logger(), "底层驱动已启动 | 端口: %s | 目标轮速绝对诚实下发 (已移除上位机死区)", 
                        port_name_.c_str());
            
            // 启动独立串口接收线程
            receive_thread_ = std::thread(&SerialTestNode::receiveLoop, this);
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", e.what());
        }
    }

    ~SerialTestNode() {
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_;
    std::thread receive_thread_;
    std::vector<uint8_t> buffer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    std::string port_name_;
    int baudrate_;
    bool publish_tf_;
    int cmd_timeout_ms_;
    double max_linear_vel_;
    double max_angular_vel_;
    double min_wheel_vel_; 

    bool is_stopped_ = true;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_th_ = 0.0;
    
    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_time_;

    // 根据车身 PDF 尺寸推导绝对轮距: 轮距X/2 + 轮距Y/2
    const float wheel_sep_x_ = 0.172f / 2.0f; 
    const float wheel_sep_y_ = 0.198f / 2.0f; 
    const float factor_      = wheel_sep_x_ + wheel_sep_y_; 

    // --- 独立接收线程：实时读取串口缓存 ---
    void receiveLoop() {
        uint8_t temp_buf[256];
        while (rclcpp::ok()) {
            try {
                size_t len = serial_.read_some(boost::asio::buffer(temp_buf));
                if (len > 0) {
                    buffer_.insert(buffer_.end(), temp_buf, temp_buf + len);
                    processBuffer();
                }
            } catch (boost::system::system_error& e) {
                rclcpp::sleep_for(1s);
            }
        }
    }

    // --- 数据帧粘包处理与校验 ---
    void processBuffer() {
        const size_t FRAME_LEN = 45; // 0xAA 0x55(2) + len(1) + data(40) + sum(1) + 0x0D(1) = 45
        while (buffer_.size() >= FRAME_LEN) {
            // 寻找帧头和长度字节
            if (buffer_[0] == 0xAA && buffer_[1] == 0x55 && buffer_[2] == 40) {
                uint8_t sum = 0;
                for (int i = 3; i < 43; i++) {
                    sum += buffer_[i];
                }

                // 校验和校验以及帧尾校验
                if (sum == buffer_[43] && buffer_[44] == 0x0D) {
                    RobotData data;
                    memcpy(&data, &buffer_[3], 40);
                    publishData(data); 
                    // 处理完一帧，从缓存中删掉这45个字节
                    buffer_.erase(buffer_.begin(), buffer_.begin() + FRAME_LEN);
                } else {
                    // 校验失败，可能是个假包头，丢弃1字节，继续往后找
                    buffer_.erase(buffer_.begin(), buffer_.begin() + 1);
                }
            } else {
                // 不是帧头，丢弃1字节向前推进
                buffer_.erase(buffer_.begin(), buffer_.begin() + 1);
            }
        }
    }

    // --- ⭐⭐⭐ 核心修改区：发送下位机速度 (移除了死区补偿) ---
    void sendVelToMCU(float vx, float vy, float wz) {
        // 1. 整体底盘速度安全上限限幅
        double linear_speed = std::hypot(vx, vy);
        if (linear_speed > max_linear_vel_) {
            vx = max_linear_vel_ * (vx / linear_speed);
            vy = max_linear_vel_ * (vy / linear_speed);
        }
        if (wz > max_angular_vel_) wz = max_angular_vel_;
        if (wz < -max_angular_vel_) wz = -max_angular_vel_;

        // 2. 将 m/s 转换为符合麦克纳姆轮底盘的理论各轮轮速 mm/s
        float lf = (vx - vy - wz * factor_) * 1000.0f;
        float rf = (vx + vy + wz * factor_) * 1000.0f;
        float lb = (vx + vy - wz * factor_) * 1000.0f;
        float rb = (vx - vy + wz * factor_) * 1000.0f;

        // 3. 基础安全钳位保护 (不再进行虚假的死区映射放大)
        const float MAX_SPEED_MCU = 5500.0f; // 保护极值极限
        auto limit_speed = [&](float speed) -> float {
            if (speed > MAX_SPEED_MCU) return MAX_SPEED_MCU;
            if (speed < -MAX_SPEED_MCU) return -MAX_SPEED_MCU;
            return speed; // 回归最本质的原汁原味目标速度！
        };

        lf = limit_speed(lf);
        rf = limit_speed(rf);
        lb = limit_speed(lb);
        rb = limit_speed(rb);

        // 4. 组装数据通过串口发送给下位机
        uint8_t tx_buf[21];
        tx_buf[0] = 0xAA;
        tx_buf[1] = 0x55;
        tx_buf[2] = 16;  // 4个float = 16字节
        float speeds[4] = {lf, rf, lb, rb};
        memcpy(&tx_buf[3], speeds, 16); 
        
        uint8_t sum = 0;
        for (int i = 3; i < 19; i++) {
            sum += tx_buf[i];
        }
        tx_buf[19] = sum;
        tx_buf[20] = 0x0D;

        boost::asio::write(serial_, boost::asio::buffer(tx_buf, 21));
    }

    // --- ROS cmd_vel 回调 ---
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->now();
        is_stopped_ = false;
        sendVelToMCU(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    // --- 安全检测：命令超时立刻停车 ---
    void checkTimeoutData() {
        if (!is_stopped_) {
            auto current_time = this->now();
            if ((current_time - last_cmd_time_).seconds() * 1000.0 > cmd_timeout_ms_) {
                sendVelToMCU(0.0, 0.0, 0.0);
                is_stopped_ = true;
            }
        }
    }

    // --- 单片机数据解包后，推导 ODOM 与 TF，发布 ROS 话题 ---
    void publishData(const RobotData& data) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // ===================================
        // A. 发布 IMU 数据 (融合 EKF)
        // ===================================
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = "base_link"; 
        
        imu_msg.linear_acceleration.x = data.accel_x;
        imu_msg.linear_acceleration.y = data.accel_y;
        imu_msg.linear_acceleration.z = data.accel_z;
        
        double deg_to_rad = M_PI / 180.0;
        imu_msg.angular_velocity.x = data.gyro_x * deg_to_rad;
        imu_msg.angular_velocity.y = data.gyro_y * deg_to_rad;
        imu_msg.angular_velocity.z = data.gyro_z * deg_to_rad;
        
        imu_msg.orientation.x = 0; imu_msg.orientation.y = 0; 
        imu_msg.orientation.z = 0; imu_msg.orientation.w = 1;
        imu_msg.orientation_covariance[0] = -1.0; 
        
        imu_msg.angular_velocity_covariance[0] = 1e-4;
        imu_msg.angular_velocity_covariance[4] = 1e-4;
        imu_msg.angular_velocity_covariance[8] = 1e-4; 
        
        imu_pub_->publish(imu_msg);

        // ===================================
        // B. 正运动学解算 ODOM 航迹推演
        // ===================================
        double scale = 0.001;  // mm/s 转 m/s
        double v_lf = data.lf_speed * scale;
        double v_rf = data.rf_speed * scale;
        double v_lb = data.lb_speed * scale;
        double v_rb = data.rb_speed * scale;

        double vx  = ( v_lf + v_rf + v_lb + v_rb) / 4.0;
        double vy  = (-v_lf + v_rf + v_lb - v_rb) / 4.0;
        double vth = (-v_lf + v_rf - v_lb + v_rb) / (4.0 * factor_);

        double delta_x = (vx * cos(robot_th_) - vy * sin(robot_th_)) * dt;
        double delta_y = (vx * sin(robot_th_) + vy * cos(robot_th_)) * dt;
        double delta_th = vth * dt;

        robot_x_ += delta_x;
        robot_y_ += delta_y;
        robot_th_ += delta_th;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_th_);

        // ===================================
        // C. (可选) 直发 TF (EKF开启时通常由EKF代理)
        // ===================================
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = current_time;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = robot_x_;
            t.transform.translation.y = robot_y_;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);
        }

        // ===================================
        // D. 发布纯轮推里程计 Odometry
        // ===================================
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = robot_x_;
        odom_msg.pose.pose.position.y = robot_y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = vth;
        
        odom_msg.twist.covariance[0]  = 1e-3;   
        odom_msg.twist.covariance[7]  = 1e-3;   
        odom_msg.twist.covariance[35] = 1e-1;   
        
        odom_pub_->publish(odom_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}