#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <vector>
#include <string>
#include <set>
#include <iomanip>
#include <sstream>
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class ProtocolRFIDNode : public rclcpp::Node {
public:
    ProtocolRFIDNode() : Node("ma60_rfid_node"), io_ctx_(), serial_(io_ctx_) {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<double>("read_interval", 0.2); // 5Hz 高频读取配合自转

        auto port = this->get_parameter("serial_port").as_string();
        auto baud = this->get_parameter("baudrate").as_int();
        auto interval = this->get_parameter("read_interval").as_double();

        // 发布底层实时的原始卡号流（可能包含大量重复）
        pub_raw_ = this->create_publisher<std_msgs::msg::String>("/rfid/tag_data_raw", 10);

        // 初始化 RFID 启停控制服务
        srv_switch_ = this->create_service<std_srvs::srv::SetBool>(
            "/rfid/inventory_switch",
            std::bind(&ProtocolRFIDNode::handleInventorySwitch, this, std::placeholders::_1, std::placeholders::_2));

        // 串口初始化
        try {
            serial_.open(port);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            RCLCPP_INFO(this->get_logger(), "✅ RFID 串口连接成功，等待调度指令...");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 串口打开失败: %s", e.what());
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(interval),
            std::bind(&ProtocolRFIDNode::doInventory, this));
    }

private:
    boost::asio::io_context io_ctx_;
    boost::asio::serial_port serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_raw_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_switch_;
    
    bool is_inventory_active_ = false;
    std::vector<uint8_t> rx_buffer_;

    // 处理远程开关请求
    void handleInventorySwitch(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (request->data && !is_inventory_active_) {
            rx_buffer_.clear(); // 刚开启时清空过去的脏缓存
        }
        is_inventory_active_ = request->data;
        response->success = true;
        response->message = is_inventory_active_ ? "RFID 已开启盘点" : "RFID 已结束盘点";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void doInventory() {
        if (!is_inventory_active_ || !serial_.is_open()) return;

        const uint8_t poll_cmd[] = {0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E};
        boost::system::error_code ec;
        boost::asio::write(serial_, boost::asio::buffer(poll_cmd), ec);

        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        auto tags = readAndParseFrames(); // 调用解析协议的方法
        if (!tags.empty()) {
            std::string tag_list;
            for (const auto &tag : tags) {
                if (!tag_list.empty()) tag_list += ";";
                tag_list += tag;
            }
            auto msg = std_msgs::msg::String();
            msg.data = tag_list;
            pub_raw_->publish(msg); // 抛出原始数据
        }
    }

    // 协议层防断包粘包解析
    std::set<std::string> readAndParseFrames() {
        std::set<std::string> found_tags;
        uint8_t temp_buf[512];
        boost::system::error_code ec;
        
        size_t bytes_read = serial_.read_some(boost::asio::buffer(temp_buf), ec);
        if (!ec && bytes_read > 0) {
            rx_buffer_.insert(rx_buffer_.end(), temp_buf, temp_buf + bytes_read);
        }

        while (rx_buffer_.size() >= 7) { 
            if (rx_buffer_[0] != 0xBB) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            uint16_t pl_len = (rx_buffer_[3] << 8) | rx_buffer_[4];
            uint16_t expected_frame_len = pl_len + 7; 

            if (rx_buffer_.size() < expected_frame_len) break; 

            if (rx_buffer_[expected_frame_len - 1] != 0x7E) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            uint8_t checksum = 0;
            for (int i = 1; i < expected_frame_len - 2; ++i) checksum += rx_buffer_[i];
            
            if (checksum != rx_buffer_[expected_frame_len - 2]) {
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + expected_frame_len);
                continue;
            }

            uint8_t type = rx_buffer_[1], cmd = rx_buffer_[2];
            if (type == 0x02 && cmd == 0x22) {
                int epc_len = pl_len - 5; 
                if (epc_len > 0) {
                    std::stringstream ss;
                    for (int j = 0; j < epc_len; ++j) {
                        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)rx_buffer_[8 + j];
                    }
                    found_tags.insert(ss.str());
                }
            }

            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + expected_frame_len);
        }
        if (rx_buffer_.size() > 2048) rx_buffer_.clear(); // 防堆积保护
        return found_tags;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProtocolRFIDNode>());
    rclcpp::shutdown();
    return 0;
}