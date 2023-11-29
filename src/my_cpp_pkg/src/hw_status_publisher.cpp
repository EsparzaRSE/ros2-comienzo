#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
 
class HardwareStatusPublisherNode : public rclcpp::Node{
    public:
        HardwareStatusPublisherNode() : Node("hardware_status_publisher"){
            publisher_ = create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
            timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this)); 
            RCLCPP_INFO(get_logger(), "Hardware status publisher se ha iniciado correctamente.");
        }
    private:
        rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_{};
        rclcpp::TimerBase::SharedPtr timer_{};

        void publishHardwareStatus(){
            auto msg{my_robot_interfaces::msg::HardwareStatus()};
            msg.temperature = 57;
            msg.are_motors_ready = false;
            msg.debug_message = "¡Los motores están demasiado calientes!";
            publisher_->publish(msg);

        }
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}