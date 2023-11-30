#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include <array>
 
class LedPanelNode : public rclcpp::Node{ 
    public:
        LedPanelNode() : Node("led_panel"){ 
            server_ = create_service<my_robot_interfaces::srv::SetLed>("set_led", 
                      std::bind(&LedPanelNode::callback_led_panel, this, std::placeholders::_1, std::placeholders::_2));
            publisher_ = create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);
            timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&LedPanelNode::publishLedPanelStatus, this));
            RCLCPP_INFO(get_logger(), "LED panel se ha iniciado correctamente.");
        }
    private:
        std::array<int, 3> led_panel_{0, 0, 0};
        rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_{};
        rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr publisher_{};
        rclcpp::TimerBase::SharedPtr timer_{};

        void callback_led_panel(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                                const my_robot_interfaces::srv::SetLed::Response::SharedPtr response) {

            if(request->led_number != 3) response->success = false;
            else{         
                led_panel_[2] = request->state ? 0 : 1;
                response->success = true; 
            }
        }

        void publishLedPanelStatus(){
            auto msg{my_robot_interfaces::msg::LedStates()};
            if(led_panel_[2] == 1) msg.state_message = "Batería baja";
            else msg.state_message = "Niveles de batería correctos";
            publisher_->publish(msg);
        } 

};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}