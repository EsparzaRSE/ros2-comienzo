#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
 
class BatteryNode : public rclcpp::Node{ 
    public:
        BatteryNode() : Node("battery"), battery_state_{true}{ 
            thread1_ = std::thread(&BatteryNode::toggleBatteryState, this);
            RCLCPP_INFO(get_logger(), "Battery se ha iniciado correctamente.");      
        }

        void toggleBatteryState(){
            while(true){
                if(battery_state_) std::this_thread::sleep_for(std::chrono::seconds(4));
                else std::this_thread::sleep_for(std::chrono::seconds(6));
                
                battery_state_ = !battery_state_;
                callSetLed(3, battery_state_);
            }
        }

        void callSetLed(int a, bool b){
            auto client{create_client<my_robot_interfaces::srv::SetLed>("set_led")};

            while(!client->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
            }

            auto request{std::make_shared<my_robot_interfaces::srv::SetLed::Request>()};
            request->led_number = a;
            request->state = b;

            auto future{client->async_send_request(request)};
            
            try{
                auto response{future.get()};
                RCLCPP_INFO(get_logger(), "LLamada al servicio correcta");
            }
            catch(const std::exception &e){
                RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
            }          
        }
    private:   
        std::thread thread1_{}; 
        bool battery_state_{};
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}