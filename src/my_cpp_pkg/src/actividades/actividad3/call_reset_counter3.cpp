#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"
 
class CallResetCounter3Node : public rclcpp::Node{
    public:
        CallResetCounter3Node() : Node("call_reset_counter3"){     
            thread1_ = std::thread(std::bind(&CallResetCounter3Node::callResetCounter3Service, this, true));      
        }

        void callResetCounter3Service(bool reset){
            auto client{create_client<example_interfaces::srv::SetBool>("reset_counter")};

            while(!client->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
            }

            auto request{std::make_shared<example_interfaces::srv::SetBool::Request>()};
            request->data = reset;

            auto future{client->async_send_request(request)};

            try{
                auto response{future.get()};
                RCLCPP_INFO(get_logger(), "La Llamada al servicio se ha completada con éxito");
            }
            catch(const std::exception &e){
                RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
            }          
        }
    private:   
        std::thread thread1_{}; 
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CallResetCounter3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}