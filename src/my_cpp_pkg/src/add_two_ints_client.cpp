#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
class AddTwoIntsClientNode : public rclcpp::Node{ 
    public:
        AddTwoIntsClientNode() : Node("add_two_ints_client"){     
            thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));      
        }

        void callAddTwoIntsService(int a, int b){
            auto client{create_client<example_interfaces::srv::AddTwoInts>("add_two_ints")};

            while(!client->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
            }

            auto request{std::make_shared<example_interfaces::srv::AddTwoInts::Request>()};
            request->a = a;
            request->b = b;

            auto future{client->async_send_request(request)};

            try{
                auto response{future.get()};
                RCLCPP_INFO(get_logger(), "%d + %d = %ld", a, b, response->sum);
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
    auto node = std::make_shared<AddTwoIntsClientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}