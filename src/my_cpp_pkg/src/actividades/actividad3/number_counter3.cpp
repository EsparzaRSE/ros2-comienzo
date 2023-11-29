#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounter3Node : public rclcpp::Node{
    public:
        NumberCounter3Node() : Node("number_counter3"){
            subscriber_ = create_subscription<example_interfaces::msg::Int64>
                          ("number", 10, std::bind(&NumberCounter3Node::callbackNumber, this, std::placeholders::_1));
            publisher_ = create_publisher<example_interfaces::msg::Int64>("number_count", 10);
            server_ = create_service<example_interfaces::srv::SetBool>("reset_counter", 
                      std::bind(&NumberCounter3Node::callbackResetCounter3, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(get_logger(), "Number counter se ha iniciado correctamente.");
            
        }
    private:
        int contador{};
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_{};
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_{};
        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_{};

        void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg){        
            contador += msg->data;
            auto mensaje = example_interfaces::msg::Int64(); 
            mensaje.data = contador;
            publisher_->publish(mensaje);
            RCLCPP_INFO(get_logger(), "%d", contador); 
        }

        void callbackResetCounter3(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                                  const example_interfaces::srv::SetBool::Response::SharedPtr response){
            if(request->data){
                contador = 0;
                response->success = true;
                response->message = "El contador se ha reestablecido correctamente";
            }else{
                response->success = false;
                response->message = "La petición ha fallado, no se ha reseteado el contador";
            }
        }    
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 