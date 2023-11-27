#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node{
    public:
        NumberCounterNode() : Node("number_counter"){
            subscriber_ = create_subscription<example_interfaces::msg::Int64>
                          ("number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
            publisher_ = create_publisher<example_interfaces::msg::Int64>("number_count", 10);
            RCLCPP_INFO(get_logger(), "Number counter se ha iniciado correctamente.");
            
        }
    private:
        int contador{};
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_{};
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_{};

        void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg){        
            contador += msg->data;
            auto mensaje = example_interfaces::msg::Int64(); 
            mensaje.data = contador;
            publisher_->publish(mensaje); 
        }    
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}