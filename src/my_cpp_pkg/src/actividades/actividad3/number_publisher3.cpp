#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberPublisher3Node : public rclcpp::Node{
    public:
        NumberPublisher3Node() : Node("number_publisher3"){
            publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NumberPublisher3Node::publishNumber, this));
            RCLCPP_INFO(get_logger(), "Number publisher se ha iniciado correctamente.");
        }
    private:
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_{};
        rclcpp::TimerBase::SharedPtr timer_{};

        void publishNumber(){
            auto msg = example_interfaces::msg::Int64(); 
            msg.data = 10;
            publisher_->publish(msg); 
        }
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}