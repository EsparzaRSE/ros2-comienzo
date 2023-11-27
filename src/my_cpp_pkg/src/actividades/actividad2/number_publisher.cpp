#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberPublisherNode : public rclcpp::Node{
    public:
        NumberPublisherNode() : Node("number_publisher"){
            publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisherNode::publishNumber, this));
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
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}