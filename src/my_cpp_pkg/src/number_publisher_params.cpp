#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumberPublisherParamsNode : public rclcpp::Node{
    public:
        NumberPublisherParamsNode() : Node("number_publisher_params"){

            this->declare_parameter("number_to_publish", 2);
            this->declare_parameter("publish_frequency", 1.0);

            number_ = this->get_parameter("number_to_publish").as_int();
            publish_frecuency_ = this->get_parameter("publish_frequency").as_double();

            publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0/publish_frecuency_)), 
                                       std::bind(&NumberPublisherParamsNode::publishNumber, this));
            RCLCPP_INFO(get_logger(), "Number publisher se ha iniciado correctamente.");
        }
    private:

        int number_{};
        double publish_frecuency_{};
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_{};
        rclcpp::TimerBase::SharedPtr timer_{};

        void publishNumber(){
            auto msg = example_interfaces::msg::Int64(); 
            msg.data = number_;
            publisher_->publish(msg); 
        }
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherParamsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}