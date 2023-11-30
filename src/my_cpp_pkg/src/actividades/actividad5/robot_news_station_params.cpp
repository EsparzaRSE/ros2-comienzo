#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
 
class RobotNewsStationParamsNode : public rclcpp::Node{
    public:
        RobotNewsStationParamsNode() : Node("robot_news_station_params"){ 
            this->declare_parameter("robot_name", "R2D2");
            robot_name_ = this->get_parameter("robot_name").as_string();
            publisher_ = create_publisher<example_interfaces::msg::String>("robot_news", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotNewsStationParamsNode::publishNews, this));
            RCLCPP_INFO(get_logger(), "Robot News Station has been started.");
        }
    private:
        std::string robot_name_{};
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_{};
        rclcpp::TimerBase::SharedPtr timer_{};

        void publishNews(){
            auto msg = example_interfaces::msg::String(); 
            msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the Robots News Station");
            publisher_->publish(msg);   
        }
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationParamsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}