#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
class AddTwoIntsServerNode : public rclcpp::Node{
    public:
        AddTwoIntsServerNode() : Node("add_two_ints_server"){
            server_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
                      std::bind(&AddTwoIntsServerNode::callback_add_two_ints, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(get_logger(), "add two ints servers se ha iniciado correctamente");
        }
    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_{};

        void callback_add_two_ints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                   const example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
            response->sum = request->a + request->b;
        }       
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}