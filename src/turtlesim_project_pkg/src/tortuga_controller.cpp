#include "rclcpp/rclcpp.hpp"

 
class TortugaControllerNode : public rclcpp::Node{ 
    public:
        
            

        
    private:     
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TortugaControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}