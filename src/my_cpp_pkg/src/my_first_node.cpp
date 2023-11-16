#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv){

    rclcpp::init(argc, argv); //inicializa las comunicaciones en ROS2
    auto node = std::make_shared<rclcpp::Node>("cpp_test"); //se crea el nodo DENTRO del archivo, no es el archivo el nodo
    RCLCPP_INFO(node->get_logger(), "Hello cpp Node"); // imprimes
    rclcpp::spin(node); //mantener el nodo vivo
    rclcpp::shutdown(); //terminan las comunicaciones 

    return 0;
}