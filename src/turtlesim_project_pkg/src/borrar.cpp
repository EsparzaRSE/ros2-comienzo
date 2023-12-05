#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/tortuga_array.hpp"
#include "my_robot_interfaces/srv/catch_tortuga.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
 
class TortugaControllerNode : public rclcpp::Node{ 
    public:
        TortugaControllerNode() : Node("tortuga_controller"){
            subscriber_pose_ = create_subscription<turtlesim::msg::Pose>(
                "turtle1/pose", 10, std::bind(&TortugaControllerNode::callbackTurtlePose, this, std::placeholders::_1));
            publisher_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);  
            subscriber_tortugas_vivas_ = create_subscription<my_robot_interfaces::msg::TortugaArray>(
                "tortugas_vivas", 10, std::bind(&TortugaControllerNode::callbackTortugasVivas, this, std::placeholders::_1));  
        }
        
        
    private:  

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_pose_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
        rclcpp::Subscription<my_robot_interfaces::msg::TortugaArray>::SharedPtr subscriber_tortugas_vivas_;
        my_robot_interfaces::msg::TortugaArray array_tortugas_{};

        double target_x_{}; 
        double target_y_{};

        void callbackTurtlePose(const turtlesim::msg::Pose::SharedPtr msg){

            double error_distance = std::sqrt(std::pow(target_x_ - msg->x, 2) + std::pow(target_y_ - msg->y, 2));
            double error_angle = std::atan2(target_y_ - msg->y, target_x_ - msg->x) - msg->theta;

            if (error_distance < 0.1) {  // umbral de distancia

                auto client_catch_tortuga{create_client<my_robot_interfaces::srv::CatchTortuga>("catch_tortuga")};

                while(!client_catch_tortuga->wait_for_service(std::chrono::seconds(1))){
                    RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
                }

                auto request_catch_tortuga{std::make_shared<my_robot_interfaces::srv::CatchTortuga::Request>()};
                request_catch_tortuga->nombre = array_tortugas_.lista_tortugas.begin()->nombre;

                auto future{client_catch_tortuga->async_send_request(request_catch_tortuga)};

                try{
                auto response{future.get()};
                RCLCPP_INFO(get_logger(), "Llamada al servicio correcta");
                }
                catch(const std::exception &e){
                    RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
                }            
            }

            geometry_msgs::msg::Twist command;
            command.linear.x = 0.8 * error_distance;
            command.angular.z = 4.0 * error_angle;

            publisher_cmd_vel_->publish(command);
        }

        void callbackTortugasVivas(const my_robot_interfaces::msg::TortugaArray::SharedPtr msg){
            array_tortugas_.lista_tortugas = msg->lista_tortugas;
            target_x_ = msg->lista_tortugas.begin()->x; 
            target_y_ = msg->lista_tortugas.begin()->y;    
        } 
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TortugaControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}