#include "../include/tortuga_controller2.hpp"
#include "my_robot_interfaces/srv/catch_tortuga.hpp"
#include <cmath>

TortugaController2Node::TortugaController2Node() : Node("tortuga_controller2"){ 
    subscriber_pose_ = create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, 
                       std::bind(&TortugaController2Node::callbackTurtlePose, this, std::placeholders::_1));
    subscriber_tortugas_vivas_ = create_subscription<my_robot_interfaces::msg::TortugaArray>(
                "tortugas_vivas", 10, std::bind(&TortugaController2Node::callbackTortugasVivas, this, std::placeholders::_1)); 
    timer_control_loop_ = create_wall_timer(std::chrono::milliseconds(10), 
                                            std::bind(&TortugaController2Node::controlLoop, this));
    publisher_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);  
}

void TortugaController2Node::callbackTurtlePose(const turtlesim::msg::Pose::SharedPtr pose){
    pose_ = *pose.get();
    
}

void TortugaController2Node::callbackTortugasVivas(const my_robot_interfaces::msg::TortugaArray::SharedPtr tortugas_vivas){
    if (!tortugas_vivas->lista_tortugas.empty()) {
        tortuga_objetivo_ = tortugas_vivas->lista_tortugas.at(0);
    }   
}

//esta la he tenido que mirar porque la liaba al calcular el movimiento y ángulo
void TortugaController2Node::controlLoop(){ 

    if(tortuga_objetivo_.nombre == "") return; 
    double dist_x{tortuga_objetivo_.x - pose_.x};
    double dist_y{tortuga_objetivo_.y - pose_.y};
    double distancia{std::sqrt(dist_x * dist_x + dist_y * dist_y)};

    auto msg{geometry_msgs::msg::Twist()};

    if (distancia > 0.5){
        // Posicion
        msg.linear.x = 2 * distancia;

        // Orientacion
        double angulo_direccion{std::atan2(dist_y, dist_x)};
        double angulo_diferencia{angulo_direccion - pose_.theta};

        if(angulo_diferencia > M_PI) angulo_diferencia -= 2 * M_PI;
        else if(angulo_diferencia  < -M_PI) angulo_diferencia  += 2 * M_PI;
        
        msg.angular.z = 6 * angulo_diferencia;
    }
    else{
        // Objetivo alcanzado
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;

        hilos_tortuga_atrapada_.push_back(std::make_shared<std::thread>(std::bind(&TortugaController2Node::callCatchTortugaService, 
                                                                        this, tortuga_objetivo_.nombre)));
        tortuga_objetivo_.nombre = "";
    }
        publisher_cmd_vel_->publish(msg);
}

void TortugaController2Node::callCatchTortugaService(const std::string& nombre_tortuga){

    auto client_catch{create_client<my_robot_interfaces::srv::CatchTortuga>("catch_tortuga")};

    while(!client_catch->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
    }

    auto request_catch{std::make_shared<my_robot_interfaces::srv::CatchTortuga::Request>()};
    request_catch->nombre = nombre_tortuga;

    auto future{client_catch->async_send_request(request_catch)};

    try{
        auto response{future.get()};
    }
    catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
    }   
}
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TortugaController2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}