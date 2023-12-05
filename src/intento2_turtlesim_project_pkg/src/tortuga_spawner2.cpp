#include "../include/tortuga_spawner2.hpp"
#include "turtlesim/srv/kill.hpp"

TortugaSpawner2Node::TortugaSpawner2Node() : Node("tortuga_spawner2"), contador_tortugas_{0}, generador_(std::random_device{}()){

    declare_parameter("spawn_frequency", 1.0);
    declare_parameter<std::string>("turtle_name_prefix", "NinjaTurtle");
    frecuencia_spawn_ = get_parameter("spawn_frequency").as_double();
    prefijo_tortuga_ = get_parameter("turtle_name_prefix").as_string();
    timer_spawn_tortugas_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0/frecuencia_spawn_)), 
                                              std::bind(&TortugaSpawner2Node::crearTortuga, this));
    publisher_tortugas_vivas_ = create_publisher<my_robot_interfaces::msg::TortugaArray>("tortugas_vivas", 10);
    timer_tortugas_vivas_ = create_wall_timer(std::chrono::seconds(1), std::bind(&TortugaSpawner2Node::publishTortugasVivas, this));
    service_catch_tortuga_ = create_service<my_robot_interfaces::srv::CatchTortuga>("catch_tortuga", 
                      std::bind(&TortugaSpawner2Node::callbackCatchTortuga, this, std::placeholders::_1, std::placeholders::_2));    
}

void TortugaSpawner2Node::crearTortuga(){
    array_tortugas_.lista_tortugas.push_back(tortugaRandomGenerator());
    hilos_spawn_tortugas_.push_back(std::make_shared<std::thread>(
                std::bind(&TortugaSpawner2Node::callSpawnService, this, std::ref(array_tortugas_.lista_tortugas.back()))));
}

void TortugaSpawner2Node::publishTortugasVivas(){
    auto msg{my_robot_interfaces::msg::TortugaArray()};
    msg.lista_tortugas = array_tortugas_.lista_tortugas;
    publisher_tortugas_vivas_->publish(msg); 
}

void TortugaSpawner2Node::callSpawnService(my_robot_interfaces::msg::Tortuga& tortuga){

    auto client_spawn_service{create_client<turtlesim::srv::Spawn>("spawn")};

    while(!client_spawn_service->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
    }

    auto request_spawn{std::make_shared<turtlesim::srv::Spawn::Request>()};

    request_spawn->x = tortuga.x;
    request_spawn->y = tortuga.y;
    request_spawn->theta = tortuga.theta;
    request_spawn->name = tortuga.nombre;

    auto future{client_spawn_service->async_send_request(request_spawn)};

    try{
        auto response{future.get()};
        RCLCPP_INFO(get_logger(), "Tortuga spawneada con éxito");
    }
    catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
    } 
}

my_robot_interfaces::msg::Tortuga TortugaSpawner2Node::tortugaRandomGenerator(){
    
    my_robot_interfaces::msg::Tortuga tortuga{};              
    std::uniform_real_distribution<float> distribution(0.1, 0.9);

    tortuga.x = distribution(generador_) * 10.0;
    tortuga.y = distribution(generador_) * 10.0;
    tortuga.theta = distribution(generador_) * 2 * M_PI;
    tortuga.nombre = prefijo_tortuga_ + std::to_string(contador_tortugas_++);

    return tortuga;  
}

void TortugaSpawner2Node::callbackCatchTortuga(const my_robot_interfaces::srv::CatchTortuga::Request::SharedPtr request,
                                               const my_robot_interfaces::srv::CatchTortuga::Response::SharedPtr response){
    hilos_kill_tortugas_.push_back(
            std::make_shared<std::thread>(
                std::bind(&TortugaSpawner2Node::callKillTortugaService, this, request->nombre)));
        response->success = true;
}

void TortugaSpawner2Node::callKillTortugaService(std::string nombre_tortuga){

        auto client_kill{create_client<turtlesim::srv::Kill>("kill")};

        while(!client_kill->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
        }

        auto request_kill{std::make_shared<turtlesim::srv::Kill::Request>()};
        request_kill->name = nombre_tortuga;

        auto future{client_kill->async_send_request(request_kill)};

        try{
            future.get();

            for (int i{0}; i < static_cast<int>(array_tortugas_.lista_tortugas.size()); ++i){
                if (array_tortugas_.lista_tortugas.at(i).nombre == nombre_tortuga){
                    array_tortugas_.lista_tortugas.erase(array_tortugas_.lista_tortugas.begin() + i);
                    RCLCPP_INFO(get_logger(), "Tortuga eliminada con éxito");
                    publishTortugasVivas();
                    break;
                }
            }
        }
        catch(const std::exception &e){
            RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
        }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TortugaSpawner2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}