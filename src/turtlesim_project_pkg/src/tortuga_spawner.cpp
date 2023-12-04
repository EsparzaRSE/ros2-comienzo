#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/tortuga.hpp"
#include "my_robot_interfaces/msg/tortuga_array.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>
 
class TortugaSpawnerNode : public rclcpp::Node{ 
    public:
        TortugaSpawnerNode() : Node("tortuga_spawner"), generador(std::random_device{}()){
            declare_parameter("spawn_frequency", 1.0);
            declare_parameter<std::string>("turtle_name_prefix");
            frecuencia_spawn_ = get_parameter("spawn_frequency").as_double();
            prefijo_tortuga_ = get_parameter("turtle_name_prefix").as_string();
            hilo1_ = std::thread(&TortugaSpawnerNode::startSpawn, this);
            RCLCPP_INFO(get_logger(), "Tortuga spawner se ha iniciado correctamente.");
            publisher_tortugas_vivas_ = create_publisher<my_robot_interfaces::msg::TortugaArray>("tortugas_vivas", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TortugaSpawnerNode::publishTortugasVivas, this));

        }

        void startSpawn(){ 
            
            while(true){
                callSpawn();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0/frecuencia_spawn_)));
            }                           
        }

        void callSpawn(){

            auto client_spawn{create_client<turtlesim::srv::Spawn>("spawn")};

            while(!client_spawn->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_WARN(get_logger(), "Esperando a que el servidor esté disponible...");
            }

            auto request_spawn{std::make_shared<turtlesim::srv::Spawn::Request>()};

            array_tortugas_.lista_tortugas.push_back(tortugaRandomGenerator());
            request_spawn->x = array_tortugas_.lista_tortugas.back().x;
            request_spawn->y = array_tortugas_.lista_tortugas.back().y;
            request_spawn->name = array_tortugas_.lista_tortugas.back().nombre;

            auto future{client_spawn->async_send_request(request_spawn)};
            try{
                auto response{future.get()};
                RCLCPP_INFO(get_logger(), "Llamada al servicio correcta");
            }
            catch(const std::exception &e){
                RCLCPP_ERROR(get_logger(), "La llamada al servicio ha fallado.");
            }  
        }

        void publishTortugasVivas(){
            auto msg = my_robot_interfaces::msg::TortugaArray();
            msg.lista_tortugas = array_tortugas_.lista_tortugas;
            publisher_tortugas_vivas_->publish(msg); 
        }
        
    private:
        rclcpp::Publisher<my_robot_interfaces::msg::TortugaArray>::SharedPtr publisher_tortugas_vivas_{};
        std::thread hilo1_{};
        rclcpp::TimerBase::SharedPtr timer_{};
        my_robot_interfaces::msg::TortugaArray array_tortugas_{};
        double frecuencia_spawn_{};
        int contador_tortugas_{0};
        std::string prefijo_tortuga_{};
        std::default_random_engine generador;

        my_robot_interfaces::msg::Tortuga tortugaRandomGenerator(){

            my_robot_interfaces::msg::Tortuga tortuga{};              
            std::uniform_real_distribution<float> distribution(0.0, 11.0);

            tortuga.x = distribution(generador);
            tortuga.y = distribution(generador);
            tortuga.nombre = prefijo_tortuga_ + std::to_string(contador_tortugas_++);

            return tortuga;              
        }
};
 
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TortugaSpawnerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}