#pragma once

#ifndef TORTUGA_SPAWNER2
#define TORTUGA_SPAWNER2

#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/msg/tortuga.hpp"
#include "my_robot_interfaces/msg/tortuga_array.hpp"
#include "my_robot_interfaces/srv/catch_tortuga.hpp"
#include <random>

// Incluye otros headers necesarios

class TortugaSpawner2Node : public rclcpp::Node{ 
    public:
        TortugaSpawner2Node();
        void crearTortuga();
        void callSpawnService(my_robot_interfaces::msg::Tortuga& tortuga);
        my_robot_interfaces::msg::Tortuga tortugaRandomGenerator();
        void publishTortugasVivas();
        void callbackCatchTortuga(const my_robot_interfaces::srv::CatchTortuga::Request::SharedPtr request,
                                  const my_robot_interfaces::srv::CatchTortuga::Response::SharedPtr response);
        void callKillTortugaService(std::string nombre_tortuga);

    private:
        rclcpp::TimerBase::SharedPtr timer_spawn_tortugas_{};
        rclcpp::TimerBase::SharedPtr timer_tortugas_vivas_{};  
        rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr service_spawn_tortugas_{};
        rclcpp::Publisher<my_robot_interfaces::msg::TortugaArray>::SharedPtr publisher_tortugas_vivas_{};
        std::vector<std::shared_ptr<std::thread>> hilos_spawn_tortugas_{};
        std::string prefijo_tortuga_{};
        int contador_tortugas_{};
        std::default_random_engine generador_{};
        my_robot_interfaces::msg::TortugaArray array_tortugas_{};
        double frecuencia_spawn_{};
        rclcpp::Service<my_robot_interfaces::srv::CatchTortuga>::SharedPtr service_catch_tortuga_{};
        std::vector<std::shared_ptr<std::thread>> hilos_kill_tortugas_{};           
}; 

#endif