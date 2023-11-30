Las interfaces que usa son custom y están en el paquete my_robot_interfaces

Para probarlo hay que iniciar el nodo de led_panel que es el servidor, y el nodo battery que es el cliente.
Ya que no hay nodo para comprobar el estado del panel led, se puede probar directamente desde la consola usando
ros2 topic echo led_panel_state

El topic informará si la batería está baja o tiene niveles correctos a intervalos de 2 segundos.

--Mejorado--

Se ha añadido al msg un int64[] para que aparezca el estado de los leds al hacer el topic echo

