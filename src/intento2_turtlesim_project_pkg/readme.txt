El proyecto consiste en, utilizando turtlesim y la tortuga inicial como cazadora, ir generando nuevas tortugas 
y desplazando la inicial a la posición de la recién creada. Una vez que la tortuga inicial alcanza la tortuga destino 
se elimina la tortuga cazada y se repite este proceso de manera indefinida.

En el pdf hay pistas sobre los pasos que había que seguir.

De momento lo que más problemas me ha dado es el tema de hilos y timers, ya que se me bloqueaba en la primera
llamada al servicio /kill y ya ni spawneaba ni avanzaba la tortuga.

El tema de servicios, cliente, topics y demás ya me voy acostumbrando, así como a los launcher e interfaces y toda la 
configuración adicional para compilar y ejecutar. 

Para probarlo se necesita turtlesim, my_robot_bringup si se quiere utilizar el launcher y my_robot_interfaces para los msg y servicios

                               ros2 launch my_robot_bringup intento2_turtlesim_project.launch.py 




                                            -- Con esto termina la primera parte del curso ---