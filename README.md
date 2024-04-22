# GVNT
Localización y guiado de un robot con ROS en C++.

Se trata de un filtro de partículas para la localización del robot. Para ello, se carga un mapa virtual y se simulan las observaciones de las partículas.

Primero posiciones aleatorias de x, y y angulo para crear las partículas.
Prediccion de la posicion del robot y de las particulas en base a la posición y la velocidad en el instante anterior.
Observaciones: mapa virtual y medidas de los láseres del robot.
Luego se hace el matching de las observaciones del robot con las simuladas de las partículas.
Resample de las particulas para reubicar las partículas con menor peso.


COMANDOS DE EJECUCUCIÓN:
//catkin_make
//roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
//rosrun practica1 practica2324_node

Comentarios:
hay que usar atan2 para que te de el cuadrante bien, porque la tangente no es la misma, depende del cuadrante
del fichero txt sacamos los puntos a los que queremos movernos
primero lo orientamos y luego lo movemos la distancia que queremos
es un cuaterno que pasamos a euler y nos quedamos con el z
el robot de 0 a pi y de -pi a 0 otra vez
