# GVNT
Localización, guiado y control de un robot con ROS en C++. Para ejecutar una parte u otra, simplemente cambiar el CMakeLists.txt y añadir el .cpp que se desee.

La LOCALIZACIÓN (practica2324.cpp) del robot se hará con un filtro de partículas. Para ello, se carga un mapa virtual que debe coincidir con el real y se simulan las observaciones de las partículas.

Filtro de partículas:
 - Primero posiciones aleatorias de (x, y,ángulo) para crear las partículas.
 - Predicción de la posición del robot y de las partículas en base a la posición y la velocidad en el instante anterior (se hace uso de una fórmula de predicción de movimiento).
 - Cálculo de las observaciones: mapa virtual y medidas de los láseres del robot.
 - Matching de las observaciones del robot con las simuladas de las partículas.
 - Resample de las partículas para reubicar las partículas con menor peso. La posición del robot será la media de las ubicaciones de las partículas, que deberían estar agrupadas alrededor de la posición real del robot.

El GUIADO (practica2.cpp) o navegación para la evasión de obstáculos se hace gracias al vector VFF. Se tiene en cuanta el target y el obstáculo que se interpone entre el robot y el target, de manera que hay un vector obstáculo y un vector target, el vector resultante será el vector VFF, de donde extraemos la velocidad angular necesaria para evadir el obstáculo. Una vez el obstáculo se va dejando atrás, es menor su vector, por lo que el vector target vuelve a ser mayor y se llega al target.

El CONTROL (practica3.cpp y CFuzzySpeedController.cpp) se hace con lógica difusa en un controlador que define los rangos de los triángulos o trapezoides y las velocidades lineal y angular. Recibe como parámetro la distancia al target y el error de orientación del robot respecto al target, en base a dichos parámetros y unas reglas que definen el comportamiento deseado, se establecen una velocidad angular y una velocidad lineal.


COMANDOS DE COMPILACIÓN Y EJECUCUCIÓN:
  - catkin_make
  - roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
  - rosrun practica1 practica2324_node

Comentarios:
Hay que usar atan2 para que te de el cuadrante bien, porque la tangente no es la misma, depende del cuadrante
Del fichero XML sacamos los puntos a los que queremos movernos
Primero lo orientamos y luego lo movemos la distancia que queremos
Es un cuaterno que pasamos a euler y nos quedamos con el z
El robot de 0 a pi y de -pi a 0 otra vez

Para la parte de control es necesario instalar fuzzylite en usr/local/lib/:
  - git clone https://github.com/fuzzylite/fuzzylite.git
  - cd fuzzylite/
  - mkdir build && cd build
  - cmake ..
  - make -j4
  - sudo make install

En caso de error al exportar fuzzylite-7.0.0:
 - export LD_LIBRARY_PATH=$LD_LIBRARY_PATH=/usr/local/lib/
