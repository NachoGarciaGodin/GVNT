#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include <string>
#include <cmath>
#include <tf/tf.h>
#include <tinyxml2.h>
#include <iostream>
#include <fstream>

//catkin_make
//roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
//rosrun practica1 practica2324_node

//hay que usar atan2 para que te de el cuadrante bien, porque la tangente no es la misma, depende del cuadrante
//del fichero txt sacamos los puntos a loa que queremos movernos
//primero lo orientamos y luego lo movemos la distancia que queremos
//es un cuaterno que pasamos a euler y nos quedamos con el z
//el robot de 0 a pi y de -pi a 0 otra vez

//primero posiciones aleatorias de x, y y angulo para crear las partículas
//prediccion de la posicion del robot y de las particulas
//observaciones: mapa virtual y medidas de los láseres del robot
//lueo se hace el matching
//resample de las particulas

//la imagen es de 574x536
//10,177  320,177 320,74
//10,380(abajo izquierda)   275,380(abajo derecha)  274,290(arriba derecha)

struct Posicion {
    double x;
    double y;
};

struct Particula {
    double x;
    double y;
    double theta;
    Particula() : x(0), y(0), theta(0) {

    }

    Particula(const Particula& other) : x(other.x), y(other.y), theta(other.theta) {

    }
};

struct Obstacle {
    double x;
    double y;
    double width;
    double height;
};

Posicion ubicacionActual,dato; //variable global para la ubicación actual
float angulo2,dist2,yaw;
std::vector<Posicion> puntos; //= {{4,4}, {4,2}, {2,4}}; //defino los puntos a los que quiero que vaya el robot

int map_width, map_height;
std::vector<Obstacle> obstacles;
int num_particulas = 1000;
std::vector<Particula*> particulas;


std::vector<std::vector<float>> distances;
std::vector<float> distanciaMahalanobis(num_particulas);//vector para guarda los valores de semejanza calculados con la distancia de Mahalannobis
std::vector<float> medidasLaser(360, 0.0); //para las medidas del robot que adquiero de los láseres


void readXmlFile(const std::string &filename, int &map_width, int &map_height, std::vector<Obstacle> &obstacles)
{
    puntos.clear();
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr << "Error al cargar el archivo XML: " << filename << std::endl;
        return;
    }

    tinyxml2::XMLElement *mapElement = doc.FirstChildElement("map");
    if (!mapElement)
    {
        std::cerr << "Formato XML no válido: falta el elemento 'map'" << std::endl;
        return;
    }

    mapElement->QueryIntAttribute("width", &map_width);
    mapElement->QueryIntAttribute("height", &map_height);

    tinyxml2::XMLElement *obstaclesElement = mapElement->FirstChildElement("obstacles");
    if (!obstaclesElement)
    {
        std::cerr << "Formato XML no válido: falta el elemento 'obstacles'" << std::endl;
        return;
    }

    for (tinyxml2::XMLElement *obstacleElement = obstaclesElement->FirstChildElement("obstacle");
         obstacleElement;
         obstacleElement = obstacleElement->NextSiblingElement("obstacle")) {
        Obstacle obstacle;
        obstacleElement->QueryDoubleAttribute("x", &obstacle.x);
        obstacleElement->QueryDoubleAttribute("y", &obstacle.y);
        obstacleElement->QueryDoubleAttribute("width", &obstacle.width); // Extraer el ancho del obstáculo
        obstacleElement->QueryDoubleAttribute("height", &obstacle.height); // Extraer el alto del obstáculo
        obstacles.push_back(obstacle);
    }
    std::cout << "Obstacles read from XML file: " << obstacles.size() << std::endl;
    std::cout << "Obstacles: " << std::endl;
    for (const auto &obstacle : obstacles)
    {
        std::cout << "  {x:" << obstacle.x << ", y:" << obstacle.y << ", width:" << obstacle.width << ", height:" << obstacle.height << "}" << std::endl;
    }
    std::cout << std::endl;

    tinyxml2::XMLElement *pointsElement = mapElement->FirstChildElement("points");
    if (!pointsElement)
    {
        std::cerr << "Formato XML no válido: falta el elemento 'points'" << std::endl;
        return;
    }
    for (tinyxml2::XMLElement *pointElement = pointsElement->FirstChildElement("point");
         pointElement;
         pointElement = pointElement->NextSiblingElement("point"))
    {
        Posicion newPoint;
        pointElement->QueryDoubleAttribute("x", &newPoint.x);
        pointElement->QueryDoubleAttribute("y", &newPoint.y);
        puntos.push_back(newPoint);
    }
    std::cout << "Puntos leídos del archivo XML: " << puntos.size() << std::endl;
    std::cout << "Puntos: " << std::endl;
    for (const auto &point : puntos)
    {
        std::cout << "  {" << point.x << ", " << point.y << "}" << std::endl;
    }
    std::cout << std::endl;

}

bool esUbicacionValida(const Particula &posicion, const std::vector<Obstacle> &obstacles) {
    // Iterar sobre todos los obstáculos
    for (const auto &obstacle : obstacles) {
        // Calcular las coordenadas del punto inferior izquierdo y superior derecho del obstáculo
        double obstaculo_x_min = obstacle.x;
        double obstaculo_x_max = obstacle.x + obstacle.width;
        double obstaculo_y_min = obstacle.y;
        double obstaculo_y_max = obstacle.y + obstacle.height;

        // Verificar si la posición está dentro del obstáculo
        if (posicion.x >= obstaculo_x_min && posicion.x <= obstaculo_x_max &&
            posicion.y >= obstaculo_y_min && posicion.y <= obstaculo_y_max) {
            return false; // La posición está dentro de un obstáculo
        }
    }
return true; // La posición es válida
}

void inicializarParticulasAleatorias(int num_particulas, int map_width, int map_height, const std::vector<Obstacle> &obstacles) {
for (int i = 0; i < num_particulas; ++i) {
    Particula* particula = new Particula();
    do {
        particula->x = rand() % map_width;  // Genera una coordenada x aleatoria dentro de los límites del mapa
        particula->y = rand() % map_height; // Genera una coordenada y aleatoria dentro de los límites del mapa
        particula->theta = static_cast<double>(std::rand()) / RAND_MAX * 2.0 * M_PI - M_PI;                  // Rango: -pi a pi
    } while (!esUbicacionValida(*particula, obstacles)); // Repetir hasta que la ubicación sea válida
    particulas.push_back(particula);
}

}

void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    //std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << "}" << std::endl;
    ubicacionActual= {msg->pose.pose.position.x, msg->pose.pose.position.y};
    yaw=tf::getYaw(msg->pose.pose.orientation);
    //std::cout << "Yaw: " << yaw << std::endl;
}

void laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
    medidasLaser = std::vector<float>(scan->ranges.begin(), scan->ranges.end());
    // std::cout << "Medidas Laser: ";
    //     for (const auto& medida : medidasLaser) {
    //         std::cout << medida << " ";
    //     }
    //     std::cout << std::endl;
}

double calcularDistancia(Posicion pos1, Posicion pos2) {
    float distancia = std::sqrt(std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2));
    return distancia;
}

double calcularAngulo(Posicion pos1, Posicion pos2) {
    float angulo = std::atan2(pos2.y - pos1.y, pos2.x - pos1.x);
    return angulo;
}

void prediccionPosicionParticulas() {
    for (auto& particula : particulas) {
        double delta_x = 0.1 * std::cos(particula->theta)*0.1+0.01; //0.01 es  Q(x), es decir, el ruido
        double delta_y = 0.1 * std::sin(particula->theta)*0.1+0.01;
        double delta_theta = 0.1*0.1+0.01; // Rango: -0.1 a 0.1
        particula->x += delta_x;
        particula->y += delta_y;
        particula->theta += delta_theta;
    }
}

// Función para calcular si hay un punto de corte entre una línea y un obstáculo
double distanciaParticulaObstaculo(const Particula* particula, const std::vector<Obstacle>& obstacles) {
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& obstacle : obstacles) {
        // Calcular la pendiente de la línea
        double m = std::tan(particula->theta);

        // Calcular la intersección de la línea con los lados del obstáculo
        double y_intercept_left = particula->y - m * (obstacle.x - particula->x);
        double y_intercept_right = particula->y - m * (obstacle.x + obstacle.width - particula->x);
        double x_intercept_bottom = particula->x + (obstacle.y - particula->y) / m;
        double x_intercept_top = particula->x + (obstacle.y + obstacle.height - particula->y) / m;

        // Comprobar si los puntos de intersección están dentro del rango del obstáculo
        bool intersect_left = (y_intercept_left >= obstacle.y && y_intercept_left <= obstacle.y + obstacle.height);
        bool intersect_right = (y_intercept_right >= obstacle.y && y_intercept_right <= obstacle.y + obstacle.height);
        bool intersect_bottom = (x_intercept_bottom >= obstacle.x && x_intercept_bottom <= obstacle.x + obstacle.width);
        bool intersect_top = (x_intercept_top >= obstacle.x && x_intercept_top <= obstacle.x + obstacle.width);

        // Calcular la distancia al punto de intersección más cercano
        double distance;
        if (intersect_left || intersect_right) {
            distance = std::abs(obstacle.x - particula->x) / std::cos(particula->theta);
        } else if (intersect_bottom || intersect_top) {
            distance = std::abs(obstacle.y - particula->y) / std::sin(particula->theta);
        } else {
            // No hay intersección
            distance = std::numeric_limits<double>::max();
        }

        // Actualizar la distancia mínima si es necesario
        min_distance = std::min(min_distance, distance);
    }

    return min_distance;
}

// Función para calcular la matriz de distancias para cada grado de orientación
void calculaObservaciones() {
    const int num_grados = 360;
    distances.resize(particulas.size(), std::vector<float>(num_grados));

    for (size_t i = 0; i < particulas.size(); ++i) {
        for (int j = 0; j < num_grados; ++j) {
            // Calcular el ángulo en radianes
            double theta = j * M_PI / 180.0;
            Particula* particulaAux = new Particula(*particulas[i]);
            particulaAux->theta = theta;

            // Calcular la distancia al obstáculo más cercano
            double distanceAux = distanciaParticulaObstaculo(particulaAux, obstacles);
            // Almacenar la distancia en la matriz
            distances[i][j] = std::abs(distanceAux);
        }
    }
}

void calcularDistanciaMahalanobis() {
    for (size_t i = 0; i <num_particulas; ++i) {
        double sum = 0;
        for (int j = 0; j < 360; ++j) {
            if (distances[i][j] > 12.06) {
                distances[i][j] = 12.06; //valor máximo de la distancia de los láseres
            }
            if (medidasLaser[j] > 12.06){
                medidasLaser[j] = 12.06;
            }
            sum += std::pow(  distances[i][j] - medidasLaser[j], 2);
        }
        distanciaMahalanobis[i] = std::sqrt(sum/0.01); //se divide entre una varianza
    }
}

void resampleParticulas() {
    int num_mejores = num_particulas * 0.2;
    int num_peores = num_particulas - num_mejores;
    std::vector<Particula*> particulasNuevas;
    double error = 0.1; // variación en la nuevaubicación
    // Vector de pares (distancia, índice) para ordenar las partículas
    std::vector<std::pair<double, int>> distanciasIndices;
    for (int i = 0; i < num_particulas; ++i) {
        distanciasIndices.push_back(std::make_pair(distanciaMahalanobis[i], i));
    }

    std::sort(distanciasIndices.begin(), distanciasIndices.end()); // Ordenar las partículas 
    
    for (int i = 0; i < num_mejores; ++i) { // Copiar las partículas del 20% superior
        int idx = distanciasIndices[i].second;
        particulasNuevas.push_back(new Particula(*particulas[idx]));
    }

    for (int i = 0; i < num_peores; ++i) { // Reubicar el 80%
        // Seleccionar una partícula aleatoria del 20% superior
        int idx_mejor = distanciasIndices[std::rand() % num_mejores].second;
        Particula* particulaMejor = particulas[idx_mejor];
        Particula* nuevaParticula = new Particula(*particulaMejor);
        nuevaParticula->x += error; // Añadir el error
        nuevaParticula->y += error;
        particulasNuevas.push_back(nuevaParticula);
    }
    
    for (auto& particula : particulas) { // Liberar la memoria de las partículas antiguas
        delete particula;
    }

    particulas = particulasNuevas; 
}

void calculoLocalizacion() {
    double x = 0;
    double y = 0;
    double theta = 0;
    for (const auto& particula : particulas) {
        x += particula->x;
        y += particula->y;
        theta += particula->theta;
    }
    x /= num_particulas;
    y /= num_particulas;
    theta /= num_particulas;
    std::cout << "Localización: {" << x << ", " << y << ", " << theta << "}" << std::endl;
    std::cout << "Ubicación: {" << ubicacionActual.x << "," << ubicacionActual.y << "}" << std::endl;
}

int main (int argc, char** argv){

    ros::init(argc, argv, "practica2324");
    ros::NodeHandle nh;
    srand(time(0));
    ros::Subscriber odom_sub = nh.subscribe("/robot0/odom", 1000, odometryCallback);
    ros::Subscriber laser_sub = nh.subscribe("/robot0/laser_0", 1000, laserCallback);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1000);

    std::string xmlFilePath = "/home/alumno/robotica_movil_ws/src/practica1/src/map.xml";
    readXmlFile(xmlFilePath, map_width, map_height, obstacles);
    inicializarParticulasAleatorias(num_particulas, map_width, map_height, obstacles); //creo las particulas
    // for (const auto& particula : particulas) {
    //     std::cout << "Particula: {" << particula->x << ", " << particula->y << ", " << particula->theta << "}" << std::endl;
    // }

    ros::Rate loop(10);//frequency

    int coger_dato = 1,cuenta=0;

    while (ros::ok()) {
        if (coger_dato == 1) { //&& std::getline(archivo, linea)) {
            //double x1 = std::stod(linea.substr(0, linea.find(',')));
            //double y1 = std::stod(linea.substr(linea.find(',') + 1));
            if(cuenta == puntos.size()){
                return 0;
            }
            dato = puntos[cuenta];
            coger_dato = 0;
            cuenta++;
        }
        //voy calculando el  angulo y la distancia para ver que van disminuyendo
        angulo2 = calcularAngulo(ubicacionActual, dato);
        dist2 = calcularDistancia(ubicacionActual, dato);
        // std::cout << "angulo2: " << angulo2 << std::endl;
        // std::cout << "dist2: " << dist2 << std::endl;
        // std::cout << "dato: {" << dato.x << ", " << dato.y << "}" << std::endl;

        geometry_msgs::Twist speed;
        
        prediccionPosicionParticulas();
        // for (const auto& particula : particulas) {
        // std::cout << "Particula: {" << particula->x << ", " << particula->y << ", " << particula->theta << "}" << std::endl;
        // }

        calculaObservaciones();   
        // Mostrar las distancias calculadas para cada particula
        // for (size_t i = 0; i < particulas.size(); ++i) {
        //     std::cout << "Distancias para la partícula " << i + 1 << ":" << std::endl;
        //     for (int j = 0; j < 360; ++j) {
        //         std::cout << "Grado " << j << ": " << distances[i][j] << std::endl;
        //     }
        //     std::cout << std::endl;
        // }  
        
        calcularDistanciaMahalanobis();   
        // Mostrar las distancias de Mahalanobis calculadas
        // for (size_t i = 0; i < num_particulas; ++i) {
        //     std::cout << "Distancia de Mahalanobis para la partícula " << i + 1 << ": " << distanciaMahalanobis[i] << std::endl;
        // }        
        
        resampleParticulas();
        // for (const auto& particula : particulas) {
        //     std::cout << "Particula: {" << particula->x << ", " << particula->y << ", " << particula->theta << "}" << std::endl;
        // }
        
        calculoLocalizacion();

        if ((angulo2 - yaw > 0.01) || (angulo2 - yaw < -0.01)){
            // angulo2 = calcularAngulo(ubicacionActual, dato);
            // std::cout << "angulo2: " << angulo2 << std::endl;
            if (angulo2 - yaw > 0.01){
                speed.angular.z = 0.1;
            }else{
                speed.angular.z = -0.1;
            }
            speed.linear.x = 0;
            speed_pub.publish(speed);
            ros::spinOnce();
            loop.sleep();
        }
        else if ( (dist2 > 0.1)){ //no entro aquí hasta que esté alineado
            //dist2 = calcularDistancia(ubicacionActual, dato);
            //std::cout << "dist2: " << dist2 << std::endl;
            speed.angular.z = 0;
            speed.linear.x = 0.1;
            speed_pub.publish(speed);
            ros::spinOnce();
            loop.sleep();
        }else{
            coger_dato  = 1;
            speed.angular.z = 0;
            speed.linear.x = 0;
            speed_pub.publish(speed);
            ros::spinOnce();
            loop.sleep();
        }
    }

    return 0;

}


