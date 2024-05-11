#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <tf/tf.h>
#include <tinyxml2.h>
#include <iostream>
#include <fstream>

#include "CFuzzySpeedController.h"

//este código lee de un archivo xml los puntos a los que se dirige el robot y se mueve hacia ellos

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
//observaciones: mapa virtual y medidas de los láseres
//lueo se hace el matcing
//resample de las particulas

struct Posicion {
    double x;
    double y;
};

Posicion ubicacionActual,dato; //variable global para la ubicación actual
double angulo2,dist2,yaw;
std::vector<Posicion> puntos; //= {{4,4}, {4,2}, {2,4}}; //defino los puntos a los que quiero que vaya el robot
//size_t tamano = puntos.size(); //calculo el tamaño del vector puntos
CFuzzySpeedController fuzzyController;
double linearVelocity, angularVelocity;

void readXmlFile(const std::string &filename)
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

void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << "}" << std::endl;
    ubicacionActual= {msg->pose.pose.position.x, msg->pose.pose.position.y};
    yaw=tf::getYaw(msg->pose.pose.orientation);
    std::cout << "Yaw: " << yaw << std::endl;
}

double calcularDistancia(Posicion pos1, Posicion pos2) {
    double distancia = std::sqrt(std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2));
    return distancia;
}


double calcularAngulo(Posicion pos1, Posicion pos2) {
    double angulo = std::atan2(pos2.y - pos1.y, pos2.x - pos1.x);
    return angulo;
}


int main (int argc, char** argv){

    ros::init(argc, argv, "practica2324");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/robot0/odom", 1000, odometryCallback);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1000);

    std::string xmlFilePath = "/home/alumno/robotica_movil_ws/src/practica1/src/map.xml";
    readXmlFile(xmlFilePath);


    ros::Rate loop(10);

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
        std::cout << "angulo2: " << angulo2 << std::endl;
        std::cout << "dist2: " << dist2 << std::endl;
        std::cout << "dato: {" << dato.x << ", " << dato.y << "}" << std::endl;

        geometry_msgs::Twist speed;

        fuzzyController.getSystemInput(dist2, angulo2, &linearVelocity, &angularVelocity);
        speed.linear.x = linearVelocity;
        speed.angular.z = angularVelocity;
        std::cout << "linearVelocity: " << linearVelocity << std::endl;
        std::cout << "angularVelocity: " << angularVelocity << std::endl;
        speed_pub.publish(speed);
        ros::spinOnce();
        loop.sleep();
        if(dist2 < 0.1){
            coger_dato = 1;
        }

        // if ((angulo2 - yaw > 0.01) || (angulo2 - yaw < -0.01)){
        //     // angulo2 = calcularAngulo(ubicacionActual, dato);
        //     // std::cout << "angulo2: " << angulo2 << std::endl;
        //     if (angulo2 - yaw > 0.01){
        //         speed.angular.z = 0.1;
        //     }else{
        //         speed.angular.z = -0.1;
        //     }
        //     speed.linear.x = 0;
        //     speed_pub.publish(speed);
        //     ros::spinOnce();
        //     loop.sleep();
        // }
        // else if ( (dist2 > 0.1)){ //no entro aquí hasta que esté alineado
        //     //dist2 = calcularDistancia(ubicacionActual, dato);
        //     //std::cout << "dist2: " << dist2 << std::endl;
        //     speed.angular.z = 0;
        //     speed.linear.x = 0.2;
        //     speed_pub.publish(speed);
        //     ros::spinOnce();
        //     loop.sleep();
        // }else{
        //     coger_dato  = 1;
        //     speed.angular.z = 0;
        //     speed.linear.x = 0;
        //     speed_pub.publish(speed);
        //     ros::spinOnce();
        //     loop.sleep();
        // }
    }

    return 0;

}

