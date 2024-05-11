#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include <string>
#include <cmath>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <tinyxml2.h>  // Agrega la inclusión de TinyXML
#include <cstdlib> // Para la función std::rand
#include <ctime>   // Para la función std::time
#include <vector>
#include <limits> // Para std::numeric_limits
#include <algorithm> // Para std::min
#include <std_msgs/Int32.h>

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

    Posicion operator+(const Posicion& other){
        Posicion result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
    
    }

    Posicion operator-(const Posicion& other){
        Posicion result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
    
    }
};

struct EstructuraVector
{
	double ganancia;
    double orientacion;

    EstructuraVector operator+(const EstructuraVector& other){
        EstructuraVector result;
        result.ganancia = ganancia + other.ganancia;
        result.orientacion = orientacion + other.orientacion;
        return result;
    
    }

    EstructuraVector operator-(const EstructuraVector& other){
        EstructuraVector result;
        result.ganancia = ganancia - other.ganancia;
        result.orientacion = orientacion - other.orientacion;
        return result;
    
    }
};

Posicion ubicacionActual,dato; //variable global para la ubicación actual
double angulo2,dist2,yaw;
std::vector<Posicion> puntos; //= {{4,4}, {4,2}, {2,4}}; //defino los puntos a los que quiero que vaya el robot
//size_t tamano = puntos.size(); //calculo el tamaño del vector puntos

float distanciaRobotObstaculo, medidasIzq, medidasDcha;
std::vector<float> medidasLaser;
EstructuraVector vector_target;
Posicion target,vectorTarget,vectorObstaculo,vectorVff;
EstructuraVector vector_obstaculo;
EstructuraVector vector_vff;
double alpha=0.01; //ganancia para el vff
int primeraVez=0;
bool vff_activo=0;

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
    //std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << "}" << std::endl;
    ubicacionActual= {msg->pose.pose.position.x, msg->pose.pose.position.y};
    yaw=tf::getYaw(msg->pose.pose.orientation);
    //std::cout << "Yaw: " << yaw << std::endl;
}

int numero=0;
void laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
    medidasLaser.clear();
    medidasLaser.insert(medidasLaser.begin(), scan->ranges.begin(), scan->ranges.end());
    distanciaRobotObstaculo= *std::min_element(medidasLaser.begin()+90, medidasLaser.begin()+270);
    medidasDcha= *std::min_element(medidasLaser.begin()+90, medidasLaser.begin()+180);
    medidasIzq= *std::min_element(medidasLaser.begin()+180, medidasLaser.begin()+270);
    std::cout << "Distancia del robot al obstáculo: " << distanciaRobotObstaculo << std::endl;
    std::cout << "Min izq" << medidasIzq <<std::endl;
    std::cout << "Min dcha" << medidasDcha <<std::endl;
    // std::cout<<"Medidas del laser:"<<std::endl;
    // for (auto medida : medidasLaser){
    // std::cout << numero << ": " << medida << std::endl;
    // numero++;
    // }
    // numero=0;
}

double calcularDistancia(Posicion pos1, Posicion pos2) {
    double distancia = std::sqrt(std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2));
    return distancia;
}


double calcularAngulo(Posicion pos1, Posicion pos2) {
    double angulo = std::atan2(pos2.y - pos1.y, pos2.x - pos1.x);
    return angulo;
}


double vff(){
    //distanciaRobotObstaculo=*std::min_element(medidasLaser.begin()+170, medidasLaser.end()+191);
    target.x=ubicacionActual.x-dato.x; 
    target.y=ubicacionActual.y-dato.y;
    vectorTarget.x= target.x*cos(angulo2)-target.y*sin(angulo2);
    vectorTarget.y= target.x*sin(angulo2)+target.y*cos(angulo2);

    vectorObstaculo.x = /*ubicacionActual.x-*/ (1.0/distanciaRobotObstaculo)*cos(angulo2);
    vectorObstaculo.y = /*ubicacionActual.y-*/ (1.0/distanciaRobotObstaculo)*sin(angulo2);

    vectorVff = vectorTarget + vectorObstaculo;

    float resul;
    resul = vectorVff.x*alpha;
    if ((medidasDcha>medidasIzq) && resul>0){
        resul = -vectorVff.x*alpha; 
        // if(abs(resul)>0.4)  
        //     resul = 0.2;
    }
    else if (medidasDcha<medidasIzq && resul<0){
        resul = abs(resul); 
        // if(abs(resul)>0.4)  
        //     resul = 0.2;  
    }  
    return resul;
}


int main (int argc, char** argv){

    ros::init(argc, argv, "practica2324");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/robot0/odom", 1000, odometryCallback);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1000);
    ros::Subscriber laser_sub = nh.subscribe("/robot0/laser_0",1000,laserCallback);

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
        // std::cout << "angulo2: " << angulo2 << std::endl;
        // std::cout << "dist2: " << dist2 << std::endl;
        // std::cout << "dato: {" << dato.x << ", " << dato.y << "}" << std::endl;

        geometry_msgs::Twist speed;

        if (((angulo2 - yaw > 0.01) || (angulo2 - yaw < -0.01)) && primeraVez==0 && vff_activo==0){
            // angulo2 = calcularAngulo(ubicacionActual, dato);
            // std::cout << "angulo2: " << angulo2 << std::endl;
            if (angulo2 - yaw > 0.01){
                speed.angular.z = 0.1;
            }else{
                speed.angular.z = -0.1; //girar derecha
            }
            speed.linear.x = 0;
            speed_pub.publish(speed);
            //std::cout<< "voy mal, no funciono"<<std::endl;
            ros::spinOnce();
            loop.sleep();
        }
        else if ( (dist2 > 0.1)){ //no entro aquí hasta que esté alineado
            //dist2 = calcularDistancia(ubicacionActual, dato);
            //std::cout << "dist2: " << dist2 << std::endl;
            //std::cout<< "vff es:" << vff_activo << std::endl;
            //std::cout << "La distancia del robot al obstáculo es:" << distanciaRobotObstaculo << std::endl;
            if(distanciaRobotObstaculo>4.5){
                vff_activo=0;
                //std::cout << "He desactivado el vff" << std::endl;
            }
            if(distanciaRobotObstaculo>4.0 && vff_activo==0){ 
                speed.angular.z = 0.0;
                speed.linear.x = 0.3;
                speed_pub.publish(speed);
                primeraVez=0;
            }else{
                vff_activo=1;
                primeraVez=1; 
                speed.angular.z = vff();
                std::cout << "La velocidad angular es:" << speed.angular.z << std::endl;
                speed.linear.x = 0.2;
                speed_pub.publish(speed);
            }
            ros::spinOnce();
            loop.sleep();
        }else{
            coger_dato  = 1;
            speed.angular.z = 0.0;
            speed.linear.x = 0.0;
            speed_pub.publish(speed);
            ros::spinOnce();
            loop.sleep();
        }
    }

    return 0;

}

