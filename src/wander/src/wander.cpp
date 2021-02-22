#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include <cstdlib> 
#include <ctime> 
#include <limits>
#include <cmath>


enum Estado
{
	mover_delante,
	girar_izquierda,
	girar_derecha
};

class Wander
{
	protected:
		ros::Publisher commandPub; 
		ros::Subscriber laserSub; 
		ros::Subscriber odometry;

		static const float MIN_DISTANCE_TURN = 1;
		static const double FORWARD_SPEED = 1;
		static const double TURNING_FORWARD_SPEED = 0;
		static const double TURNING_ANGULAR_SPEED = 2;
		static const uint16_t BARRIDO_DETECT_MIN = 35; //520 para visión completa (por arriba y por abajo)
		static const uint16_t BARRIDO_DETECT_MAX = 200;

		static const float SUB_DELTA = 0.1f;

		float min_distance_front;
		float max_distance_front;
		float distance_exactly_front;
		float max_distance_front_seek;
		bool ha_girado_derecha;

		bool approx_equals(float a, float b)
		{
			return std::abs(b - a) <= SUB_DELTA;
		}

		Estado transicion(Estado e0)
		{
			switch (e0)
			{
				case mover_delante:
					if (min_distance_front <= MIN_DISTANCE_TURN)
					{
						max_distance_front_seek = max_distance_front;
						if (ha_girado_derecha)
						{
							ha_girado_derecha = false;
							return girar_izquierda;
						}
						ha_girado_derecha = true;
						return girar_derecha;
					}
					return mover_delante;
				case girar_izquierda:
					if (approx_equals(distance_exactly_front, max_distance_front_seek))
						return mover_delante;
					return girar_izquierda;
				case girar_derecha:
					if (approx_equals(distance_exactly_front, max_distance_front_seek))
						return mover_delante;
					return girar_derecha;
			}
		}

		void print_estado(Estado e)
		{
			switch (e)
			{
				case mover_delante:
					printf("Estado: moviendo delante\n");
					break;
				case girar_izquierda:
					printf("Estado: girando izquierda\n");
					break;
				case girar_derecha:
					printf("Estado: girando derecha\n");
					break;
			}
		}

		void aplicar_estado(double& velocidad_lineal, double& velocidad_angular, Estado e0)
		{
			switch (e0)
			{
				case mover_delante:
					velocidad_lineal = FORWARD_SPEED;
					velocidad_angular = 0;
					break;
				case girar_derecha:
					velocidad_lineal = TURNING_FORWARD_SPEED;
					velocidad_angular = TURNING_ANGULAR_SPEED;
					break;
				case girar_izquierda:
					velocidad_lineal = TURNING_FORWARD_SPEED;
					velocidad_angular = -TURNING_ANGULAR_SPEED;
					break;
			}
		}

	public:
		Wander(ros::NodeHandle& nh){ //: rotateStartTime(ros::Time::now()), rotateDuration(0.f) {
			min_distance_front = std::numeric_limits<float>::max();
			max_distance_front = std::numeric_limits<float>::min();
			distance_exactly_front = 5;
			max_distance_front_seek = 0;
			ha_girado_derecha = false;
			min_distance_front = 5;
			// Este método nos permite indicar al sistema que vamos a publicar mensajes de cmd_vel
			// El valor de 1 indica que si acumulamos varios mensajes, solo el último será enviado.
			// El método devuelve el Publisher que recibirá los mensajes.
			commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			// Suscribe el método commandCallback al tópico base_scan (el láser proporcionado por Stage)
			// El método commandCallback será llamado cada vez que el emisor (stage) publique datos 
			laserSub = nh.subscribe("base_scan", 1, &Wander::commandCallback, this);
			odometry = nh.subscribe("odom", 1, &Wander::commandOdom, this);
		};

		void commandOdom(const nav_msgs::Odometry::ConstPtr &msg)
		{
			// ROS_INFO_STREAM("Odometry x: " << msg->pose.pose.position.x);
			// ROS_INFO_STREAM("Odometry y: " << msg->pose.pose.position.y);
			// ROS_INFO_STREAM("Odometry angz: " << 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
		}

		// Procesa los datos de láser
		void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
			// printf("Ángulo mínimo %.10f\n", msg->angle_min);
			// printf("Ángulo máximo %.10f\n\n", msg->angle_max);
			// printf("Ángulo incremento %.10f\n\n", msg->angle_increment);
			// printf("Distancia mínima %.4f\n\n", msg->range_min);
			// printf("Distancia máxima %.4f\n\n", msg->range_max);
			// printf("Elementos en vector %lu\n\n", msg->ranges.size());
			min_distance_front = std::numeric_limits<float>::max();
			max_distance_front = std::numeric_limits<float>::min();
			const size_t n = msg->ranges.size();
			distance_exactly_front = msg->ranges[n / 2];
			for (size_t i = n / 2 - BARRIDO_DETECT_MIN; i <= n  / 2 + BARRIDO_DETECT_MIN; ++i)
				min_distance_front = std::min(min_distance_front, msg->ranges[i]);
			for (size_t i = n / 2 - BARRIDO_DETECT_MAX; i <= n / 2 + BARRIDO_DETECT_MAX; ++i)
				max_distance_front = std::max(max_distance_front, msg->ranges[i]);
			printf("Distancia mínima en frente %.4f\n", min_distance_front);
			printf("Distancia máxima en frente %.4f\n", max_distance_front);
		};

		// Bucle principal
		void bucle()
		{
			Estado e = mover_delante;
			ros::Rate rate(10); // Especifica el tiempo de bucle en Hertzios. Ahora está en ciclo por segundo, pero normalmente usaremos un valor de 10 (un ciclo cada 100ms).
			while (ros::ok())
			{ // Bucle que estaremos ejecutando hasta que paremos este nodo o el roscore pare.
				geometry_msgs::Twist msg; // Este mensaje está compuesto por dos componentes: linear y angular. Permite especificar dichas velocidades
							  //  Cada componente tiene tres posibles valores: x, y, z, para cada componente de la velocidad. En el caso de
							  // robots que reciben velocidad linear y angular, debemos especificar la x linear y la z angular.
				e = transicion(e);
				print_estado(e);
				aplicar_estado(msg.linear.x, msg.angular.z, e);
				commandPub.publish(msg);		
				ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
				rate.sleep(); // Espera a que finalice el ciclo
			}
		}
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "wander"); // Inicializa un nuevo nodo llamado wander
	ros::NodeHandle nh;
	Wander wand(nh); // Crea un objeto de esta clase y lo asocia con roscore
	wand.bucle(); // Ejecuta el bucle
	return 0;
};

