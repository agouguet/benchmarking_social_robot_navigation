#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from math import atan2, sqrt, cos, sin, pi

class DWANavigation(Node):

    def __init__(self):
        super().__init__('dwa_navigation')

        # Publisher pour envoyer les commandes de vitesse
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher pour les marqueurs de visualisation
        self.publisher_marker = self.create_publisher(Marker, '/goal_marker', 10)

        # Subscriber pour recevoir les données LIDAR
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Subscriber pour recevoir la position actuelle (Odometry)
        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscriber pour recevoir le but
        self.subscriber_goal = self.create_subscription(
            PoseStamped,
            '/global_goal',
            self.goal_callback,
            10
        )

        # Timer pour la boucle de contrôle (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Initialisation des variables de position et d'orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Variables pour les données LIDAR
        self.obstacle_distances = []

        # Paramètres de vitesse du robot
        self.max_speed = 0.5  # Vitesse linéaire maximale (m/s)
        self.max_angular_speed = 1.0  # Vitesse angulaire maximale (rad/s)
        self.min_speed = 0.1  # Vitesse linéaire minimale
        self.min_angular_speed = -1.0  # Vitesse angulaire minimale
        self.dt = 0.1  # Intervalle de temps de simulation (s)
        self.predict_time = 2.0  # Temps de prédiction de la trajectoire (s)

        # Paramètres du but
        self.goal_x = 0.0  # Position en x du but
        self.goal_y = 0.0  # Position en y du but

    def laser_callback(self, msg):
        # Met à jour les distances des obstacles à partir du LIDAR
        self.obstacle_distances = msg.ranges

    def odom_callback(self, msg):
        # Met à jour la position du robot avec les données d'odométrie
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Calcul de l'orientation (yaw) du robot
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_theta = self.euler_from_quaternion(orientation_q)

    def goal_callback(self, msg):
        """Callback pour le but."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.publish_goal_marker()

    def euler_from_quaternion(self, quat):
        """Convertir quaternion en angles d'Euler (roll, pitch, yaw)."""
        import math
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw

    def publish_goal_marker(self):
        """Publier un marqueur pour visualiser le but dans RViz."""
        marker = Marker()
        marker.header.frame_id = "map"  # Le cadre de référence
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0  # Rouge
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opacité
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()  # Durée de vie infinie
        self.publisher_marker.publish(marker)

    def generate_trajectories(self):
        """Génère des trajectoires potentielles en balayant les vitesses possibles."""
        trajectories = []
        for v in range(int(self.min_speed * 10), int(self.max_speed * 10), 1):
            for w in range(int(self.min_angular_speed * 10), int(self.max_angular_speed * 10), 1):
                v = v / 10.0
                w = w / 10.0
                trajectory = self.simulate_trajectory(v, w)
                trajectories.append((v, w, trajectory))
        return trajectories

    def simulate_trajectory(self, v, w):
        """Simule la trajectoire d'un robot en fonction des vitesses données."""
        x = self.robot_x
        y = self.robot_y
        theta = self.robot_theta
        trajectory = []

        for _ in range(int(self.predict_time / self.dt)):
            # Calculer la nouvelle position du robot en fonction des vitesses
            x += v * cos(theta) * self.dt
            y += v * sin(theta) * self.dt
            theta += w * self.dt

            trajectory.append((x, y, theta))

        return trajectory

    def evaluate_trajectory(self, trajectory):
        """Évalue une trajectoire en fonction de trois critères : distance au but, évitement des obstacles et vitesse."""
        final_pos = trajectory[-1]
        goal_score = self.distance_to_goal(final_pos[0], final_pos[1])
        obstacle_score = self.calculate_obstacle_score(trajectory)
        speed_score = trajectory[0][0]  # Vitesse linéaire initiale

        # Ajuster la pondération pour favoriser la distance au but
        total_score = -goal_score + obstacle_score * 10 + speed_score * 0.5
        return total_score

    def distance_to_goal(self, x, y):
        """Calcule la distance entre la position actuelle et le but."""
        return sqrt((self.goal_x - x) ** 2 + (self.goal_y - y) ** 2)

    def calculate_obstacle_score(self, trajectory):
        """Calcule la distance minimale d'une trajectoire aux obstacles."""
        min_distance = float('inf')

        for x, y, _ in trajectory:
            for i, obstacle_distance in enumerate(self.obstacle_distances):
                if obstacle_distance > 0.1:  # Filtrer les valeurs nulles
                    angle = i * (2 * pi / len(self.obstacle_distances)) - pi
                    obstacle_x = x + obstacle_distance * cos(angle)
                    obstacle_y = y + obstacle_distance * sin(angle)
                    distance = sqrt((x - obstacle_x) ** 2 + (y - obstacle_y) ** 2)
                    if distance < min_distance:
                        min_distance = distance
        return min_distance

    def control_loop(self):
        """Boucle principale qui génère des trajectoires, les évalue et choisit la meilleure."""
        best_trajectory = None
        best_score = float('-inf')

        trajectories = self.generate_trajectories()

        for v, w, trajectory in trajectories:
            total_score = self.evaluate_trajectory(trajectory)

            if total_score > best_score:
                best_score = total_score
                best_trajectory = (v, w)

        if best_trajectory:
            v, w = best_trajectory
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = v
            cmd_vel_msg.angular.z = w
            self.publisher_cmd_vel.publish(cmd_vel_msg)

            # Imprimer les vitesses choisies pour le débogage
            self.get_logger().info(f'Vitesse linéaire: {v}, Vitesse angulaire: {w}')


def main(args=None):
    rclpy.init(args=args)
    dwa_nav_node = DWANavigation()
    rclpy.spin(dwa_nav_node)

    # Arrêt propre du noeud
    dwa_nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
