#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

positions_x = []
positions_y = []

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    positions_x.append(x)
    positions_y.append(y)
    rospy.loginfo(f"Posición del robot: x={x:.2f}, y={y:.2f}")

if __name__ == '__main__':
    rospy.init_node('dibuja_mov', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    rospy.loginfo("Nodo dibuja_mov ejecutándose. Presiona Ctrl+C para detener y graficar la trayectoria.")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Detenido. Generando gráfico de la trayectoria...")
    
    # Graficar la trayectoria
    plt.figure()
    plt.plot(positions_x, positions_y, marker='o')
    plt.xlabel("Posición X")
    plt.ylabel("Posición Y")
    plt.title("Trayectoria del robot")
    plt.grid(True)
    plt.show()

