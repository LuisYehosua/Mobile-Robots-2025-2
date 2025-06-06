#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose

# Servicio real de tu proyecto
from manip_srvs.srv import InverseKinematicsPose2Pose, InverseKinematicsPose2PoseRequest

NAME = "Luis Yehosua"

obstacle_detected = False
has_reacted = False  # bandera para no repetir acciones

def callback_scan(msg):
    global obstacle_detected
    n = int((msg.angle_max - msg.angle_min) / msg.angle_increment / 2)
    obstacle_detected = msg.ranges[n] < 1.0
    return

def crear_pose(x=0.0, y=0.0, z=0.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    # Orientación por defecto (sin rotación)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose

def main():
    global obstacle_detected, has_reacted
    print("ROS BASICS - " + NAME)
    rospy.init_node("robot_patron_movimiento")
    rospy.Subscriber("/hardware/scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Esperar servicios
    rospy.wait_for_service('/manipulation/ik_pose2pose')
    mover_parte = rospy.ServiceProxy('/manipulation/ik_pose2pose', InverseKinematicsPose2Pose)

    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg_cmd_vel = Twist()

        if not obstacle_detected:
            msg_cmd_vel.linear.x = 0.3
            has_reacted = False  # permitir nueva reacción
        else:
            msg_cmd_vel.linear.x = 0.0

            if not has_reacted:
                rospy.loginfo("Obstáculo detectado. Ejecutando patrón...")

                # Mover cabeza simulada (posición ficticia)
                req_cabeza = InverseKinematicsPose2PoseRequest()
                req_cabeza.target_pose = crear_pose(0.0, 0.2, 1.5)  # posición simulada de cabeza
                mover_parte(req_cabeza)

                rospy.sleep(1.0)  # pequeña pausa entre movimientos

                # Mover brazo derecho simulado (posición ficticia)
                req_brazo = InverseKinematicsPose2PoseRequest()
                req_brazo.target_pose = crear_pose(0.3, -0.2, 1.0)  # posición simulada de brazo derecho
                mover_parte(req_brazo)

                has_reacted = True

        pub_cmd_vel.publish(msg_cmd_vel)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
