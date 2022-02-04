#!/usr/bin/env python3
import rospy
from sys import argv
path = argv ###две волшебные строки с первой ссылки в гугле, path - просто переменная
from std_msgs.msg import Float32
rospy.init_node('pid_setter_node')
pid_msg = Float32()
pub = rospy.Publisher('set_pid', Float32 ,queue_size = 15)
rospy.sleep(1)
with open(f"{path[1]}", 'r') as f: ## хуй знает почему так, но 0 аргумент это путь на сам иполняемый файл
    pids = f.readlines()
    for coeff in pids:
        for val in coeff.split(" "):
            pid_msg.data = float(val) 
            pub.publish(pid_msg)
            rospy.loginfo(f"Published {val}")
            rospy.sleep(0.1)


