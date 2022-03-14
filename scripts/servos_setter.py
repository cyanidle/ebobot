#!/usr/bin/env python3
import rospy
import roslib
roslib.load_manifest('ebobot')
#from sys import argv
rospy.init_node('servos_setter')
#
from ebobot.srv import Servos, ServosRequest, ServosResponse
#
import yaml
def read(file):
        with open(file, "r") as stream:
            try:
                route = (yaml.safe_load(stream))
                return route
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
def main():
    file = rospy.get_param("~/file", "config/servos.yaml")
    dict = read(file)
    for num in dict:
        vals = dict[num]
        req = ServosRequest()
        req.num = num
        req.channel = vals[0]
        req.speed = vals[1]
        req.min_val = vals[2]
        req.max_val = vals[3]
        client(req)


def client(req):
    rospy.wait_for_service("servos_settings_service")
    try:
        servos = rospy.ServiceProxy("servos_settings_service", Servos)
        resp = servos(req)
        rospy.loginfo(f"{resp.resp}")
    except rospy.ServiceException as e:
        rospy.logerr(f"ERROR {e}")




if __name__=="__main__":
    rospy.sleep(2)
    main()