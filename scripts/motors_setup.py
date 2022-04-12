#!/usr/bin/env python3
import rospy
import roslib
import yaml
roslib.load_manifest('ebobot')
#from sys import argv
rospy.init_node('motors_setup')
#
from ebobot.srv import NewMotor, NewMotorRequest
#
def read(file):
        with open(file, "r") as stream:
            try:
                route = (yaml.safe_load(stream))
                return route
            except yaml.YAMLError as exc:
                rospy.logerr(f"Loading failed ({exc})")
def main():
    rospy.logwarn(f"Setting values for motors...")
    file = rospy.get_param("~file", "config/motors.yaml")
    req = NewMotorRequest()
    dict = read(file)
    for num in dict:
        rospy.logwarn(f"Initialising motor {num}")
        try:
            vals = dict[num]
            req.motor = int(num)
            req.angle = vals["angle"]
            req.pid.P = vals["p"]
            req.pid.I = vals["i"]
            req.pid.D = vals["d"]
            req.pin_layout.encoder_a = vals["encoder_a"]
            req.pin_layout.encoder_b = vals["encoder_b"]
            req.pin_layout.pwm = vals["pwm"]
            req.pin_layout.fwd_dir = vals["fwd_dir"]
            req.pin_layout.back_dir = vals["back_dir"]
            req.wheel_rad = vals["wheel_rad"]
            req.ticks_per_rotation = vals["ticks_per_rotation"]  
            ################# Global
            req.turn_max_speed = vals["turn_max_speed"]
            req.max_speed = vals["max_speed"]
        except:
            rospy.logerr(f"Incorrect syntax for motor {num}!")
        client(req)
        rospy.sleep(0.5)
def client(req):
    rospy.wait_for_service("motors_settings_service")
    try:
        servos = rospy.ServiceProxy("motors_settings_service", NewMotor)
        resp = servos(req)
        rospy.loginfo(f"{resp.resp}")
    except rospy.ServiceException as e:
        rospy.logerr(f"ERROR {e}")
if __name__=="__main__":
    rospy.sleep(5)
    main()
