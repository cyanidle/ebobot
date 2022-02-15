import roslib
roslib.load_manifest('ebobot')
import rospy


from costmap_server import Costmap
from global_planer import Global
from local_planer import Local
import motors_info_callback




rospy.sleep(1)
    while not rospy.is_shutdown():
        #rospy.loginfo("1 cycle")
        if not Global.goal_reached:
            while not Global.goal_reached:
                Global.appendNextPos()
            if Global.cleanup_feature:
                Global.cleanupDeadEnds()
            Global.publish()
        rate.sleep()