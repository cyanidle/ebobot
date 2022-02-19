import roslib
roslib.load_manifest('ebobot')
import rospy
import tf
########################NOT NEEDED
def main():
    rospy.init_node("map")
    map_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        zero_quat = tf.transformations.quaternion_from_euler(0,0,0)
        map_broadcaster.sendTransform(
            (0, 0, 0),
            zero_quat,
            rospy.Time.now() ,
            "costmap",
            "map"
            )
        rospy.sleep(2)

if __name__=="__main__":
    main()


