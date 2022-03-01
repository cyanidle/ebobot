import rospy
import tf
from visualization_msgs.msg import Marker

marker_publisher = rospy.Publisher("markers", Marker, queue_size = 10)
broadcaster = tf.TransformBroadcaster()

def pubMarker(cls,point,num,duration = 1,size=0.05,frame_name = "default"):
    if cls.debug:
        rospy.loginfo(f"Pubbing marker {point}")
    mark = Marker()
    mark.header.frame_id = f"{frame_name}_markers"
    mark.header.stamp = rospy.Time.now()
    mark.lifetime = rospy.Duration(duration)
    mark.pose.position.y = point[0]
    mark.pose.position.x = point[1]
    mark.pose.orientation.x = 0
    mark.pose.orientation.y = 0
    mark.pose.orientation.z = 0
    mark.pose.orientation.w = 1
    mark.scale.x = size
    mark.scale.y = size
    mark.scale.z = size
    mark.color.a = 1.0 ##Don't forget to set the alpha!
    mark.color.r = 0.0
    mark.color.g = 0.0
    mark.color.b = 1.0
    mark.ns = frame_name
    mark.id = num
    mark.type = Marker.CUBE
    mark.action = Marker.ADD
    marker_publisher.publish(mark)
    broadcaster.sendTransform(
        (point[0], point[1], 0),
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        f"{frame_name}_markers",
        "costmap"
    )

 