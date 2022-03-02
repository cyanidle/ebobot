import rospy
import tf
from visualization_msgs.msg import Marker

marker_publisher = rospy.Publisher("markers", Marker, queue_size = 10)
broadcaster = tf.TransformBroadcaster()

def pubMarker(point:tuple,num:int,duration = 10,size=0.08,frame_name = "default",debug = 1,add = 1,type = "sphere"):
    "Publishes a marker into 'markers' topic and (frame_name) namespace"
    if debug:
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
    mark.color.a = 1.0 ##Don't forget to set the alpha! (male)
    mark.color.r = 0.0
    mark.color.g = 1.0
    mark.color.b = 0.0
    mark.ns = frame_name
    mark.id = num
    if type == "cube":
        mark.type = Marker.CUBE
    elif type == "sphere":
        mark.type = Marker.SPHERE
    else:
        mark.type = Marker.ARROW
    if add:
        mark.action = Marker.ADD
    else:
        mark.action = Marker.DELETE
    marker_publisher.publish(mark)
    
    broadcaster.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        f"{frame_name}_markers",
        f"costmap"
    )

 