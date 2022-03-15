import rospy
import tf
from visualization_msgs.msg import Marker
marker_publisher = rospy.Publisher("markers", Marker, queue_size = 50)
broadcaster = tf.TransformBroadcaster()

def pubMarker(point:tuple,num:int,duration = 10,size=0.08,frame_name = "default",debug = 1,deletall=0,add = 1,type = "sphere",r=0,g=1,b=0):
    "Publishes a marker into 'markers' topic and (frame_name) namespace, types = [cube, sphere, cylinder (or arrow)]"
    #if debug:
        #rospy.loginfo(f"Pubbing marker {num} {point} in ns {frame_name}")
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
    mark.color.r = r
    mark.color.g = g
    mark.color.b = b
    mark.text = frame_name
    mark.ns = frame_name
    mark.id = num
    if type == "cube":
        mark.type = Marker.CUBE
    elif type == "sphere":
        mark.type = Marker.SPHERE
    elif type == "cylinder":
        mark.type = Marker.CYLINDER
    else:
        mark.type = Marker.ARROW
    if deletall:
        mark.action = Marker.DELETEALL
        if debug:
             rospy.loginfo(f"Deleting all markers in ns {frame_name}...")
    else:
        if add:
            mark.action = Marker.ADD
        else:
            mark.action = Marker.DELETE
    marker_publisher.publish(mark)
    #print(f"test - {frame_name}_markers")
    broadcaster.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        f"{frame_name}_markers",
        "costmap"
    )

class transform:
    @staticmethod
    def send(point:tuple,child_frame:str,frame:str):
        "Sends specified transform, while keeping main code tidy"
        broadcaster.sendTransform(
        (point[1], point[0], 0), #pasha loh
        tf.transformations.quaternion_from_euler(0,0,point[2]),
        rospy.Time.now(),
        frame,
        child_frame
    )

 