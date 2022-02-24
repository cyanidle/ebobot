@classmethod
    def pubPoint(cls,point,num):
        if cls.debug:
            rospy.loginfo(f"Pubbing marker {point}")
        mark = Marker()
        mark.header.frame_id = "costmap"
        mark.header.stamp = rospy.Time.now()
        mark.lifetime = rospy.Duration(5)
        mark.pose.position.y = point[0]
        mark.pose.position.x = point[1]
        mark.pose.orientation.x = 0
        mark.pose.orientation.y = 0
        mark.pose.orientation.z = 0
        mark.pose.orientation.w = 1
        mark.scale.x = 0.05
        mark.scale.y = 0.05
        mark.scale.z = 0.05
        mark.color.a = 1.0 ##Don't forget to set the alpha!
        mark.color.r = 0.0
        mark.color.g = 0.0
        mark.color.b = 1.0
        mark.ns = "Locals"
        mark.id = num
        mark.type = Marker.CUBE
        mark.action = Marker.ADD
        cls.point_publisher.publish(mark)


 @classmethod
    def pubPath(cls):
        msg = Path()
        rviz_coeff = (1/cls.costmap_resolution)
        targ = PoseStamped()
        curr = PoseStamped()
        curr.header.frame_id = "costmap"
        curr.header.stamp = rospy.Time.now()
        targ.header.frame_id = "costmap"
        targ.header.stamp = rospy.Time.now()
        curr.pose.position.x = cls.robot_pos[1] / rviz_coeff
        curr.pose.position.y = cls.robot_pos[0]/rviz_coeff
        targ.pose.position.x = cls.actual_target[1] / rviz_coeff
        targ.pose.position.y = cls.actual_target[0]/rviz_coeff
        #rviz_quat = tf.transformations.quaternion_from_euler(0, 0, cls.actual_target[2]/ rviz_coeff)
        targ.pose.orientation.x,curr.pose.orientation.x = 0,0
        targ.pose.orientation.y,curr.pose.orientation.y = 0,0
        targ.pose.orientation.z ,curr.pose.orientation.z= 0,0
        targ.pose.orientation.w,curr.pose.orientation.w = 1,1
        msg.poses.append(curr)
        msg.poses.append(targ)
        cls.tfBroadcast(cls.robot_pos[0] / rviz_coeff,   cls.robot_pos[1]/rviz_coeff,     0)
        cls.tfBroadcast(cls.actual_target[0] / rviz_coeff,     cls.actual_target[1]/rviz_coeff,    0)
        #cls.sendTransfrom(cls.actual_target[0]/ rviz_coeff,cls.actual_target[1]/ rviz_coeff,cls.actual_target[2])
        cls.rviz_publisher.publish(msg)
    @classmethod
    def tfBroadcast(cls,x,y,th):
        cls.rviz_broadcaster.sendTransform(
            (x, y, 0),
            tf.transformations.quaternion_from_euler(0,0,th),
            rospy.Time.now(),
            "odom",
            "rviz_local_path"
        )