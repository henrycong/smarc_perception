#!/usr/bin/env python3
import rospy
from sss_object_detection.consts import ObjectID
from vision_msgs.msg import ObjectHypothesisWithPose, Detection2DArray, Detection2D
import tf2_ros
import tf2_geometry_msgs
from smarc_msgs.msg import GotoWaypoint
from nav_msgs.msg import Odometry

import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor


class sss_detection_listener:
    def __init__(self, robot_name):

        self.rope_pose = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_pose = None
        self.odom_sub = rospy.Subscriber(
            '/{}/dr/odom'.format(robot_name), Odometry,
            self.update_pose)

        self.detection_topic = '/{}/payload/sidescan/detection_hypothesis'.format(
            robot_name)
        self.detection_sub = rospy.Subscriber(self.detection_topic, Detection2DArray,
                                         self.detection_callback)
        self.waypoint_topic = '/{}/ctrl/goto_waypoint'.format(robot_name)
        self.waypoint_topic_type = GotoWaypoint #ROS topic type
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, self.waypoint_topic_type
                 #                            , queue_size=5)
                ,self.waypoint_topic_type, queue_size=5)
       
        print(self.waypoint_topic)

    def wait_for_transform(self, from_frame, to_frame):
        """Wait for transform from from_frame to to_frame"""
        trans = None
        while trans is None:
            try:
                trans = self.tf_buffer.lookup_transform(
                    to_frame, from_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                print('Failed to transform. Error: {}'.format(error))
        return trans

    def transform_pose(self, pose, from_frame, to_frame):
        trans = self.wait_for_transform(from_frame=from_frame,
                                         to_frame=to_frame)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed

    def update_pose(self, msg):
        # might need to transform pose to another frame
        #to_frame = 'utm'
        #object_pose = self.transform_pose(msg, from_frame=msg.header.frame_id, to_frame=to_frame)
        self.current_pose = msg

    def detection_callback(self, msg):
        # Assume each Detection2DArray only contains one Detection2D message
        # Further assume each Detection2D only contains one ObjectHypothesisWithPose
        object_hypothesis = msg.detections[0].results[0]
        object_frame_id = msg.header.frame_id[1:]
        # Pose msg
        object_pose = object_hypothesis.pose
        detection_confidence = object_hypothesis.score
        object_id = object_hypothesis.id

        to_frame = 'utm'
        object_pose = self.transform_pose(object_pose, from_frame=object_frame_id, to_frame=to_frame)

        if object_id == ObjectID.ROPE.value:
            print('Detected rope at frame {}, pose {}, with confidence {}'.format(
                object_frame_id, object_pose, detection_confidence))
            # Do whatever you want to do
           
            self.rope_pose.append(object_pose)
            
            if self.rope_pose[len(self.rope_pose)-1][1] >= 15:

            #print >>sys.stderr, 'counter1 =  "%s"'  % self.counter 
            #print >>sys.stderr, 'range1 =  "%s"'  % int(points_np[len(points_np)-1][1])
            # #if self.counter <= int(points_np[len(points_np)-1][1]):
            #     #print >>sys.stderr, 'counter2 =  "%s"'  % self.counter 
            #     #print >>sys.stderr, 'range2 =  "%s"'  % int(points_np[len(points_np)-1][1])
            #     #"""Changing the corrdinates as in the moving point cloud self.Y increases as the robot moves forward"""
                if self.rope_pose[len(self.rope_pose)-1][1] <= 50:
                     X= np.array(self.rope_pose.y)
                     y= np.array(self.rope_pose.x)
                else: 
                     X= np.array(self.rope_pose.y[-50:])
                     y= np.array(self.rope_pose.x[-50:])
                

                 
                ransac = RANSACRegressor(LinearRegression(), 
                             max_trials=100, 
                             min_samples=10, 
                             residual_threshold=0.001)
                ransac.fit(X.reshape(-1,1), y)
                inlier_mask = ransac.inlier_mask_
                outlier_mask = np.logical_not(inlier_mask)


            #    predicted_intercept_map =PointStamped()
            #    predicted_intercept_map.header.frame_id = "map"
            #    predicted_intercept_map.header.stamp = rospy.Time(0)
                x_pred = np.float64(X[len(X)-1]+5.0)
            #    #self.counter = int(x_pred)
                m_slope = ransac.estimator_.coef_[0]
            #    #print >>sys.stderr,'m_slope: ' % m_slope
                c_intercept = ransac.estimator_.intercept_
            #    #print >>sys.stderr,'c_intercept: ' % c_intercept
            
                y_pred = (x_pred*m_slope) + c_intercept
                # get pose of SAM
                auv_pose = self.transform_pose(self.odom_sub, from_frame=object_frame_id, to_frame=to_frame)
                x_auv=auv_pose.x
                y_auv=auv_pose.y
                
                #caulate waypoint
                #设直线方程为ax+by+c=0,点坐标为(m,n) 则垂足为((b*b*m-a*b*n-a*c)/(a*a+b*b),(a*a*n-a*b*m-b*c)/(a*a+b*b))
                #a=m_slope b=-1 c=c_intercept m=x_auv n=y_auv
                vpoint=[((x_auv+m_slope*y_auv-m_slope*c_intercept)/(m_slope*m_slope+1),
                       (m_slope*m_slope*y_auv+m_slope*x_auv+c_intercept)/(m_slope*m_slope+1))]
                if m_slope<20:#test if m_slope always>0
                    x_distanGo=5/(1+m_slope^2)
                    if auv_pose.heading < 180:
                        Waypoint_x=(vpoint[1]+x_distanGo)*2/3+x_auv*1/3
                        Waypoint_y=((vpoint[1]+x_distanGo)*m_slope+c_intercept)*2/3+y_auv*1/3
                    else:
                        Waypoint_x=(vpoint[1]-x_distanGo)*2/3+x_auv*1/3
                        Waypoint_y=((vpoint[1]-x_distanGo)*m_slope+c_intercept)*2/3+y_auv*1/3
                else:
                    x_togo=0
                    y_togo=5
                    if auv_pose.heading < 90 or auv_pose.heading > 270:
                        Waypoint_x=(vpoint[1]+x_togo)*2/3+x_auv*1/3
                        Waypoint_y=((vpoint[1]+y_togo)*m_slope+c_intercept)*2/3+y_auv*1/3
                    else:
                        Waypoint_x=(vpoint[1]-x_togo)*2/3+x_auv*1/3
                        Waypoint_y=(vpoint[1]-y_togo)*2/3+y_auv*1/3
                
                self.publish_waypoint()
        if object_id == ObjectID.BUOY.value:
            pass
            #print('Detected buoy at frame {}, pose {}, with confidence {}'.format(
            #    object_frame_id, object_pose, detection_confidence))
        if object_id == ObjectID.NADIR.value:
            pass
            #print('Detected nadir at frame {}, pose {}, with confidence {}'.format(
            #    object_frame_id, object_pose, detection_confidence))

    def publish_waypoint(self):
        print('Publishing waypoint')
        msg = GotoWaypoint()
        msg.waypoint_pose = self.rope_pose[-1]
        msg.waypoint_pose.header.stamp = rospy.Time(0)
        msg.goal_tolerance = 2
        self.waypoint_pub.publish(msg)


def main():
    rospy.init_node('sss_detection_listener', anonymous=True)
    rospy.Rate(5)  # ROS Rate at 5Hz

    robot_name_param = '~robot_name'
    if rospy.has_param(robot_name_param):
        robot_name = rospy.get_param(robot_name_param)
        print('Getting robot_name = {} from param server'.format(robot_name))
    else:
        robot_name = 'sam'
        print('{} param not found in param server.\n'.format(robot_name_param))
        print('Setting robot_name = {} default value.1231233'.format(robot_name))
    
    listner = sss_detection_listener(robot_name)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
