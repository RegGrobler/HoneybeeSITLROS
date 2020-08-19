#!/usr/bin/env python
# ROS python API
from __future__ import print_function
import rospy
import numpy as np
# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

import roslib
import cv2
# import other system/utils
import time, sys, math

from fiducial_msgs.msg import FiducialTransformArray
from tf2_msgs.msg import TFMessage 
from sensor_msgs.msg import NavSatFix

# Global variables

threshold = 0.1 # How small the position error and velocity should be before sending the next waypoint
waypoint_time = 8 # If < 0, will send next waypoint when current one is reached. If >= 0, will send next waypoint after the amount of time has passed
y = 6.5 #the y position of the gps location sent to the drone
x = 4.5 #the x position of the gps location sent tp the drone
z = 9   #the z position of the aruco marker
yaw = 0 #commanded yaw
timeout_time = 2
marker_hover_z = 1.5
vision_threshold = 0.05
error = 2 #error in relative circle marker detection threshold. If the current measurement - the previous measurement is larger than this value then an error occured and the current value will be ignored
land_count = 0
# Flight modes class

# Flight modes are activated using ROS services
class FlightModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print ("Service arming call failed: %s"%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Land Mode could not be set."%e)

    def setTakeoffMode(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 7, latitude = -34.046623, longitude = 18.740375)
        except rospy.ServiceException, e:
            print ("Service takeoff call failed: %s"%e)

# Offboard controller for sending setpoints
class Controller:
    def __init__(self):
        # Drone state
        self.state = State()

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        # A Message for the current linear velocity of the drone
        self.local_vel = Vector3(0.0, 0.0, 0.0)

        # Instantiate the position setpoint message
        self.pos_sp = PositionTarget()
        # set the flag to control height
        self.pos_sp.type_mask = int('100111111000', 2)
        # LOCAL_NED
        self.pos_sp.coordinate_frame = 1
        # initial values for setpoints
        self.pos_sp.position.x = 0.0
        self.pos_sp.position.y = 0.0
        self.pos_sp.position.z = 0.0
        self.pos_sp.yaw = math.radians(90)

        self.marker_sp = PositionTarget()
        self.marker_sp.type_mask = int('100111111000', 2)
        self.marker_sp.coordinate_frame = 1
        self.marker_sp.position.x = 0.0
        self.marker_sp.position.y = 0.0
        self.marker_sp.position.z = 0.0
        self.marker_sp.yaw = math.radians(90)
        self.marker_id = 0

        self.circle_marker_sp = PositionTarget()
        self.circle_marker_sp.type_mask = int('100111111000', 2)
        self.circle_marker_sp.coordinate_frame = 1
        self.circle_marker_sp.position.x = 0.0
        self.circle_marker_sp.position.y = 0.0
        self.circle_marker_sp.position.z = 0.0
        self.circle_marker_sp.yaw = math.radians(90)
        self.circle_marker_spotted = 1
        self.circle_marker_error_flag = 0
        self.counter = 0
        self.circle_latitude = 0
        self.circle_longitude = 0
        self.land_flag = 1
        
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("SquareMarker",Image,queue_size=1)

        self.timer_flag = 0
        self.last_time = 0
        self.imagerate = rospy.Rate(20.0)




	#Instantiate the markerTarget message
	self.marker = MarkerTarget()
	self.marker.marker_position.x = 0.0
	self.marker.marker_position.y = 0.0
	self.marker.marker_position.z = 0.0
	self.marker.marker_spotted = 0.0
    

    # Update setpoint message
    def updateSp(self, n_step, e_step, d_step, yaw_step):
        # Set step value
        self.pos_sp.position.y = n_step
        self.pos_sp.position.x = e_step
        self.pos_sp.position.z = -d_step
        self.pos_sp.yaw = math.radians(90 + yaw_step)

        # Set mask
        self.pos_sp.type_mask = int('100111111000', 2) 

    def updateSp_vel(self,z_step):
        self.pos_sp.velocity.z = z_step

    def updateMarkerSp(self,x_rel,y_rel,z_rel,marker_spotted):
	self.marker.marker_position.x = x_rel
	self.marker.marker_position.y = y_rel
	self.marker.marker_position.z = z_rel
	self.marker.marker_spotted = marker_spotted
    # Callbacks.

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Drone local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone linear velocity callback
    def velCb(self, msg):
        self.local_vel.x = msg.twist.linear.x
        self.local_vel.y = msg.twist.linear.y
        self.local_vel.z = msg.twist.linear.z

    def arucoCb(self,msg):
        try:
            self.marker_sp.position.x = msg.transforms[0].transform.translation.x
            self.marker_sp.position.y = -msg.transforms[0].transform.translation.y
            self.marker_sp.position.z = msg.transforms[0].transform.translation.z
            self.marker_id = msg.transforms[0].fiducial_id
        except:
            
            #self.marker_sp.position.x = 0
            #self.marker_sp.position.y = 0
            #self.marker_sp.position.z = 0
            self.marker_id = 0
            pass
    
    
    def gpsCb(self,msg):
        self.circle_latitude = msg.latitude
        self.circle_longitude = msg.longitude


    def imageCb(self,msg):
        try:
            
            cv_image_raw = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            self.imagerate.sleep()
            cv_image = cv2.resize(cv_image_raw,(640,480))
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, threshold = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
            _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contour_count = 1

            for cnt in contours:

                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                a_min_480 = -2.45665871e+04
                a_max_480 = -6.24018837e+04
                b_min_480 = 7.84733531e-01 
                b_max_480 =  7.95711615e-01
                c_min_480 = 3.26416165e+02 
                c_max_480 = 9.07711110e+02
                marker_width_max = 0.6

                (x_square, y_square, w, h) = cv2.boundingRect(approx)
                area_min = -a_min_480*np.exp(-b_min_480*abs(self.local_pos.z))+c_min_480 -200
                area_max = -a_max_480*np.exp(-b_max_480*abs(self.local_pos.z))+c_max_480 + 300
                area = cv2.contourArea(approx)
                  
                if len(approx) <= 6 and len(approx) > 3 and area >= area_min and area <= area_max:
                    pxl_width = area**(1/2.0)
                    pxl_m_ratio = pxl_width/marker_width_max
                    self.circle_marker_sp.position.x = -(cv_image.shape[1]/2-(x_square+pxl_width/2))/pxl_m_ratio
                    self.circle_marker_sp.position.y = (cv_image.shape[0]/2-(y_square+pxl_width/2))/pxl_m_ratio
                    self.circle_marker_sp.position.z = self.local_pos.z
                    self.circle_marker_spotted = 1
                    self.last_time = time.time()
                    cv2.drawContours(cv_image, [approx], 0, (50,255,0), 2)                    
                elif time.time() - self.last_time >= 1:
                    self.circle_marker_spotted = 0 

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))       

        except CvBridgeError as e:
            print(e)

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

def publish_marker_setpoint(cnt,pub_rel_pos):
    pub_rel_pos.publish(cnt.marker)

def run(argv):
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = FlightModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity', TwistStamped, cnt.velCb)

    rospy.Subscriber('fiducial_transforms', FiducialTransformArray,cnt.arucoCb)

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, cnt.gpsCb)

    rospy.Subscriber("/honeybee/camera/image_raw",Image, cnt.imageCb,queue_size=1)

    # Setpoint publishers   
    sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Relative position to marker publisher
    marker_rel_pub = rospy.Publisher('mavros/marker_Info/marker_info_sub', MarkerTarget, queue_size=1)


    # activate OFFBOARD mode
    print("Activate OFFBOARD mode")
    while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k=0
        while k<10:
            cnt.updateSp(cnt.local_pos.y, cnt.local_pos.x, -cnt.local_pos.z, 0)
            publish_setpoint(cnt, sp_pos_pub)
            k = k + 1
        #modes.setOffboardMode()
        rate.sleep()
    print("OFFBOARD mode activated\n")

    # ROS main loop - 
    mission_flag = 1
    state_machine_flag = 0 #state machine variable 0-Drone waits before starting mission
    last_time = time.time()
    land_count = 1
    first_time = time.time()
    
    while mission_flag and not rospy.is_shutdown():
        current_time = time.time()

        f = open("followsquare.txt","a+") 
        f.write("%f,%f,%f,%f,\n" % ((5.5-cnt.circle_marker_sp.position.y) , (cnt.local_pos.y),(time.time()-first_time),(cnt.local_pos.z)))
        f.close()

        if state_machine_flag == 0: #In this state the drone is commanded to fly to the unaccurate location of the marker
            cnt.updateMarkerSp(0,0,0,0)
            publish_marker_setpoint(cnt,marker_rel_pub)
            cnt.updateSp(y,x,-z,yaw) #pos_sp now gets the unaccurate location of the marker
            publish_setpoint(cnt,sp_pos_pub) #publish the unaccurate location of the marker to the drone
            rate.sleep()
            if abs(cnt.local_pos.x - x) < threshold+0.5 and abs(cnt.local_vel.x) < threshold+0.5 and abs(cnt.local_pos.y - y) < threshold+0.5 and abs(cnt.local_vel.y) < threshold+0.5 and abs(cnt.local_pos.z - z) < threshold+0.5 and abs(cnt.local_vel.z) < threshold+0.5:
                state_machine_flag = 1
                #state_machine_flag = 0
                print("1 - The Drone Has now reached the marker GPS offset\n")
                print("1 - The Drone will now commence to accurately track the circle marker\n")
                

        if state_machine_flag == 1:
            if  cnt.circle_marker_spotted == 1: 
                cnt.updateMarkerSp(cnt.circle_marker_sp.position.x,cnt.circle_marker_sp.position.y,cnt.circle_marker_sp.position.z,1) # Here the position of the marker is being published to the FCU. Marker Spotted is set to 1
                publish_marker_setpoint(cnt,marker_rel_pub) # publish marker positions
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-8,yaw) #Here the UAV is commanded to fly altitude of 8m
                publish_setpoint(cnt,sp_pos_pub) #publish yaw of 0
                rate.sleep()
                last_time = time.time()
                if abs(cnt.local_vel.x) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_vel.z) < threshold and abs(cnt.circle_marker_sp.position.x) < threshold and abs(cnt.circle_marker_sp.position.y) < threshold:
                    state_machine_flag =  3
                    print("1 - The drone is stable above the square marker")
                    print("1 - The altitude will now be lowerd \n")
            else:
                print("Marker Lost\n\n")
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-8,yaw) #Let the drone hover on the spot untill the marker is seen
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()
                if time.time()-last_time >= timeout_time:
                    state_machine_flag = 2
                    print("2 - The drone has lost the marker and will now switch back to using GPS\n\n")
                else:
                    state_machine_flag == 1
        
        if state_machine_flag == 2: # in this state the marker has been lost and the drone is comanded back to its gps location without the use of marker data
            #cnt.circle_marker_error_flag = 0 #Here we set the error flag to 0 so that the circle marker data can be acquired normally sorta pointless here
            cnt.updateMarkerSp(0,0,0,0)
            publish_marker_setpoint(cnt,marker_rel_pub)
            cnt.updateSp(y,x,-z,yaw) 
            publish_setpoint(cnt,sp_pos_pub) 
            rate.sleep()
            if abs(cnt.local_pos.x - x) < threshold + 0.5 and abs(cnt.local_vel.x) < threshold +0.5 and abs(cnt.local_pos.y - y) < threshold +0.5 and abs(cnt.local_vel.y) < threshold + 0.5 and abs(cnt.local_pos.z - z) < threshold + 0.5 and abs(cnt.local_vel.z) < threshold + 0.5 :
                state_machine_flag = 1
                cnt.land_flag = 1
                print("1 - Lets Retry \n\n")
        
        if state_machine_flag == 3: #In this state the drone tracks the square marker but lowers altitude so that the aruco marker can come into view once the height threshold has been reached then go to state machine 4
            if cnt.circle_marker_spotted == 1:
                cnt.updateMarkerSp(cnt.circle_marker_sp.position.x,cnt.circle_marker_sp.position.y,cnt.circle_marker_sp.position.z,1)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-marker_hover_z,yaw) #UAV is commanded to altitude 1.5 m 
                publish_setpoint(cnt,sp_pos_pub) 
                rate.sleep()
                last_time = time.time()
                if  abs(cnt.local_vel.z) < vision_threshold and abs(cnt.circle_marker_sp.position.z-marker_hover_z) < threshold:
                    state_machine_flag = 4
                    print("4 - The UAV will now switch over to the aruco marker\n")
            else:
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-8,yaw) #Let the drone hover on the spot untill the marker is seen
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()
                if time.time()-last_time >= timeout_time:
                    state_machine_flag = 2
                else:
                    state_machine_flag = 3
            #modes.setLandMode()
        if state_machine_flag == 4: #in this state the drone is tasked to localize with the aruco marker
            if cnt.marker_id == 1:
                cnt.updateMarkerSp(cnt.marker_sp.position.x,cnt.marker_sp.position.y,cnt.marker_sp.position.z,1)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-marker_hover_z,yaw) 
                publish_setpoint(cnt,sp_pos_pub) 
                rate.sleep()
                last_time = time.time()
                if abs(cnt.local_vel.x) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_vel.z) < threshold and abs(cnt.marker_sp.position.x) < threshold and abs(cnt.marker_sp.position.y) < threshold:
                    state_machine_flag = 5
            else:
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-8,yaw) #Let the drone hover on the spot untill the marker is seen
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()
                if time.time()-last_time >= timeout_time:
                    state_machine_flag = 2
                else:
                    state_machine_flag = 4

        if state_machine_flag == 5:
            if cnt.marker_id == 1:
                cnt.updateMarkerSp(cnt.marker_sp.position.x,cnt.marker_sp.position.y,cnt.marker_sp.position.z,1)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-0.6,yaw) 
                publish_setpoint(cnt,sp_pos_pub) 
                rate.sleep()
                last_time = time.time()
                if abs(cnt.local_vel.x) < vision_threshold and abs(cnt.local_vel.y) < vision_threshold and abs(cnt.local_vel.z) < vision_threshold and abs(cnt.marker_sp.position.x) < vision_threshold and abs(cnt.marker_sp.position.y) < vision_threshold and abs(cnt.marker_sp.position.z)<0.7:
                    state_machine_flag = 6
            else:
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-cnt.local_pos.z,yaw) #Let the drone hover on the spot untill the marker is seen
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()
                if time.time()-last_time >= timeout_time:
                    state_machine_flag = 2
                else:
                    state_machine_flag = 5
        
        if state_machine_flag == 6:
            if cnt.marker_id == 1:
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                last_time = time.time()
                modes.setLandMode()
            else:
                if time.time()-last_time >= timeout_time:
                    while not (cnt.state.mode == "AUTO.TAKEOFF" and cnt.state.armed or rospy.is_shutdown()):
                        while not (cnt.state.armed or rospy.is_shutdown()):
                            modes.setArm()
                        while not (cnt.state.mode == "AUTO.TAKEOFF" or rospy.is_shutdown()):
                            modes.setTakeoffMode()
                    while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
                        k=0
                        while k<10:
                            cnt.updateSp(cnt.local_pos.y, cnt.local_pos.x, -cnt.local_pos.z, 0)
                            publish_setpoint(cnt, sp_pos_pub)
                            rate.sleep()
                            k = k + 1
                        modes.setOffboardMode()
                        rate.sleep()
                    state_machine_flag = 2

                elif -cnt.local_pos.z <=0.1 and cnt.land_flag == 1:
                    #print("\nLocal y: %f\n" % cnt.local_pos.y-5.5)
                    #print("Local x: %f\n" % cnt.local_pos.x-5.5)
                    f = open("CircleMarkerLanding.txt","a+") 
                    f.write("%f,%f,\n" % ((cnt.marker_sp.position.y) , (cnt.marker_sp.position.x)))
                    f.close()
                    print("writing to file complete\n")
                    print("Landing %d completed\n\n" % land_count)
                    print("Landed at %f-Y and %f-X"%((cnt.marker_sp.position.y) , (cnt.marker_sp.position.x)))

                    land_count = land_count+1 
                    cnt.land_flag = 0
                else:
                    state_machine_flag = 6
                
                           
def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
