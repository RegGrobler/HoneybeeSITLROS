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
y = 7  #the y position of the gps location sent to the drone
x = 6 #the x position of the gps location sent tp the drone
z = 6   #the z position of the aruco marker
yaw = 0 #commanded yaw
timeout_time = 5
marker_hover_z = 2
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
        self.xy_marker_P_Gain = 1		#Proportional gain used to increase the xy marker position published to the fcu.
        
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("SquareMarker",Image,queue_size=1)

        self.timer_flag = 0
        self.last_time = 0
        self.start_time = time.time()
        self.imagerate = rospy.Rate(20.0)

        self.a_min_480 = -2.45665871e+04
        self.a_max_480 = -6.24018837e+04
        self.b_min_480 = 7.84733531e-01 
        self.b_max_480 =  7.95711615e-01
        self.c_min_480 = 3.26416165e+02 
        self.c_max_480 = 9.07711110e+02
        
        #self.a_min_480 = -1.87406485e+05		#These paramters represent actual measureremnts
        #self.b_min_480 = 1.06508327e+00
        #self.c_min_480 = 2.09015433e+03
        #self.a_max_480 = -4.52270733e+05
        #self.b_max_480 = 1.06844146e+00
        #self.c_max_480 = 4.90956739e+03

        self.marker_width_max = 0.6
        #self.marker_width_max = 0.39
        self.commanded_altitude = 7


	    #Instantiate the markerTarget message
        self.marker = MarkerTarget()
        self.marker.marker_position.x = 0.0
        self.marker.marker_position.y = 0.0
        self.marker.marker_position.z = 0.0
        self.marker.marker_spotted = 0.0
        self.aruco_last_spotted_time = 0.0
    

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
            self.aruco_last_spotted_time = time.time()
        except:
            if time.time() - self.aruco_last_spotted_time >= 2:
                self.marker_id = 0
            else:
                pass
    
    
    def gpsCb(self,msg):
        self.circle_latitude = msg.latitude
        self.circle_longitude = msg.longitude


    def imageCb(self,msg):
        try:
            cv_image_raw = self.bridge.imgmsg_to_cv2(msg,"bgr8") 		#convert image to OpenCV format
            self.imagerate.sleep()
            cv_image = cv2.resize(cv_image_raw,(640,480)) 	 		#resize image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)			#Convert to greyscale
            _, threshold = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)	#threshold image to remove black and white
            _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	#contour the image
            height = self.local_pos.z
            #height = self.marker_sp.position.z
            area_min = -self.a_min_480*np.exp(-self.b_min_480*abs(height))+self.c_min_480 +300 #-400  #calculate the minimum area +300 was used for simulated test
            area_max = -self.a_max_480*np.exp(-self.b_max_480*abs(height))+self.c_max_480 + 1000 	#calculate the maximum area

            for cnt in contours:
                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)		#create points using approx
                area = cv2.contourArea(approx)							#calculate area of points

                if len(approx) <= 5 and len(approx) > 3 and area >= area_min and area <= area_max:
                    (x_square, y_square, w, h) = cv2.boundingRect(approx) 	#Get imformation about square rectangle
                    pxl_width = area**(1/2.0)							    #Get the width of the box in pixels
                    pxl_m_ratio = pxl_width/self.marker_width_max			#Get ratio of pixels to meters
		    
                    if pxl_m_ratio == 0:							
                        pass
                    else:
                        self.circle_marker_sp.position.x = -((cv_image.shape[1]/2-(x_square+pxl_width/2))/pxl_m_ratio) 	#only calculate the distance if pxl_m_ratio is not 0
                        self.circle_marker_sp.position.y = ((cv_image.shape[0]/2-(y_square+pxl_width/2))/pxl_m_ratio)	
                        self.circle_marker_sp.position.z = self.local_pos.z						#Z_position of marker is set to the current height of the uav using estimated values
                        current_time = time.time() - self.start_time 												#This should make no difference as it was disabled in the PX4 software
                        #f = open("/home/honeybee/autolanding_1.txt","a+") 
                        #f.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,\n" % (self.circle_marker_sp.position.x,self.circle_marker_sp.position.y,self.marker_sp.position.z,self.local_pos.z,current_time,self.local_pos.x,self.local_pos.y,self.marker_sp.position.x,self.marker_sp.position.y))
                        #f.close()
                            
                        self.circle_marker_spotted = 1									#Marker has been spotted
                        self.last_time = time.time()									#last time marker was spotted it updated

                        cv2.drawContours(cv_image, [approx], 0, (50,255,0), 2)						#Draws the marker //Can be removed
                    
                if time.time() - self.last_time >= 5:  #Has the marker stopped being identified for more than 5 seconds 
                    self.circle_marker_spotted = 0
                if abs(self.local_pos.z) >= 10:        #If the UAV flies too high for some reason then enter bailout state
		            self.circle_marker_spotted = -1

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))       #publish drawn square

        except CvBridgeError as e:
            print(e)

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

def publish_marker_setpoint(cnt,pub_rel_pos):
    pub_rel_pos.publish(cnt.marker)

def run(argv):
    # initiate node
    rospy.init_node('autolander_CV', anonymous=True)

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

    rospy.Subscriber("/honeybee/camera/image_raw",Image, cnt.imageCb,queue_size=1) #Used for Simulation
    #rospy.Subscriber("/jetson_camera_node/image_raw",Image, cnt.imageCb,queue_size=1) #Used for practical tests

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
 	    cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-8,yaw) #Here the UAV is commanded to fly altitude of 8m
            publish_setpoint(cnt,sp_pos_pub)
            k = k + 1
	    rate.sleep()
        rate.sleep()
    print("OFFBOARD mode activated\n")

    # ROS main loop - 
    mission_flag = 1
    state_machine_flag = 1 #state machine variable
    last_time = time.time()
    
    
    while mission_flag and not rospy.is_shutdown():

        if state_machine_flag == 1:
            if  cnt.circle_marker_spotted == 1:
                last_time = time.time() 
                cnt.updateMarkerSp(cnt.circle_marker_sp.position.x,cnt.circle_marker_sp.position.y,-cnt.circle_marker_sp.position.z,1) # Here the position of the marker is being published to the FCU. Marker Spotted is set to 1
                publish_marker_setpoint(cnt,marker_rel_pub) # publish marker positions
                rate.sleep()
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-cnt.commanded_altitude,yaw) #Here the UAV is commanded to fly altitude of 7m
                publish_setpoint(cnt,sp_pos_pub) #publish yaw of 0
                rate.sleep()
                if abs(cnt.local_vel.x) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_vel.z) < threshold and abs(cnt.circle_marker_sp.position.x) < threshold and abs(cnt.circle_marker_sp.position.y) < threshold and abs(cnt.local_pos.z) > cnt.commanded_altitude-0.1:
                    state_machine_flag = 2
                    #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                    #f.write("State 2 \n")
                    #f.close()
                    print("state 2\n")
            if cnt.circle_marker_spotted == 0:
                #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                #f.write("Marker Lost - state 1\n")
                #f.close()

                print("marker lost in state 1\n")
                cnt.updateMarkerSp(0,0,0,0) #switch back to normal control
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-6,yaw) #Let the drone hover on the spot untill the marker is seen
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()
                if time.time()-last_time >= timeout_time:
		            print("Bail out\n")
                else:
                    state_machine_flag == 1
            if cnt.circle_marker_spotted == -1:
                #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                #f.write("UAV flew too high - state 1 \n")
                #f.close()
                cnt.updateMarkerSp(0,0,0,0) #switch back to normal control
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                cnt.updateSp(0,0,-3,yaw) #Fly down to an altitude of 4m
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()


        if state_machine_flag == 2:
            if  cnt.circle_marker_spotted == 1:
                cnt.updateMarkerSp(cnt.circle_marker_sp.position.x,cnt.circle_marker_sp.position.y,-cnt.marker_sp.position.z,1) # Here the position of the marker is being published to the FCU. Marker Spotted is set to 1
                publish_marker_setpoint(cnt,marker_rel_pub) # publish marker positions
                rate.sleep()
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-4.5,yaw) #Here the UAV is commanded to fly altitude of 6m
                publish_setpoint(cnt,sp_pos_pub) #publish yaw of 0
                rate.sleep()
                if  abs(cnt.marker_sp.position.x) < threshold+0.5 and abs(cnt.marker_sp.position.y) < threshold+0.5  and abs(cnt.local_pos.z)<4.7:
                    state_machine_flag = 3
                    #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                    #f.write("Switching to Aruco\n")
                    #f.close()
                    print("switching to aruco \n")

            if cnt.circle_marker_spotted == 0:
                #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                #f.write("Square marker lost - state 2 \n")
                #f.close()
                print("marker lost in state 2\n")
                cnt.updateMarkerSp(0,0,0,0) #switch back to normal control
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-6,yaw) #Let the drone hover on the spot untill the marker is seen
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()
                if time.time()-last_time >= timeout_time:
		            print("Bail out\n")
                else:
                    state_machine_flag == 1

            if cnt.circle_marker_spotted == -1:
                #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                #f.write("UAV flew too high - state 2 \n")
                #f.close()
                cnt.updateMarkerSp(0,0,0,0) #switch back to normal control
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                cnt.updateSp(0,0,-3,yaw) #Fly down to an altitude of 4m
                publish_setpoint(cnt,sp_pos_pub)
                rate.sleep()

        
        if state_machine_flag == 3:
            if cnt.marker_id == 0:
                #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                #f.write("Lost aruco marker - state 3 \n")
                #f.close()
                print("Aruco marker lost\n")
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                cnt.updateSp(cnt.local_pos.y,cnt.local_pos.x,-4,yaw) #Here the UAV is commanded to fly altitude of 6m
                publish_setpoint(cnt,sp_pos_pub) #publish yaw of 0
                rate.sleep()
            if cnt.marker_id == 1:   
                #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                #f.write("Found Aruco marker - state 3 \n")
                #f.close()
                cnt.updateMarkerSp(cnt.marker_sp.position.x,cnt.marker_sp.position.y,cnt.marker_sp.position.z,2) # Here the position of the marker is being published to the FCU. Marker Spotted is set to 1
                publish_marker_setpoint(cnt,marker_rel_pub) # publish marker positions
                rate.sleep()
                cnt.updateSp(0,0,-2.5,yaw) #Here the UAV is commanded to fly altitude of 6m
                publish_setpoint(cnt,sp_pos_pub) #publish yaw of 0
                rate.sleep()
                if  abs(cnt.marker_sp.position.x) < 0.01 and abs(cnt.marker_sp.position.y) < 0.01 and abs(cnt.local_pos.z)<2.7:
                    state_machine_flag = 4
                    #f = open("/home/honeybee/autolanding_1_log.txt","a+") 
                    #f.write("Landing\n")
                    #f.close()
                    print("landing")

        if state_machine_flag == 4:
                cnt.updateMarkerSp(0,0,0,0)
                publish_marker_setpoint(cnt,marker_rel_pub)
                rate.sleep()
                last_time = time.time()
                modes.setLandMode()
            

	if cnt.state.mode != "OFFBOARD": 
	    cnt.updateMarkerSp(0,0,0,0)
            publish_marker_setpoint(cnt,marker_rel_pub)  
            rate.sleep()

def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
