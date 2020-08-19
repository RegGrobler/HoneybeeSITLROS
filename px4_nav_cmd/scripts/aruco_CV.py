#!/usr/bin/env python
# ROS python API
import rospy
import roslib
import cv2

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

# import other system/utils
import time, sys, math

from fiducial_msgs.msg import FiducialTransformArray

# Global variables

threshold = 0.8
final_threshold = 0.3 # How small the position error and velocity should be before sending the next waypoint
waypoint_time = 8 # If < 0, will send next waypoint when current one is reached. If >= 0, will send next waypoint after the amount of time has passed

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
            print "Service arming call failed: %s"%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Land Mode could not be set."%e

    def setTakeoffMode(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 7, latitude = -34.046623, longitude = 18.740375)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

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

        self.marker_global_sp = GlobalPositionTarget()
        self.marker_global_sp.type_mask = int('10011111100',2)
        self.marker_global_sp.coordinate_frame = 5
        self.marker_global_sp.latitude = 0
        self.marker_global_sp.longitude = 0


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

        #print("")

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

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

    # Setpoint publishers   
    sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Relative position to marker publisher
    #marker_rel_pub = rospy.Publisher('mavros/marker_Info/marker_info_sub',mavros_msgs/MarkerTarget,queue_size=1)

    # Arm the drone
    # print("Arming")
    # while not (cnt.state.armed or rospy.is_shutdown()):
    #     modes.setArm()
    #     rate.sleep()
    # print("Armed\n")

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
    mission_flag = 1 #flag that starts the main loop of the mission
    state_machine_flag = 1 #state machine variable 0-Drone waits before starting mission
    last_time = time.time() 
    y = 8  #the y position of the gps location sent to the drone
    x = 7   #the x position of the gps location sent tp the drone
    z = 9   #the z position of the aruco marker
    y_box = 0
    x_box = 0
    box_index = 0 
    land_flag = 1
    landing_counter = 1
    yaw = 0 #commanded yaw
    threshold = 0.8
    while mission_flag and not rospy.is_shutdown():
        current_time = time.time()
        #print("marker id is - " + str(cnt.marker_id))
        if state_machine_flag == 0:
            cnt.updateSp(cnt.local_pos.y, cnt.local_pos.x, -cnt.local_pos.z, 0) #The drone is now commanded to stay in its current position
            publish_setpoint(cnt,sp_pos_pub) #publish new setpoints
            rate.sleep()
        #print("waiting 10 seconds")
        if current_time - last_time >= waypoint_time and state_machine_flag == 0: #waits untill waypoint_time seconds have passed before starting the next state
            state_machine_flag = 1
            print("Flying to aruco marker")
        
        if state_machine_flag == 1: #In this state the drone has to fly to the marker and wait until threshhold has been reached
            cnt.updateSp(y,x,-z,yaw) #pos_sp now gets the unaccurate location of the marker
            publish_setpoint(cnt,sp_pos_pub) #publish the unaccurate location of the marker to the drone
            rate.sleep()
            if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_vel.x) < threshold and abs(cnt.local_pos.y - y) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_pos.z - z) < threshold and abs(cnt.local_vel.z) < threshold:
                state_machine_flag = 2
                print("Aruco Marker has been reached")
                print("The drone is now searching for the marker")

        if state_machine_flag == 2:

            cnt.updateSp(y,x,-z,yaw) #The drone is now commanded to stay in its current position
            publish_setpoint(cnt,sp_pos_pub) #publish new setpoints
            rate.sleep()

            if cnt.marker_id == 1:
                state_machine_flag = 3
                last_time = time.time()
                print("The marker with ID-%d has been found" % (cnt.marker_id))
                print("The drone will now try to hover over the marker at an altitude of 4m")
                z = 4
                
            else:
                state_machine_flag = -3
                print("Cannot locate marker")
                print("The drone will lower altitude and look around")
                y_box = cnt.local_pos.y
                x_box = cnt.local_pos.x
                y = cnt.local_pos.y
                x = cnt.local_pos.x
            

        if state_machine_flag == 3:
            y = cnt.local_pos.y + cnt.marker_sp.position.y
            x = cnt.local_pos.x + cnt.marker_sp.position.x
            cnt.updateSp(y,x,-z,yaw)
            publish_setpoint(cnt,sp_pos_pub) #publish new setpoints
            rate.sleep()
            #print("x - %f" %x)
            #print("y - %f" %y)
            if abs(x-5.5) < threshold and abs(cnt.local_vel.x) < threshold and abs(y-5.5) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_pos.z - z) < threshold and abs(cnt.local_vel.z) < threshold:
                z = 1.5
                print("The drone is now hovering above the target")
                print("Altitude will now be lowered to 1.5m")
                state_machine_flag = 4


        if state_machine_flag == -3:
            if cnt.marker_id == 1:
                print("I have entered the found marker loop")
                y = cnt.local_pos.y
                x = cnt.local_pos.x
                z = 9
                if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_vel.x) < threshold and abs(cnt.local_pos.y - y) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_pos.z - z) < threshold and abs(cnt.local_vel.z) < threshold:
                    print("The marker with ID-%d has been found" % (cnt.marker_id))
                    print("The drone will now try to hover over the marker at an altitude of 4m")
                    y = cnt.local_pos.y + cnt.marker_sp.position.y
                    x = cnt.local_pos.x + cnt.marker_sp.position.x
                    z = 4
                    box_index = 0
                    state_machine_flag = 3
            else:
                if box_index == 0:
                    print("starting box fly mode to locate the marker")
                    box_index = 1
                if box_index == 1:
                    y = y_box + 5
                    x = x_box
                    if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_pos.y - y) < threshold:
                        box_index = 2
                if box_index == 2:
                    x = x_box + 5
                    if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_pos.y - y) < threshold:
                        box_index = 3
                if box_index == 3:
                    y = y_box - 5
                    if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_pos.y - y) < threshold:
                        box_index = 4
                if box_index == 4:
                    x = x_box - 5
                    if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_pos.y - y) < threshold:
                        box_index = 5
                if box_index == 5:
                    y = y_box + 5
                    if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_pos.y - y) < threshold:
                        box_index = 6
                if box_index == 6:
                    x = x_box
                    if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_pos.y - y) < threshold:
                        box_index = 7
                if box_index == 7:
                    print("No box was found")
                    box_index = 0
                    modes.setLandMode()

            cnt.updateSp(y,x,-z,yaw) #The drone is now commanded to stay in its current position
            publish_setpoint(cnt,sp_pos_pub) #publish new setpoints
            rate.sleep()


        if state_machine_flag == 4:
            z = 1.2
            y = cnt.local_pos.y + cnt.marker_sp.position.y
            x = cnt.local_pos.x + cnt.marker_sp.position.x
            cnt.updateSp(y,x,-z,yaw)
            publish_setpoint(cnt,sp_pos_pub) #publish new setpoints
            rate.sleep()
            if abs(x-5.5) < final_threshold and abs(cnt.local_vel.x) < final_threshold and abs(y-5.5) < final_threshold and abs(cnt.local_vel.y) < final_threshold and abs(cnt.local_pos.z) < 1.3:
                print("Landing\n\n")
                state_machine_flag = 5
                last_time = time.time()


        if state_machine_flag == 5:
            if cnt.marker_id == 1:
                modes.setLandMode()
            if time.time() - last_time >= 8 and cnt.marker_id == 0:
                if land_flag == 1:
                    print("X - %f" % (cnt.local_pos.x -5.5))
                    print("Y - %f" % (cnt.local_pos.y -5.5))
                    f = open("markerOnly_GPS_assisted.txt","a+") 
                    f.write("%f,%f,\n" % ((cnt.marker_sp.position.y) , (cnt.marker_sp.position.x)))
                    f.close()
                    print("Landing number %d is complete" % landing_counter)
                    land_flag = 0 
                    landing_counter = landing_counter + 1
                while not (cnt.state.mode == "AUTO.TAKEOFF" and cnt.state.armed or rospy.is_shutdown()):
                    while not (cnt.state.armed or rospy.is_shutdown()):
                        modes.setArm()
                    while not (cnt.state.mode == "AUTO.TAKEOFF" or rospy.is_shutdown()):
                        modes.setTakeoffMode()
                while (cnt.state.mode != "OFFBOARD" and time.time()-last_time >= 8):
                    k=0
                    while k<10:
                        cnt.updateSp(cnt.local_pos.y, cnt.local_pos.x, -cnt.local_pos.z, 0)
                        publish_setpoint(cnt, sp_pos_pub)
                        rate.sleep()
                        k = k + 1
                    modes.setOffboardMode()
                    rate.sleep()
                    x = 3
                    y = 8  
                    z = 9   
                    state_machine_flag = 1
                    land_flag = 1
        


        img = cv2.imread("shapes1.png", cv2.IMREAD_GRAYSCALE)
        imgc =  cv2.imread("shapes1.png")
        _, threshold = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
    
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            area = cv2.contourArea(approx)
    

            if len(approx) <= 15 and len(approx) > 3 and area >= 400 and area <= 2000:
                print("Area is %f" % area)
                print("Number of contours %f" % len(approx))
                cv2.drawContours(imgc, [approx], 0, (50,255,0), 2)


        cv2.imshow("shapes", imgc)
#cv2.imshow("Threshold", threshold)


        cv2.waitKey(0)
        cv2.destroyAllWindows()


            


        
        

    # wait before landing
    print("Waiting for user to terminate (press Ctrl-C)...")
    init_time = time.time()
    current_time = time.time()
    while not rospy.is_shutdown():
        cnt.updateSp(waypoints[current_wp - 1][0], waypoints[current_wp - 1][1], -waypoints[current_wp - 1][2], -waypoints[current_wp - 1][3])
        publish_setpoint(cnt, sp_pos_pub)
        rate.sleep()

        current_time = time.time()
    print("Done\n")

def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
