#!/usr/bin/env python
# ROS python API
import rospy

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import other system/utils
import time, sys, math

from fiducial_msgs.msg import FiducialTransformArray

# Global variables

threshold = 0.05
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
            takeoffService(altitude = 1, latitude = -34.0466177, longitude = 18.7403701)
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
    last_time = 0
    last_time = time.time()
    print("State machine 1\n\n")







    while mission_flag and not rospy.is_shutdown():

        if state_machine_flag == 1:
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
            state_machine_flag = 2
            print("State machine 2\n\n")

        if state_machine_flag == 2:
            if abs(cnt.local_vel.x) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_vel.z) < threshold and abs(cnt.local_pos.z > 0.95):
                state_machine_flag = 3
                last_time = time.time()
                print("Landing\n\n")

        if state_machine_flag == 3:
            modes.setLandMode() 
            if time.time()-last_time >= 10:
                while not (cnt.state.armed or rospy.is_shutdown()):
                    modes.setArm()
                while not (cnt.state.mode == "AUTO.TAKEOFF" or rospy.is_shutdown()):
                    modes.setTakeoffMode()
                while (cnt.state.mode != "OFFBOARD" and time.time()-last_time >= 13):
                    k=0
                    while k<10:
                        cnt.updateSp(cnt.local_pos.y, cnt.local_pos.x, -cnt.local_pos.z, 0)
                        publish_setpoint(cnt, sp_pos_pub)
                        rate.sleep()
                        k = k + 1
                    modes.setOffboardMode()
                    rate.sleep()
                state_machine_flag = 2
                print("state machine 2 again")
                last_time = time.time()
        

        
        


            


        
        


def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
