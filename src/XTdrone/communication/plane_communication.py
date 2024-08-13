import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from std_msgs.msg import String
import sys
from gazebo_msgs.msg import ModelStates
import math

rospy.init_node('plane_'+sys.argv[1]+"_communication")
rate = rospy.Rate(30)

class Communication:

    def __init__(self, vehicle_id):
        
        self.vehicle_type = 'plane'
        self.vehicle_id = vehicle_id
        self.local_pose = None
        self.target_motion = PositionTarget()
        self.coordinate_frame = 1
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        # ADD
        self.sorted_model_states = None
        self.now_position = Pose()
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback,queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,queue_size=1)     
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback,queue_size=3)
        # add 
        self.gazebo_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.position_callbak, queue_size=1)

        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        # add
        self.motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)

        print("communication initialized")

    def position_callbak(self,msg):
        model_states = list(zip(msg.name, msg.pose, msg.twist))[1:]
        self.sorted_model_states = sorted(model_states, key=lambda x: eval(x[0].split("_")[1]))
        self.now_position.position = self.sorted_model_states[0][1].position

    def start(self):
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            target_position = Pose()
            target_position.position.x = 1000
            target_position.position.y = 1000
            target_position.position.z = 50

            self.target_motion = self.construct_target(x=target_position.position.x,y=target_position.position.y,z=target_position.position.z)
            self.coordinate_frame = 1
            
            self.target_motion.type_mask = (
                                 PositionTarget.IGNORE_AFX |
                                 PositionTarget.IGNORE_AFY |
                                 PositionTarget.IGNORE_AFZ |
                                 PositionTarget.FORCE |
                                 PositionTarget.IGNORE_YAW |
                                 PositionTarget.IGNORE_YAW_RATE)
            print(self.target_motion.type_mask)
            velocity = 20
            self.target_motion.velocity.x = velocity*(target_position.position.x - self.now_position.position.x)/math.sqrt((target_position.position.x - self.now_position.position.x)**2 + (target_position.position.y - self.now_position.position.y)**2 )
            self.target_motion.velocity.y = velocity*(target_position.position.y - self.now_position.position.y)/math.sqrt((target_position.position.x - self.now_position.position.x)**2 + (target_position.position.y - self.now_position.position.y)**2 )
            self.target_motion.velocity.z = 0.0
            self.target_motion_pub.publish(self.target_motion)

            # twist = TwistStamped()
            # twist.twist.linear.x = 15
            # twist.twist.linear.y = 15
            # twist.twist.linear.z = 0
            # self.motion_pub.publish(twist)
            try:
                rate.sleep()
            except:
                continue

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def construct_target(self, x=0, y=0, z=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame 

        if self.coordinate_frame == 1:
            target_raw_pose.position.x = x
            target_raw_pose.position.y = y
            target_raw_pose.position.z = z
        else:
            target_raw_pose.position.x = -y
            target_raw_pose.position.y = x
            target_raw_pose.position.z = z

        if self.mission == 'takeoff':
            target_raw_pose.type_mask = 4096
        elif self.mission == 'land':
            target_raw_pose.type_mask = 8192
        elif self.mission == 'loiter':
            target_raw_pose.type_mask = 12288
        else:
            target_raw_pose.type_mask = 16384

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        # FRAME_BODY_OFFSET_NED=9
        self.coordinate_frame = 9
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)
 
    def cmd_pose_enu_callback(self, msg):
        # uint8 FRAME_LOCAL_NED=1
        self.coordinate_frame = 1
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)

    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data in ['takeoff', 'land', 'loiter', 'idle']:
            self.mission = msg.data   
            print(self.mission)
        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

    def arm(self):
        if self.armService(True):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": disarming failed!")
            return False
    def flight_mode_switch(self):
        if self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1])
    communication.start()

