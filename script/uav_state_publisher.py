import sys
import rospy
import roslib
import tf
import math
import time
import std_msgs.msg
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QTextBrowser
from python_qt_binding.QtWidgets import QLineEdit
import numpy as np 


from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode

from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class UAVStatePublisher(QMainWindow):

    def __init__(self):
        super().__init__()

        #Not working: how to set window dimension
        self.height =300
        self.width =300
        self.top =50
        self.left =50       
        font = QFont("Helvetica", 9, QFont.Bold)

        rospy.init_node("uav_state_publisher")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        self.arm_disarm_req = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_req = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_req = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.offboard_req = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.odomCb)
        rospy.Subscriber("/mavros/state", State, self.stateCb)
        self.state = "Idle"
        self.arm_state = False
        self.local_cmd = ([0.0, 0.0, 0.0])
        self.yaw_cmd = 0.0

        self.setpoint_raw_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.raw_msg = PositionTarget()

        #MASK_POSITION = 0b0000011111111000, // x, y, z, vyaw
        #MASK_VELOCITY = 0b0000011111000111, // vx, vy, vz, vyaw
        self.raw_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        #self.raw_msg.type_mask = 0b0000011111111000
        '''
        uint16 IGNORE_PX=1
        uint16 IGNORE_PY=2
        uint16 IGNORE_PZ=4
        uint16 IGNORE_VX=8
        uint16 IGNORE_VY=16
        uint16 IGNORE_VZ=32
        uint16 IGNORE_AFX=64
        uint16 IGNORE_AFY=128
        uint16 IGNORE_AFZ=256
        uint16 FORCE=512
        uint16 IGNORE_YAW=1024
        uint16 IGNORE_YAW_RATE=2048
        '''
        self.raw_msg.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY +  PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW + PositionTarget.FORCE   
        

        window = QWidget()
        self.layout = QVBoxLayout()

        self.setWindowTitle("UAV State Publisher")
        
        #label = QLabel("Yaw")
        #label.setFont(font)       
        #layout.setGeometry(50,50,320,200)
        #self.layout.addWidget(label)
        #layout.addWidget(QPushButton('Bottom'))                
        #mySlider = QSlider(Qt.Horizontal, self)
        #mySlider.setGeometry(30, 40, 200, 30)
        #mySlider.valueChanged[int].connect(self.changeValue)
        #mySlider.setRange(-180, 180)
        #mySlider.setValue(0)
        #self.layout.addWidget(mySlider)

        #Position label
        self.textbox_state = QLabel(self)
        self.textbox_state.move(20, 20)
        self.textbox_state.resize(280,40)
        self.textbox_state.setFont(font)  


        #Position label
        self.textbox_pos = QLabel(self)
        self.textbox_pos.move(20, 20)
        self.textbox_pos.resize(280,40)

        #Yaw Label        
        self.textbox_yaw = QLabel(self)
        self.textbox_yaw.move(20, 20)
        self.textbox_yaw.resize(280,40)

        #Command position label
        self.textbox_cmd_pos = QLabel(self)
        self.textbox_cmd_pos.move(20, 20)
        self.textbox_cmd_pos.resize(280,40)

        #Command yaw label
        self.textbox_cmd_yaw = QLabel(self)
        self.textbox_cmd_yaw.move(20, 20)
        self.textbox_cmd_yaw.resize(280,40)


        self.arm_button = QPushButton('Arm / Disarm', self)
        self.arm_button.setToolTip('Start/Stop motor spinning')
        self.arm_button.move(100,70)
        self.arm_button.clicked.connect(self.on_click_arm)
        
        self.toff_button = QPushButton('Take-off', self)
        self.toff_button.setToolTip('takeoff the UAV')
        self.toff_button.move(100,70)
        self.toff_button.clicked.connect(self.on_click_toff)

        self.land_button = QPushButton('Land', self)
        self.land_button.setToolTip('Land the UAV')
        self.land_button.move(100,70)
        self.land_button.clicked.connect(self.on_click_land)
        
        self.offboard_button = QPushButton('OFFBOARD', self)
        self.offboard_button.setToolTip('Start offboard mode')
        self.offboard_button.move(100,70)
        self.offboard_button.clicked.connect(self.on_click_offboard)


        yaw_cmdlabel = QLabel("Slide to command the yaw rate (d/s)")
        yaw_cmdlabel.setFont(font)
        #layout.setGeometry(50,50,320,200)

        mySlider = QSlider(Qt.Horizontal, self)
        mySlider.setGeometry(30, 40, 200, 30)
        mySlider.valueChanged[int].connect(self.changeValue)
        mySlider.setRange(-60, 60.0)
        mySlider.setValue(0)
        #self.layout.addWidget(mySlider)

        
        self.layout.addWidget(self.textbox_state)
        self.layout.addWidget(self.textbox_pos)
        self.layout.addWidget(self.textbox_yaw)
        self.layout.addWidget(self.textbox_cmd_pos)
        self.layout.addWidget(self.textbox_cmd_yaw)
        self.layout.addWidget(self.arm_button)
        self.layout.addWidget(self.toff_button)
        self.layout.addWidget(self.land_button)
        self.layout.addWidget(self.offboard_button)
        self.layout.addWidget(yaw_cmdlabel)
        self.layout.addWidget(mySlider)
        

        window.setLayout(self.layout)
        window.show()
        app.exec_()

        sys.exit()
        '''
        self.setWindowTitle("UAV State Publisher")
        font = QFont("Helvetica", 9, QFont.Bold)
        label = QLabel("Yaw")
        label.setFont(font)
        addWidget(label)

        mySlider = QSlider(Qt.Horizontal, self)
        mySlider.setGeometry(30, 40, 200, 30)
        mySlider.valueChanged[int].connect(self.changeValue)

        #self.setGeometry(50,50,320,200)
        self.show()
        '''

    def on_click_arm(self):
        if self.arm_state == True:
            self.arm_disarm_req( False )
        else:
            self.arm_disarm_req( True )


    def on_click_toff(self):
        print("Request Takeoff") #Todo
        self.local_cmd[2] = self.local_cmd[2]+2.0            
       
    def on_click_land(self):
        print("Request Land") #Todo
        self.land_req(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)


    def on_click_offboard(self):            
        self.offboard_req( base_mode = 0, custom_mode = "OFFBOARD" )

    def changeValue(self, value):
        self.yaw_cmd = ( value*3.1415)/180.0
        print(self.yaw_cmd)

    def stateCb(self,msg): 
        self.state = msg.mode
        self.arm_state = msg.armed
        
    def odomCb(self, msg):
        
        self.textbox_state.setText( "State: " + self.state )

        str_x = str( round( msg.pose.pose.position.x, 2))
        str_y = str( round( msg.pose.pose.position.y, 2))
        str_z = str( round( msg.pose.pose.position.z, 2))
        coords = str_x + " " + str_y + " " + str_z       
        self.textbox_pos.setText( "Pos: " + coords )
        
        quaternion = ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) 
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.textbox_yaw.setText( "Yaw: " + str( round( yaw, 2) ) )
        self.raw_msg.header.stamp = rospy.get_rostime()


        if( self.state == "OFFBOARD"):
            x = 1
            
            #self.local_cmd[0] = 1
            #self.local_cmd[1] = 0
            #self.local_cmd[2] = 3
        else:
            self.yaw_cmd = 0.0
            self.local_cmd[0] = msg.pose.pose.position.x
            self.local_cmd[1] = msg.pose.pose.position.y
            self.local_cmd[2] = msg.pose.pose.position.z
    
        self.raw_msg.position.x = self.local_cmd[0]
        self.raw_msg.position.y = self.local_cmd[1]
        self.raw_msg.position.z = self.local_cmd[2]
        self.raw_msg.yaw_rate = self.yaw_cmd

        self.textbox_cmd_pos.setText( "Cmd Pos: " + str(round(self.local_cmd[0], 2)) + " " + str( round(self.local_cmd[1], 2)) + " " + str( round(self.local_cmd[2], 2)) )
        self.textbox_cmd_yaw.setText( "Cmd Yaw: " + str(round(self.yaw_cmd, 2)))
        self.setpoint_raw_pub.publish( self.raw_msg )


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = UAVStatePublisher()
    sys.exit(app.exec_())

    bg3xevpjZq