#!/usr/bin/env python

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

from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QCheckBox
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class UAVStatePublisher(QMainWindow):

    def __init__(self):
        super(UAVStatePublisher, self).__init__()

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

        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odomCb)
        rospy.Subscriber("/mavros/state", State, self.stateCb)
        self.state = "Idle"
        self.arm_state = False
        self.local_cmd = ([0.0, 0.0, 0.0])
        self.dx_cmd = 0.0
        self.dy_cmd = 0.0
        self.dz_cmd = 0.0
        self.yaw_cmd = 0.0
        
        self.ctrl_mode = "position"
        self.setpoint_raw_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)       
        
        
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
        self.pos_raw_msg = PositionTarget()
        self.pos_raw_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pos_raw_msg.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY +  PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW + PositionTarget.FORCE   
        

        self.vel_raw_msg = PositionTarget()
        self.vel_raw_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.vel_raw_msg.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY +  PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW + PositionTarget.FORCE   
        

        window = QWidget()
        self.layout = QVBoxLayout()

        self.setWindowTitle("UAV State Publisher")
        
     
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

        #Position control
        self.checkbox_pvcontrol = QCheckBox('Position/velocity control')
        #self.checkbox_vcontrol = QCheckBox('Velocity control')
        self.checkbox_pvcontrol.stateChanged.connect(self.select_chkbox_pvcontrol)
        #self.checkbox_vcontrol.stateChanged.connect(self.select_chkbox_vcontrol)

        
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


        

        x_cmdlabel = QLabel("Slide to command the x rate (d/s)")
        x_cmdlabel.setFont(font)
        
        self.cmd_dx_slider = QSlider(Qt.Horizontal , self)
        self.cmd_dx_slider.setGeometry(30, 40, 200, 30)
        self.cmd_dx_slider.valueChanged[int].connect(self.changeValue_dx)
        self.cmd_dx_slider.setRange(-10, 10)
        self.cmd_dx_slider.setValue(0)

        y_cmdlabel = QLabel("Slide to command the y rate (d/s)")
        y_cmdlabel.setFont(font)

        self.cmd_dy_slider = QSlider(Qt.Horizontal, self)
        self.cmd_dy_slider.setGeometry(30, 40, 200, 30)
        self.cmd_dy_slider.valueChanged[int].connect(self.changeValue_dy)
        self.cmd_dy_slider.setRange(-10, 10)
        self.cmd_dy_slider.setValue(0)

        z_cmdlabel = QLabel("Slide to command the z rate (d/s)")
        z_cmdlabel.setFont(font)

        self.cmd_dz_slider = QSlider(Qt.Horizontal, self)
        self.cmd_dz_slider.setGeometry(30, 40, 200, 30)
        self.cmd_dz_slider.valueChanged[int].connect(self.changeValue_dz)
        self.cmd_dz_slider.setRange(-10, 10)
        self.cmd_dz_slider.setValue(0)

        self.center_slider_button = QPushButton('Center', self)
        self.center_slider_button.setToolTip('Put in the middel all sliders')
        self.center_slider_button.move(100,70)
        self.center_slider_button.clicked.connect(self.on_click_center)


        yaw_cmdlabel = QLabel("Slide to command the yaw rate (d/s)")
        yaw_cmdlabel.setFont(font)
        

        self.cmd_dyaw_slider = QSlider(Qt.Horizontal, self)
        self.cmd_dyaw_slider.setGeometry(30, 40, 200, 30)
        self.cmd_dyaw_slider.valueChanged[int].connect(self.changeValue_dyaw)
        self.cmd_dyaw_slider.setRange(-60, 60)
        self.cmd_dyaw_slider.setValue(0)

        joy_cmdlabel = QLabel("Click and release buttons to command the UAV")
        joy_cmdlabel.setFont(font)

        self.forward_button = QPushButton('FORWARD', self)
        self.forward_button.setToolTip('Move robot in positive x direction')
        self.forward_button.move(100,70)
        self.forward_button.pressed.connect(self.on_pres_forward)
        self.forward_button.released.connect(self.on_rel_forward)

        self.back_button = QPushButton('BACKWARD', self)
        self.back_button.setToolTip('Move robot in negative x direction')
        self.back_button.move(100,70)
        self.back_button.pressed.connect(self.on_pres_back)
        self.back_button.released.connect(self.on_rel_back)


        self.left_button = QPushButton('LEFT', self)
        self.left_button.setToolTip('Move robot in positive y direction')
        self.left_button.move(100,70)
        self.left_button.pressed.connect(self.on_pres_left)
        self.left_button.released.connect(self.on_rel_left)


        self.right_button = QPushButton('RIGHT', self)
        self.right_button.setToolTip('Move robot in negative y direction')
        self.right_button.move(100,70)
        self.right_button.pressed.connect(self.on_pres_right)
        self.right_button.released.connect(self.on_rel_right)


        self.up_button = QPushButton('UP', self)
        self.up_button.setToolTip('Move robot in positive z direction')
        self.up_button.move(100,70)
        self.up_button.pressed.connect(self.on_pres_up)
        self.up_button.released.connect(self.on_rel_up)


        self.down_button = QPushButton('DOWN', self)
        self.down_button.setToolTip('Move robot in negative z direction')
        self.down_button.move(100,70)
        self.down_button.pressed.connect(self.on_pres_down)
        self.down_button.released.connect(self.on_rel_down)

        self.rot_left_button = QPushButton('ROTATE_LEFT', self)
        self.rot_left_button.setToolTip('rotate robot in left direction')
        self.rot_left_button.move(100,70)
        self.rot_left_button.pressed.connect(self.on_pres_yawleft)
        self.rot_left_button.released.connect(self.on_rel_yawleft)

        self.rot_right_button = QPushButton('ROTATE_RIGHT', self)
        self.rot_right_button.setToolTip('rotate robot in right direction')
        self.rot_right_button.move(100,70)
        self.rot_right_button.pressed.connect(self.on_pres_yawright)
        self.rot_right_button.released.connect(self.on_rel_yawright)

    
        self.layout.addWidget(self.textbox_state)
        self.layout.addWidget(self.textbox_pos)
        self.layout.addWidget(self.textbox_yaw)
        self.layout.addWidget(self.textbox_cmd_pos)
        self.layout.addWidget(self.textbox_cmd_yaw)
        self.layout.addWidget(self.arm_button)
        self.layout.addWidget(self.toff_button)
        self.layout.addWidget(self.land_button)
        self.layout.addWidget(self.offboard_button)

        self.layout.addWidget(self.checkbox_pvcontrol)
        #self.layout.addWidget(self.checkbox_vcontrol)

        self.layout.addWidget(x_cmdlabel)
        self.layout.addWidget(self.cmd_dx_slider)
        self.layout.addWidget(y_cmdlabel)
        self.layout.addWidget(self.cmd_dy_slider)
        self.layout.addWidget(z_cmdlabel)
        self.layout.addWidget(self.cmd_dz_slider)
        self.layout.addWidget(yaw_cmdlabel)
        self.layout.addWidget(self.cmd_dyaw_slider)
        self.layout.addWidget(self.center_slider_button)      


        self.layout.addWidget(joy_cmdlabel)
        self.layout.addWidget(self.forward_button)
        self.layout.addWidget(self.back_button)
        self.layout.addWidget(self.left_button)
        self.layout.addWidget(self.right_button)
        self.layout.addWidget(self.up_button)
        self.layout.addWidget(self.down_button)
        self.layout.addWidget(self.rot_left_button)
        self.layout.addWidget(self.rot_right_button)

        self.checkbox_pvcontrol.setChecked( True )
        

        self.forward_button.setEnabled(False)
        self.back_button.setEnabled(False)
        self.left_button.setEnabled(False)
        self.right_button.setEnabled(False)
        self.up_button.setEnabled(False)
        self.down_button.setEnabled(False)
        self.rot_left_button.setEnabled(False)
        self.rot_right_button.setEnabled(False)

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

    def select_chkbox_pvcontrol( self, state ):   
        if state == 0:
            self.cmd_dx_slider.setValue(0)
            self.cmd_dy_slider.setValue(0)
            self.cmd_dz_slider.setValue(0)
            self.cmd_dyaw_slider.setValue(0)

            self.center_slider_button.setEnabled( False )
            self.cmd_dx_slider.setEnabled( False )
            self.cmd_dy_slider.setEnabled( False )
            self.cmd_dz_slider.setEnabled( False )
            self.cmd_dyaw_slider.setEnabled( False )
            

            self.forward_button.setEnabled(True)
            self.back_button.setEnabled(True)
            self.left_button.setEnabled(True)
            self.right_button.setEnabled(True)
            self.up_button.setEnabled(True)
            self.down_button.setEnabled(True)
            self.rot_left_button.setEnabled(True)
            self.rot_right_button.setEnabled(True)
            self.ctrl_mode = "velocity"

        elif state == 2:
            self.cmd_dx_slider.setValue(0)
            self.cmd_dy_slider.setValue(0)
            self.cmd_dz_slider.setValue(0)
            self.cmd_dyaw_slider.setValue(0)

            self.center_slider_button.setEnabled( True )
            self.cmd_dx_slider.setEnabled( True )
            self.cmd_dy_slider.setEnabled( True )
            self.cmd_dz_slider.setEnabled( True )
            self.cmd_dyaw_slider.setEnabled( True )

            self.forward_button.setEnabled(False)
            self.back_button.setEnabled(False)
            self.left_button.setEnabled(False)
            self.right_button.setEnabled(False)
            self.up_button.setEnabled(False)
            self.down_button.setEnabled(False)
            self.rot_left_button.setEnabled(False)
            self.rot_right_button.setEnabled(False)
            self.ctrl_mode = "position"
        
        
        

    #def select_chkbox_pcontrol(self, state):
    #    print("select_chkbox_pcontrol state: ", state)
    #    self.checkbox_vcontrol.setChecked( False )
    #    self.checkbox_pcontrol.setChecked( True )


    #def select_chkbox_vcontrol( self, state ):        
    #    print("select_chkbox_vcontrol state: ", state)
    #    self.checkbox_pcontrol.setChecked( False )
    #    self.checkbox_vcontrol.setChecked( True )        

    def on_pres_yawleft(self):
        self.yaw_cmd = 0.3
        
    def on_rel_yawleft(self):
        self.yaw_cmd = 0.0

    def on_pres_yawright (self):
        self.yaw_cmd = -0.3

    def on_rel_yawright (self):
        self.yaw_cmd = 0.0
       

    def on_pres_forward(self):
        self.dy_cmd = 0.3    

    def on_rel_forward(self):
        self.dy_cmd = 0.0


    def on_pres_back(self):
        self.dy_cmd = -0.3    

    def on_rel_back(self):
        self.dy_cmd = 0.0

    def on_pres_left(self):
        self.dx_cmd = -0.3    

    def on_rel_left(self):
        self.dx_cmd = 0.0

    def on_pres_right(self):
        self.dx_cmd = 0.3    

    def on_rel_right(self):
        self.dx_cmd = 0.0

    def on_pres_up(self):
        self.dz_cmd = 0.3    

    def on_rel_up(self):
        self.dz_cmd = 0.0

    def on_pres_down(self):
        self.dz_cmd = -0.3    

    def on_rel_down(self):
        self.dz_cmd = 0.0


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

    def on_click_center(self):
        self.cmd_dx_slider.setValue(0)
        self.cmd_dy_slider.setValue(0)
        self.cmd_dz_slider.setValue(0)
        self.cmd_dyaw_slider.setValue(0)

    def on_click_offboard(self):            
        self.offboard_req( base_mode = 0, custom_mode = "OFFBOARD" )

    def changeValue_dyaw(self, value):
        self.yaw_cmd = ( value*3.1415)/180.0

    def changeValue_dx(self, value):       
        self.dx_cmd = value / 10.0
        
        
    def changeValue_dy(self, value):
        self.dy_cmd = value / 10.0

    def changeValue_dz(self, value):
        self.dz_cmd = value / 10.0
        

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
        self.pos_raw_msg.header.stamp = rospy.get_rostime()


        if( self.state == "OFFBOARD"):
            if self.ctrl_mode == "position":
                self.local_cmd[0] = self.local_cmd[0] + self.dx_cmd*0.02
                self.local_cmd[1] = self.local_cmd[1] + self.dy_cmd*0.02
                self.local_cmd[2] = self.local_cmd[2] + self.dz_cmd*0.02 
            else:
                self.local_cmd[0] = msg.pose.pose.position.x
                self.local_cmd[1] = msg.pose.pose.position.y
                self.local_cmd[2] = msg.pose.pose.position.z
                #self.yaw_cmd = 0.0

        else:
            self.yaw_cmd = yaw
            self.local_cmd[0] = msg.pose.pose.position.x
            self.local_cmd[1] = msg.pose.pose.position.y
            self.local_cmd[2] = msg.pose.pose.position.z
    
        if self.ctrl_mode == "position":
            self.pos_raw_msg.position.x = self.local_cmd[0]
            self.pos_raw_msg.position.y = self.local_cmd[1]
            self.pos_raw_msg.position.z = self.local_cmd[2]
            self.pos_raw_msg.yaw_rate = self.yaw_cmd
            self.textbox_cmd_pos.setText( "Cmd Pos: " + str(round(self.local_cmd[0], 2)) + " " + str( round(self.local_cmd[1], 2)) + " " + str( round(self.local_cmd[2], 2)) )
            self.textbox_cmd_yaw.setText( "Cmd dYaw: " + str(round(self.yaw_cmd, 2)) + " rad/s")
            self.setpoint_raw_pub.publish( self.pos_raw_msg )


        elif self.ctrl_mode == "velocity":
            self.vel_raw_msg.velocity.x = self.dx_cmd
            self.vel_raw_msg.velocity.y = self.dy_cmd
            self.vel_raw_msg.velocity.z = self.dz_cmd
            self.vel_raw_msg.yaw_rate = self.yaw_cmd
            self.textbox_cmd_pos.setText( "Cmd Vel: " + str(self.dx_cmd) + " " + str( self.dy_cmd ) + " " + str( self.dz_cmd) )
            self.textbox_cmd_yaw.setText( "Cmd dYaw: " + str(round(self.yaw_cmd, 2)) + " rad/s")
            self.setpoint_raw_pub.publish( self.vel_raw_msg )



if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = UAVStatePublisher()
    sys.exit(app.exec_())
