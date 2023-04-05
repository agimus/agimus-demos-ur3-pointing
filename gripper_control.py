#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command

class GripperControl(object):

    def __init__(self):
        rospy.init_node ('gripper_control')
        self.sub = rospy.Subscriber('agimus/sot/gripper_status',
                                         Bool, self.callback)
        
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput',
                                        outputMsg.Robotiq2FGripper_robot_output,
                                        queue_size = 10)
        
        #Init -> activate gripper
        self.cmd = outputMsg.Robotiq2FGripper_robot_output()
        self.cmd = genCommand('r', self.cmd) 
        
        #Wait until publisher is ready
        while self.pub.get_num_connections() <= 0:
            continue
        self.pub.publish(self.cmd)
        rospy.sleep(0.1)
        self.cmd = genCommand('a', self.cmd) 
        self.pub.publish(self.cmd)

    def callback(self, msg):

        if msg.data: #Close
            self.cmd.rPR = 195
        else: #Open
            self.cmd.rPR = 0
        self.pub.publish(self.cmd)
        rospy.sleep(0.1)
        
try:
    gc = GripperControl()
    rospy.spin()
except rospy.ROSInterruptException:
    pass