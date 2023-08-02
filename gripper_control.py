#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

class GripperControl(object):
    def __init__(self):
        rospy.init_node ('gripper_control')
        self.sub = rospy.Subscriber('agimus/sot/gripper_status',
                                         Bool, self.callback)
        
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput',
                                        outputMsg.Robotiq2FGripper_robot_output,
                                        queue_size = 10)
        
        #Init -> Reset & Activate gripper
        #Reset
        self.cmd = outputMsg.Robotiq2FGripper_robot_output()
        self.cmd.rACT = 0    #Reset
        
        #Wait until publisher is ready
        while self.pub.get_num_connections() <= 0:
            continue
        self.pub.publish(self.cmd)
        rospy.sleep(0.1)
        
        #Activate 
        self.cmd.rACT = 1
        self.cmd.rGTO = 1
        self.cmd.rSP = 255
        self.cmd.rFR = 150
        self.pub.publish(self.cmd)

    def callback(self, msg):
        if msg.data: #Close
            self.cmd.rPR = 200
            self.cmd.rSP = 30
        else: #Open
            self.cmd.rPR = 180
            self.cmd.rSP = 30
        self.pub.publish(self.cmd)
        rospy.sleep(0.1)

#Main        
try:
    gc = GripperControl()
    rospy.spin()
except rospy.ROSInterruptException:
    pass