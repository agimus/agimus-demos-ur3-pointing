# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys, argparse, numpy as np, time, rospy
from math import pi, sqrt
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, PathGenerator

class PartPlaque:
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/plaque-ur3.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/plaque-ur3.srdf"
    rootJointType = "freeflyer"

# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of UR3 pointing')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
args = p.parse_args ()

joint_bounds = {}
def setRobotJointBounds(which):
    for jn, bound in jointBounds[which]:
        robot.setJointBounds(jn, bound)

try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/ur3/pointing/urdf/robot.urdf.xacro",
       "transmission_hw_interface:=hardware_interface/PositionJointInterface")
Robot.srdfString = ""

loadServerPlugin (args.context, "manipulation-corba.so")
newProblem()
client = CorbaClient(context=args.context)
#client.basic._tools.deleteAllServants()
client.manipulation.problem.selectProblem (args.context)
def wd(o):
    from hpp.corbaserver import wrap_delete
    return wrap_delete(o, client.basic._tools)

robot = Robot("robot", "ur3", rootJointType="anchor", client=client)
crobot = wd(wd(robot.hppcorba.problem.getProblem()).robot())

print("Robot loaded")
robot.opticalFrame = 'camera_color_optical_frame'
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")

ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .2)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)

## Shrink joint bounds of UR-5
#
jointBounds = dict()
jointBounds["unlimited"] = [('ur3/shoulder_pan_joint', [-2*pi, 2*pi]),
  #('ur3/shoulder_pan_joint', [-pi, pi])
  ('ur3/shoulder_lift_joint', [-2*pi, 2*pi]),
  ('ur3/elbow_joint', [-2*pi, 2*pi]),
  ('ur3/wrist_1_joint', [-2*pi, 2*pi]),
  ('ur3/wrist_2_joint', [-2*pi, 2*pi]),
  ('ur3/wrist_3_joint', [-2*pi, 2*pi])]

setRobotJointBounds("unlimited")

## Remove some collision pairs
ur3JointNames = list(filter(lambda j: j.startswith("ur3/"), robot.jointNames))
ur3LinkNames = [ robot.getLinkNames(j) for j in ur3JointNames ]

# Get class_name str from rosparam
Part_name = rospy.get_param('/demo/objects/part/class_name');
# Instanciate the Part
try:
    class_ = globals()[Part_name]
except Exception as e:
    raise ValueError("Probably unvalid part name, check yaml files or rosparam") from e
Part = class_()

vf.loadRobotModel (Part, "part")

#Part joint
robot.setJointBounds('part/root_joint', [-0.388, 0.372,
                                          -0.795, 0.135, 
                                           0.98, 1.7392])
print(f"{Part.__class__.__name__} loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur3", "package://agimus_demos/srdf/ur3_robot.srdf")

def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))

n = norm([0, 0, 0.7071068, 0.7071068])
partPose =  [0, -0.35, 1.01, 0, 0, 0.7071068/n, 0.7071068/n]

## Define initial configuration
q0 = robot.getCurrentConfig()
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]

## Define initial configuration
q0 = robot.getCurrentConfig()
q_calib = [-2.4942739645587366, -1.5445440451251429, -1.0446308294879358, -1.9494207541095179, 1.4083086252212524, 0.6596791744232178, 0, 0, 0, 0, 0, 0, 0]
q_calib_2 = [-2.783581797276632, -2.3941038290606897, 0.40172576904296875, -2.598349396382467, 1.5528945922851562, 0.37268954515457153, 0, 0, 0, 0, 0, 0, 0]
q_calib_3 = [-0.17332965532411748, -0.5632131735431116, -0.6789363066302698, -0.428781811391012, -1.561974827443258, -0.20535022417177373,0, 0, 0, 0, 0, 0, 0]

r = robot.rankInConfiguration['part/root_joint']
q0[r:r+7] = partPose
q_calib[r:r+7] = partPose
q_calib_2[r:r+7] = partPose
q_calib_3[r:r+7] = partPose
gripper = 'ur3/gripper'


## Create specific constraint for a given handle
#  Rotation is free along x axis.
#  Note that locked part should be added to loop edge.
def createFreeRxConstraintForHandle(handle):
    name = gripper + ' grasps ' + handle
    handleJoint, jMh = robot.getHandlePositionInJoint(handle)
    gripperJoint, jMg = robot.getGripperPositionInJoint(gripper)
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, jMg, jMh,
         [True, True, True, False, True, True])
    # pregrasp
    shift = 0.13
    M = Transform(jMg)*Transform([shift,0,0,0,0,0,1])
    name = gripper + ' pregrasps ' + handle
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, M.toTuple(), jMh,
         [True, True, True, False, True, True])

def createConstraintGraph():
    # Return a list of available elements of type type handle
    all_handles = ps.getAvailable('handle')
    part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))
        
    # Define the set of contact surfaces used for each object
    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)
    #Set gripper
    factory.setGrippers(["ur3/gripper",])
    #Set kapla
    factory.setObjects(["part",], [part_handles], [["ur3/top",],])
    factory.setPreplacementDistance("part", 0.01)
    factory.generate()
    for handle in all_handles:
        loopEdge = 'Loop | 0-{}'.format(factory.handles.index(handle))
        graph.addConstraints(edge = loopEdge, constraints = Constraints
            (numConstraints=['part/root_joint']))

    n = norm([-0.576, -0.002, 0.025, 0.817])
    ps.createTransformationConstraint('look-at-part', 'part/base_link', 'ur3/wrist_3_link',
                                    [-0.126, -0.611, 1.209, -0.576/n, -0.002/n, 0.025/n, 0.817/n],
                                    [True, True, True, True, True, True,])
    graph.createNode(['look-at-part'])
    graph.createEdge('free', 'look-at-part', 'go-look-at-part', 1, 'free')
    graph.createEdge('look-at-part', 'free', 'stop-looking-at-part', 1, 'free')

    graph.addConstraints(node='look-at-part',
                        constraints = Constraints(numConstraints=['look-at-part']))
    ps.createTransformationConstraint('placement/complement', '','part/base_link',
                                    [0,0,0,0, 0, 0, 1],
                                    [True, True, True, True, True, True,])
    ps.setConstantRightHandSide('placement/complement', False)
    graph.addConstraints(edge='go-look-at-part',
                        constraints = Constraints(numConstraints=\
                                                ['placement/complement']))
    graph.addConstraints(edge='stop-looking-at-part',
                        constraints = Constraints(numConstraints=\
                                                ['placement/complement']))
    sm = SecurityMargins(ps, factory, ["ur3", "part"])
    sm.setSecurityMarginBetween("universe", "part", float("-inf"))
    sm.setSecurityMarginBetween("ur3", "part", 0.015)
    sm.setSecurityMarginBetween("ur3", "ur3", 0)
    sm.defaultMargin = 0.01
    sm.apply()

    graph.initialize()
    
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return factory, graph

factory, graph = createConstraintGraph()

try:
    v = vf.createViewer()
    v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")

ri = None
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)

#Path Generator
pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()

pg.setIsClogged(None)
ps.setTimeOutPathPlanning(15)

### DEMO
#q_init = ri.getObjectPose(q_init)
#Set Init
ps.setInitialConfig(q_init)
v(q_init)

#Execute a pointing task for a list of handle/holes
def executeSeveralHoles(listOfHole, q_init=q_init):
    for i in listOfHole:
        #Update with the current config
        q_init = ri.getCurrentConfig(q_init)
        #Generate a path 
        id, q = pg.planPointingPathForHole(i, q_init, 50)
        #Execute the path without visualisation
        pg.demo_execute(id, steps=False, visualize=False)