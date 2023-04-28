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

useFOV = False


class AprilTagPlank:
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/april-tag-plank.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/april-tag-plank.srdf"
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
# ps.selectConfigurationShooter('Gaussian')
# ps.setParameter('ConfigurationShooter/Gaussian/center', 12*[0.] + [1.])
# ps.setParameter('ConfigurationShooter/Gaussian/standardDeviation', 0.25)
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .1)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)

## Shrink joint bounds of UR-5
#
jointBounds = dict()
jointBounds["default"] = [ (jn, robot.getJointBounds(jn)) \
                           if not jn.startswith('ur3/') else
                           (jn, [-pi, pi]) for jn in robot.jointNames]
jointBounds["limited"] = [('ur3/shoulder_pan_joint', [-3.9, pi]),
  ('ur3/shoulder_lift_joint', [-3.9, pi]),
  ('ur3/elbow_joint', [-3.1, 3.1]),
  ('ur3/wrist_1_joint', [-3.2, 3.2]),
  ('ur3/wrist_2_joint', [-3.2, 3.2]),
  ('ur3/wrist_3_joint', [-3.2, 3.2])]
# Bounds to generate calibration configurations
jointBounds["calibration"] = [('ur3/shoulder_pan_joint', [-2.5, 2.5]),
  ('ur3/shoulder_lift_joint', [-2.5, 2.5]),
  ('ur3/elbow_joint', [-2.5, 2.5]),
  ('ur3/wrist_1_joint', [-2.5, 2.5]),
  ('ur3/wrist_2_joint', [-2.5, 2.5]),
  ('ur3/wrist_3_joint', [-2.5, 2.5])]
setRobotJointBounds("limited")
## Remove some collision pairs
#
ur3JointNames = list(filter(lambda j: j.startswith("ur3/"), robot.jointNames))
ur3LinkNames = [ robot.getLinkNames(j) for j in ur3JointNames ]

## Load P72
#[1., 0, 0.8,0,0,-sqrt(2)/2,sqrt(2)/2]

# Get class_name str from rosparam
Part_name = rospy.get_param('/demo/objects/AprilTagPlank/class_name');
# Instanciate the Part
try:
    class_ = globals()[Part_name]
except Exception as e:
    raise ValueError("Probably unvalid part name, check yaml files or rosparam") from e
Part = class_()

vf.loadRobotModel (Part, "part")

#Kapla joint
robot.setJointBounds('part/root_joint',  [-0.388, 0.372,
                                          -0.795, 0.135, 
                                           1, 1.7392])
print(f"{Part.__class__.__name__} loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur3", "package://agimus_demos/srdf/ur3_robot.srdf")

# VISUAL(modification pour voir le visual servoing sur Gazebo)
# JESSY 07/12 change partPose[0]: 1.4 -> 1.6
    
def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))


n = norm([0, 0, 0.7071068, 0.7071068])
#partPose =  [0.044, -0.2785, 1.001, 0.0, 0.0,  0.7071068/n, 0.7071068/n]
partPose =  [0.044, -0.2785, 1.001, 0,0,0,1]
## Define initial configuration
q0 = robot.getCurrentConfig()
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]

## Define initial configuration
q0 = robot.getCurrentConfig()
q_calib = [0.07205140590667725, -0.4533942381488245, -0.6467822233783167, -0.5376704374896448, -1.5575411955462855, 0.037997107952833176, 0.119, -0.371, 1.001, 0, 0, 0, 1]

q_calib_2 = [-0.08119804063905889, -0.41484290758241826, -0.7924998442279261, -0.3500474135028284, -1.6260665098773401, -0.08863956132997686, 0.044, -0.2785, 1.001, 0.0, 0.0, 0.7071067811865476, 0.7071067811865476]


r = robot.rankInConfiguration['part/root_joint']
q0[r:r+7] = partPose
q_calib[r:r+7] = partPose
q_calib_2[r:r+7] = partPose


def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))

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

## Build constraint graph
def createConstraintGraph():
    all_handles = ps.getAvailable('handle')
    part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))

    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers(["ur3/gripper",])
    factory.setObjects(["part",], [part_handles], [[]])
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
# q_init = q0 #robot.getCurrentConfig()
pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()
NB_holes = 5 * 7
NB_holes_total = 44
hidden_holes = [0,2,10,11,12,14,16,24,33]
holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]
pg.setIsClogged(None)
ps.setTimeOutPathPlanning(10)

if useFOV:
    def configHPPtoFOV(q):
        return q[:6] + q[-7:]

    from ur3_fov import RobotFOV, RobotFOVGuiCallback, Feature, Features
    ur3_fov = RobotFOV(urdfString = Robot.urdfString,
                        fov = np.radians((69.4, 52)),
                        geoms = [],
                        optical_frame = "camera_color_optical_frame",
                        group_camera_link = "robot/ur3/ref_camera_link",
                        camera_link = "ref_camera_link",
                        modelConfig = configHPPtoFOV)
    robot.setCurrentConfig(q_init)
    # Add Plaque model in the field of view object
    # to test if the plaque itself obstructs the view to the features
    oMh, oMd = robot.hppcorba.robot.getJointsPosition(q_init, ["universe", "part/base_link"])
    fMm = (Transform(oMh).inverse() * Transform(oMd)).toTuple()
    ur3_fov.appendUrdfModel(PartPlaque.urdfFilename, "universe",
        fMm, prefix="part/")
    feature_list = []
    for i in range(1, NB_holes_total+1):
        feature_list.append( Feature('part/hole_' + str(i).zfill(2) + '_link', 0.003) )
    featuress = [Features(feature_list, 2, 0.005, 0)]
    ur3_fov_gui = RobotFOVGuiCallback(robot, ur3_fov, featuress, modelConfig = configHPPtoFOV)
    # Display Robot Field of view.
    #vf.guiRequest.append( (ur3_fov.loadInGui, {'self':None}))
    # Display visibility cones.
    vf.addCallback(ur3_fov_gui)
    isClogged = lambda x : ur3_fov.clogged(x, robot, featuress)
    pg.setIsClogged(isClogged)
    visibleFeatures = lambda x : ur3_fov.visible(x, robot, featuress)

### DEMO

def getDoableHoles():
    doableHoles = []
    non_doableHoles = []
    pg.testGraph()
    for i in range(NB_holes_total):
        if pg.isHoleDoable(i):
            doableHoles.append(i)
        else:
            non_doableHoles.append(i)
    print("Doable holes : ", doableHoles)
    print("Non doable holes : ", non_doableHoles)
    return doableHoles, non_doableHoles

def doDemo():
    NB_holes_to_do = 7
    demo_holes = range(NB_holes_to_do)
    pids, qend = pg.planPointingPaths(demo_holes)

holist = [7,8,9,42,43,13]
v(q_init)


### SUITE
from calibration import Calibration, checkData

calibration = Calibration(ps, graph, factory)
calibration.robot_name = "ur3"
calibration.camera_frame = 'camera_color_optical_frame'
calibration.chessboardCenter = (0, 0, 0)
calibration.addStateToConstraintGraph()
#calibration.generateConfigurationsAndPaths(q_init, nbConfigs = 10)
