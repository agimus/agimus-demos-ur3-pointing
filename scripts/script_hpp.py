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

class PartPlaque:
    urdfFilename = "package://agimus_demos/ur3/pointing/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/ur3/pointing/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"

class AprilTagPlank:
    urdfFilename = "package://agimus_demos/urdf/april-tag-plank.urdf"
    srdfFilename = "package://agimus_demos/srdf/april-tag-plank.srdf"
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

robot = Robot("robot", "ur3e", rootJointType="anchor", client=client)
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
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
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
jointBounds["limited"] = [('ur3e/shoulder_pan_joint', [-pi, pi]),
  ('ur3e/shoulder_lift_joint', [-pi, pi]),
  ('ur3e/elbow_joint', [-3.1, 3.1]),
  ('ur3e/wrist_1_joint', [-3.2, 3.2]),
  ('ur3e/wrist_2_joint', [-3.2, 3.2]),
  ('ur3e/wrist_3_joint', [-3.2, 3.2])]
# Bounds to generate calibration configurations
jointBounds["calibration"] = [('ur3e/shoulder_pan_joint', [-2.5, 2.5]),
  ('ur3e/shoulder_lift_joint', [-2.5, 2.5]),
  ('ur3e/elbow_joint', [-2.5, 2.5]),
  ('ur3e/wrist_1_joint', [-2.5, 2.5]),
  ('ur3e/wrist_2_joint', [-2.5, 2.5]),
  ('ur3e/wrist_3_joint', [-2.5, 2.5])]
setRobotJointBounds("limited")
## Remove some collision pairs
#
ur3JointNames = list(filter(lambda j: j.startswith("ur3/"), robot.jointNames))
ur3LinkNames = [ robot.getLinkNames(j) for j in ur3JointNames ]

## Load P72
#[1., 0, 0.8,0,0,-sqrt(2)/2,sqrt(2)/2]

# Get class_name str from rosparam
Part_name = rospy.get_param('/demo/objects/part/class_name');
# Instanciate the Part
try:
    class_ = globals()[Part_name]
except Exception as e:
    raise ValueError("Probably unvalid part name, check yaml files or rosparam") from e
Part = class_()

vf.loadRobotModel (Part, "part")

# JESSY 07/12 change part/root_joint y: 1.5-> 1.75
robot.setJointBounds('part/root_joint', [1, 1.75, -0.5, 0.5, -0.5, 0.5])
print(f"{Part.__class__.__name__} loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur3e", "package://agimus_demos/srdf/ur3_robot.srdf")

# VISUAL(modification pour voir le visual servoing sur Gazebo)
# JESSY 07/12 change partPose[0]: 1.4 -> 1.6
partPose = [1.6, 0.1, 0,0,0,-sqrt(2)/2,sqrt(2)/2]

## Define initial configuration
q0 = robot.getCurrentConfig()
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
r = robot.rankInConfiguration['part/root_joint']
if isinstance(Part,AprilTagPlank):
    # Initial configuration of AprilTagPlank
    q0[r:r+7] = [1.3, 0, 0, 0, 0, -1, 0]
else:
    q0[r:r+7] = partPose
## Home configuration
q_homeLAAS = [-3.415742983037262e-05, -1.5411089223674317, 2.7125137487994593, -1.5707269471934815, -3.141557280217306, 6.67572021484375e-06, 1.3, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]

## JESSY 14/12 New Home configuration
q_home = [1.571, -1.575, 2.753, -2.622, -1.571, 0.0,1.3, 0, 0, 0, 0, -0.7071, 0.7071]

# JESSY 15/12
# config intermédiaire entre calib et les trous
q_int = [1.431, -1.983, 2.243, -0.253, -1.014, 0.000,1.3, 0, 0, 0, 0, -0.7071, 0.7071]

# JESSY 05/12 Add calib
## Calibration configuration: the part should be wholly visible
q_calibLAAS = [1.5707, -3, 2.5, -2.8, -1.57, 0.0, 1.3, 0.0, 0.0, 0.0, 0.0, -0.7071067811865476, 0.7071067811865476]
# Saint Nazaire calib configuration
q_calibSN = [1.3707, -3.1, 2.1, -2.32, -1.57, 0.0, 1.3, 0.0, 0.0, 0.0, 0.0, -0.7071067811865476, 0.7071067811865476]

## PointCloud position : the part should fit the field of view
q_pointcloudF = [1.465, -2.058, 2.076, -0.436, 1.5, -3.14, 1.2804980083956572, 0.11300105405990518, -0.031348192422114174, -0.008769144315009561, 0.004377057629846714, -0.7073469546030107, 0.7067985775935985]
q_pointcloud = [1.4802236557006836, -1.7792146009257812, 2.4035003821002405, -0.9398099416545411, 1.5034907341003418, -3.1523403135882773, 1.2804980083956572, 0.11300105405990518, -0.031348192422114174, -0.008769144315009561, 0.004377057629846714, -0.7073469546030107, 0.7067985775935985]
q_pointcloud2 = [1.465, -1.465, 2.39, -1.134, 1.5, -3.14, 1.166362251685465, 0.18398354959470994, -0.040275835859414855, -0.011115686791169354, 0.00903223476857969, -0.701584375075136, 0.7124424361958492]

q_pc1 = [1.4802236557006836, -2.7792393169798792, 2.6035669485675257, 0.06014220296826167, 1.5035147666931152, -3.1523261705981653, 1.1430909403610665, 0.22893782415973665, -0.04789935128099243, -0.0033794390562739483, 0.008636413673842097, -0.6985826418521135, 0.7154692755481825]
q_pc2 = [1.480247974395752, -2.7792188129820765, 2.6035461584674278, -0.13980153024707037, 1.5035266876220703, -3.1523383299456995, 1.1138463304333714, 0.23051956151197342, -0.040682700437678854, -0.00938303410217683, 0.013303913345295534, -0.7001874363145009, 0.7137734364544993]


# JESSY 13/12
# New configuration for pointCloud demo Saint-Nazaire
# q_pc1SN = [-2,-35,-143,0,7,-95,1.5, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476] # DEG
q_pc1SN = [-0.03490658503988659,-0.6108652381980153,-2.498091544796509,0,0.12217304763960307,-1.658062789394634,1.5, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]

# q_pc2SN = [-2,-121,122,0,2,-95,1.5, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476] # DEG
q_pc2SN = [-0.03490658503988659,-2.111848398575904,2.129311522021689,0,0.03490658503988659,-1.658062789394634,1.5, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]


def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))

gripper = 'ur3e/gripper'
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
    factory.setGrippers(["ur3e/gripper",])
    factory.setObjects(["part",], [part_handles], [[]])
    factory.generate()
    for handle in all_handles:
        loopEdge = 'Loop | 0-{}'.format(factory.handles.index(handle))
        graph.addConstraints(edge = loopEdge, constraints = Constraints
            (numConstraints=['part/root_joint']))

    n = norm([-0.576, -0.002, 0.025, 0.817])
    ps.createTransformationConstraint('look-at-part', 'part/base_link', 'ur3e/wrist_3_link',
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
    sm = SecurityMargins(ps, factory, ["ur3e", "part"])
    sm.setSecurityMarginBetween("ur3e", "part", 0.015)
    sm.setSecurityMarginBetween("ur3e", "ur3e", 0)
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

# JESSY 05/12 change calib configuration
pg.setConfig("homeLAAS", q_homeLAAS)
pg.setConfig("home", q_home)
pg.setConfig("calibLAAS", q_calibLAAS)
pg.setConfig("calibSN", q_calibSN) # set calib configuration for Saint-Nazaire Demo
pg.setConfig("calib", q_calibSN) # set calib configuration for Saint-Nazaire Demo
pg.setConfig("inter", q_int) # set inter configuration for Saint-Nazaire Demo

pg.setConfig("pointcloud", q_pointcloud)
pg.setConfig("pointcloud2", q_pointcloud2)
pg.setConfig("pointcloud_bas", q_pc1)
pg.setConfig("pointcloud_haut", q_pc2)

# JESSY 13/12 ADD pointCloud configurations for SN Demo
pg.setConfig("pointcloudF", q_pointcloudF)
pg.setConfig("pc1SN", q_pc1SN)
pg.setConfig("pc2SN", q_pc2SN)
# pg.setConfig("pc3SN", q_pc3SN)


if useFOV:
    def configHPPtoFOV(q):
        return q[:6] + q[-7:]

    from ur3_fov import RobotFOV, RobotFOVGuiCallback, Feature, Features
    ur3_fov = RobotFOV(urdfString = Robot.urdfString,
                        fov = np.radians((69.4, 52)),
                        geoms = [],
                        optical_frame = "camera_color_optical_frame",
                        group_camera_link = "robot/ur3e/ref_camera_link",
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

