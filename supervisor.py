# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel
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

# Theses variables are defined:
# - robot, a SoT device
# - simulateTorqueFeedbackForEndEffector, a boolean

import time
from agimus_sot.react import TaskFactory, localizeObjectOnLoopTransition
from agimus_sot.action import Action
from dynamic_graph.ros.ros_publish import RosPublish

Action.maxControlSqrNorm = 20
class CloseGripper(object):
    timeout = 5
    def __init__(self, sotrobot):
        self.sotrobot = sotrobot
        self.gripperCloseTopic = "/agimus/sot/gripper_status"
        self.gripperClosePublisher = RosPublish("GripperClose")
        self.gripperClosePublisher.add('boolean', 'gripperClosePublisher_', self.gripperCloseTopic)
        self.gripperClosePublisher.signal('gripperClosePublisher_').value = True

    def __call__(self):
        ts = self.sotrobot.device.getTimeStep()
        to = int(self.timeout / self.sotrobot.device.getTimeStep())
        start_it = self.sotrobot.device.control.time
        while True:
            t = self.sotrobot.device.control.time
            if t > start_it + to:
                print("Failed to grasp")
                return False, "Failed to grasp"
            else:
                self.gripperClosePublisher.signal('trigger').recompute(t)
                time.sleep(ts)
                time.sleep(1.)
                return True, ""    
class OpenGripper(object):
    timeout = 5
    def __init__(self, sotrobot):
        self.sotrobot = sotrobot
        self.gripperOpenTopic = "/agimus/sot/gripper_status"
        self.gripperOpenPublisher = RosPublish("GripperOpen")
        self.gripperOpenPublisher.add('boolean', 'gripperOpenPublisher_', self.gripperOpenTopic)
        self.gripperOpenPublisher.signal('gripperOpenPublisher_').value = False

    def __call__(self):
        ts = self.sotrobot.device.getTimeStep()
        to = int(self.timeout / self.sotrobot.device.getTimeStep())
        start_it = self.sotrobot.device.control.time
        while True:
            t = self.sotrobot.device.control.time
            if t > start_it + to:
                print("Failed to grasp")
                return False, "Failed to grasp"
            else:
                self.gripperOpenPublisher.signal('trigger').recompute(t)
                time.sleep(ts)
                time.sleep(1.)
                return True, ""        
def makeLoopTransitionBIS (factory, state,edge):
    sot = factory._newSoT ('sot_'+edge)
    from agimus_sot.events import logical_and_entity
    sot. doneSignal = logical_and_entity("ade_sot_"+edge,
    [   factory.supervisor.done_events.timeEllapsedSignal,
    factory.supervisor.done_events.controlNormSignal])
    
    factory.hpTasks.pushTo(sot)
    state.manifold.pushTo(sot)
    factory.lpTasks.pushTo(sot)
    
    factory.actions[edge] = sot                

def wait():
    print("Waiting 1 second")
    time.sleep(1)
    return True, ""

def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf, attach_all_to_link
    import pinocchio
    from rospkg import RosPack
    rospack = RosPack()

    if not hasattr(robot, "camera_frame"):
        robot.camera_frame = "camera_color_optical_frame"
    srdf = {}
    # retrieve objects from ros param
    robotDict = globalDemoDict["robots"]
    if len(robotDict) != 1:
        raise RuntimeError("One and only one robot is supported for now.")
    objectDict = globalDemoDict["objects"]
    objects = list(objectDict.keys())
    # parse robot and object srdf files
    srdfDict = dict()
    for r, data in robotDict.items():
        srdfDict[r] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=r)
    for o, data in objectDict.items():
        objectModel = pinocchio.buildModelFromUrdf\
                      (rospack.get_path(data["urdf"]["package"]) +
                       data["urdf"]["file"])
        srdfDict[o] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=o)
        attach_all_to_link(objectModel, "base_link", srdfDict[o])

    grippers = list(globalDemoDict["grippers"])
    handlesPerObjects = list()
    contactPerObjects = list()
    for o in objects:
        handlesPerObjects.append(sorted(list(srdfDict[o]["handles"].keys())))
        contactPerObjects.append(sorted(list(srdfDict[o]["contacts"].keys())))

    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for k, data in srdfDict.items():
            srdf[w].update(data[w])

    supervisor = Supervisor(robot, prefix=list(robotDict.keys())[0])
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.01  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False

    from hpp.corbaserver.manipulation import Rule
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    
    grasps = (None,) * len(factory.grippers)

    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'Loop | f')    

    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'Loop | 0-0')
    
    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'ur3e/GRIPPER > kapla/handle | f_01')
    
    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'ur3e/GRIPPER > kapla/handle | f_12')
    
    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'ur3e/GRIPPER < kapla/handle | 0-0_10')
    
    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'ur3e/GRIPPER < kapla/handle | 0-0_21')
    
    state = factory.makeState(grasps, 1)
    makeLoopTransitionBIS(factory, state, 'ur3e/GRIPPER < kapla/handle | 0-0')
    
    factory.generate()
    
    #post Action
    closeGripper = CloseGripper(robot)
    supervisor.actions['Loop | 0-0'].preActions.append(closeGripper)
    
    openGripper = OpenGripper(robot)
    supervisor.actions['ur3e/GRIPPER < kapla/handle | 0-0_21'].preActions.append(openGripper)
    
    supervisor.makeInitialSot()
    
    return factory, supervisor


# Use service /agimus/sot/set_base_pose to set initial config
factory, supervisor = makeSupervisorWithFactory(robot)

supervisor.plugTopicsToRos()
supervisor.plugSot("")

