from numpy import pi
from hpp.corbaserver.manipulation import ConstraintGraph, ConstraintGraphFactory, Constraints
from utils import norm, EulerToQuaternion
from init_ur3 import robot, ps, q_init


######################################################
### Constraint Graph to grasp a kapla from factory ###
######################################################  
def createConstraintGraph():
    # Return a list of available elements of type type handle
    all_handles = ps.getAvailable('handle')
    part_handles = list(filter(lambda x: x.startswith("kapla/"), all_handles))
        
    # Define the set of contact surfaces used for each object
    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)
    #Set gripper
    factory.setGrippers(["ur3e/GRIPPER",])
    #Set kapla
    factory.setObjects(["kapla",], [part_handles], [["ur3e/top",],])
    factory.generate()

    graph.initialize()
    
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return factory, graph

################################################
### Custom Constraint Graph to grasp a kapla ###
################################################    
def createConstraintGraphCustom():
    graph = ConstraintGraph(robot, 'graph2')
    factory = ConstraintGraphFactory(graph)

    #Constraints
    qw, qx, qy, qz = EulerToQuaternion(pi,pi/2,0)
    qx, qy, qz, qw = -0.5, -0.5, 0.5, -0.5
    print(qx, qy, qz, qw)
    ps.createTransformationConstraint(
            'grasp', 
            'ur3e/GRIPPER',
            'kapla/root_joint',
            [0, 0, 0, 0, 0, 0, 1],
            [True, True, True, True, True, True,],)
    
    #Kapla in horizontal plane with free rotation z
    ps.createTransformationConstraint(
            'placement', 
            '',
            'kapla/root_joint',
            [0, 0, 1.009, 0,0,0,1],
            [False, False, True, True, True, False,],)
    
    ps.createTransformationConstraint(
            'placement/complement', 
            '',
            'kapla/root_joint',
            [0, 0, 1.009, 0,0,0,1],
            [True, True, False, False, False, True],)
    
    ps.createTransformationConstraint(
            'pre-grasp', 
            'ur3e/robotiq_85_gripper',
            'kapla/root_joint',
            [0, 0.20,0.075,qx, qy, qz, qw],
            [True, True, True, True, True, True,],)
     
    ps.createTransformationConstraint(
            'pre-release', 
            '',
            'kapla/root_joint',
            [0, 0, 1.109, 0,0,0,1],
            [False, False, True, True, True, True,],)
    
    ps.createTransformationConstraint(
            'pre-grasp/complement', 
            '',
            'ur3e/wrist_3_joint',
            [0, 0, 1.109, 0,0,0,1],
            [True, True, False, False, False, False,],)
    

    ps.setConstantRightHandSide("placement", True)
    ps.setConstantRightHandSide("pre-grasp", True)
    ps.setConstantRightHandSide("placement/complement", False)
    
    ps.setConstantRightHandSide("pre-grasp/complement", False)
    #Nodeskapla/root_joint
    graph.createNode(['grasp_placement',
                      'kapla_above_ground',
                      'gripper_above_kapla',
                      'grasp',
                      'placement'])

    #Edge
    graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
    graph.createEdge('placement', 'gripper_above_kapla', 'approach_kapla', 1, 'placement')
    graph.createEdge('gripper_above_kapla', 'grasp_placement', 'grasp_kapla', 1, 'placement')
    graph.createEdge('grasp_placement', 'kapla_above_ground', 'take_kapla_up', 1, 'grasp')
    graph.createEdge('kapla_above_ground', 'grasp', 'take_kapla_away', 1, 'grasp')
    graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
    graph.createEdge('grasp', 'kapla_above_ground', 'approach_ground', 1, 'grasp')
    graph.createEdge('kapla_above_ground', 'grasp_placement', 'put_kapla_down', 1, 'grasp')
    graph.createEdge('grasp_placement', 'gripper_above_kapla', 'move_gripper_up', 1, 'placement')
    graph.createEdge('gripper_above_kapla', 'placement', 'move_gripper_away', 1, 'placement')

    #Apply constraints
    graph.addConstraints(node='placement',constraints = Constraints(numConstraints=['placement']))
    graph.addConstraints(node='gripper_above_kapla',constraints = Constraints(numConstraints=['placement','pre-grasp']))
    graph.addConstraints(node='grasp_placement',constraints = Constraints(numConstraints=['placement', 'grasp']))
    graph.addConstraints(node='kapla_above_ground',constraints = Constraints(numConstraints=['pre-release','grasp']))
    graph.addConstraints(node='grasp',constraints = Constraints(numConstraints=['grasp']))
    
    graph.addConstraints(edge='transit', constraints = Constraints(numConstraints=['placement/complement']))
    graph.addConstraints(edge='approach_kapla', constraints = Constraints(numConstraints=['placement/complement']))
    graph.addConstraints(edge='grasp_kapla', constraints = Constraints(numConstraints=['placement/complement']))
    graph.addConstraints(edge='take_kapla_up', constraints = Constraints(numConstraints=['grasp','placement/complement',]))
    graph.addConstraints(edge='take_kapla_away', constraints = Constraints(numConstraints=['grasp']))
    graph.addConstraints(edge='transfer', constraints = Constraints(numConstraints=['grasp']))
    graph.addConstraints(edge='approach_ground', constraints = Constraints(numConstraints=['grasp']))
    graph.addConstraints(edge='put_kapla_down', constraints = Constraints(numConstraints=['grasp', 'placement/complement']))
    graph.addConstraints(edge='move_gripper_up', constraints = Constraints(numConstraints=['placement/complement']))
    graph.addConstraints(edge='move_gripper_away', constraints = Constraints(numConstraints=[ 'placement/complement']))
    

    graph.initialize()
    
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return factory, graph
    
# Create locked joint for grippers
def createGripperLockedJoints (ps, q):
    left_gripper_lock = list()
    right_gripper_lock = list()
    for n in ps.robot.jointNames:
        s = ps.robot.getJointConfigSize(n)
        r = ps.robot.rankInConfiguration[n]
        if n.startswith("ur3e/robotiq_85_right"):
            ps.createLockedJoint(n, n, q[r : r + s])
            right_gripper_lock.append(n)
        elif n.startswith("ur3e/robotiq_85_left"):
            ps.createLockedJoint(n, n, q[r : r + s])
            left_gripper_lock.append(n)
    return left_gripper_lock, right_gripper_lock        

def test_node(node, graph, robot):
    for i in range(100):
        q = robot.shootRandomConfig()
        res, q1, err = graph.applyNodeConstraints(node, q)
        if res:
            res, err = robot.isConfigValid(q1)
            if res: break
    return res, q1, err    
    
def test_edge(edge, q_i, graph, robot):
    for i in range(100):
        q = robot.shootRandomConfig()
        res, q1, err = graph.generateTargetConfig(edge,q_i, q)
        #v(q1)
        if res:
            res, err = robot.isConfigValid(q1)
            if res: break
    return res, q1, err   




