<<<<<<< HEAD
from init_ur3 import robot, ps, v, ri, q_init
=======
from init_ur3 import robot, ps, v, ri, q_init, q_init_simu, q_calibLAAS, q_calibSN
>>>>>>> e1ef91ccdd252f7603bcc3c28882b41d48d39d22
from createConstraintGraph import createConstraintGraph, test_edge, test_node
from tools_hpp import RosInterface, PathGenerator
from hpp.gepetto import PathPlayer

#Get graph
factory, graph = createConstraintGraph()

#Set Init
<<<<<<< HEAD
ps.setInitialConfig(q_init)
v(q_init)

i = 0
p = 0
#Test graph pas à pas
while(i<1):
    i +=1
    res, q1, err = test_edge('Loop | f', q_init, graph, robot)
    p = 1
    v(q1)
    if not res: continue
    res, q2, err = test_edge('ur3e/GRIPPER > kapla/handle | f', q_init, graph, robot)
    p = 2
    v(q2)
    if not res: continue
    res, q3, err = test_edge('Loop | 0-0', q2, graph, robot)
    p = 3
    v(q3)
    if not res: continue
    res, q4, err = test_edge('ur3e/GRIPPER < kapla/handle | 0-0', q3, graph, robot)
    p = 4
    v(q4)
    if not res: continue
print(res, err, p)


#Set Goal
q_goal = q_init[::]
q_goal[17] = -0.1
=======
#q_init = q_init_simu[::]
ps.setInitialConfig(q_init)
v(q_init)

#Set Goal
q_goal = q_init[::]
q_goal[6] = -0.1
>>>>>>> e1ef91ccdd252f7603bcc3c28882b41d48d39d22
ps.addGoalConfig(q_goal) #grasp

pp = PathPlayer(v)

pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()

<<<<<<< HEAD
=======
q_calib = [-0.0314868132220667, -0.3098681608783167, -0.7921517531024378, -0.4463098684894007, -1.567646328602926, -0.0652702490435999, 0.1, -0.4, 1.009, 0.0, 0.0, 0.0, 1.0]
q_calib[6:] = q_init[6:]
>>>>>>> e1ef91ccdd252f7603bcc3c28882b41d48d39d22
#ps.solve()