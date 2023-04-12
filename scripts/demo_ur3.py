from init_ur3 import robot, ps, v, ri, q_init, q_init_simu, q_calibLAAS, q_calibSN
from createConstraintGraph import createConstraintGraph, test_edge, test_node
from tools_hpp import RosInterface, PathGenerator
from hpp.gepetto import PathPlayer

#Get graph
factory, graph = createConstraintGraph()

#Set Init
#q_init = q_init_simu[::]
ps.setInitialConfig(q_init)
v(q_init)

#Set Goal
q_goal = q_init[::]
q_goal[6] = -0.1
ps.addGoalConfig(q_goal) #grasp

pp = PathPlayer(v)

pg = PathGenerator(ps, graph, ri, v, q_init)
pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()

q_calib = [-0.0314868132220667, -0.3098681608783167, -0.7921517531024378, -0.4463098684894007, -1.567646328602926, -0.0652702490435999, 0.1, -0.4, 1.009, 0.0, 0.0, 0.0, 1.0]
q_calib[6:] = q_init[6:]
#ps.solve()