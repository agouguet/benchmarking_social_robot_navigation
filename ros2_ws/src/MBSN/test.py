from mbsn.model.graph.MBSN import MBSN
import pickle
from mbsn.utils.visibility_graph import VisibilityGraph
from mbsn.solver.MCTS import MBSNAgentMCTS, MBSNAgentNode
from mbsn.solver.qtable import QTable
from mbsn.solver.multi_armed_bandit.ucb import UpperConfidenceBounds
from mbsn.utils.graph_visualisation import GraphVisualisation
from mbsn.model.graph.GraphState import GraphState
from mbsn.model.graph.GraphAction import GraphAction


scenario = "hospital"

G = pickle.load(open("../unity_sim/graphs/" + scenario + ".pickle", 'rb'))
visibility_graph = VisibilityGraph("../unity_sim/maps/"+ scenario + "/", G).visibility_graph

print(visibility_graph[0])

mdp = MBSN(G, visibility_graph, 4)
qfunction = QTable()


root_node = MBSNAgentMCTS(mdp, qfunction, UpperConfidenceBounds()).mcts(GraphState(2, 4, [0, 0, 0, 0, 0]), timeout=0.3)

# print(root_node.get_visits(), MBSNAgentNode.visits)



print(qfunction.get_q_value(GraphState(2, 4, [0, 0, 0, 0, 0]), 2))
print(qfunction.get_q_value(GraphState(2, 4, [0, 0, 0, 0, 0]), 1))

print(mdp.get_next_states(GraphState(2, 4, [0, 1, 0, 0, 0]), GraphAction(2)))

print("----------------------")

print(qfunction.save("test"))
print(qfunction.get_q_value(GraphState(2, 4, [0, 0, 0, 0, 0]), 0))

gv = GraphVisualisation(max_level=25)
graph = gv.single_agent_mcts_to_graph(root_node, filename="mcts")
graph.view()

print(qfunction.qtable)
# print(qfunction.get_q_value(GraphState(2, 4, [0, 0, 0, 0, 0]), GraphAction(1)))

print(qfunction.qtable[0])
print(qfunction.qtable[1])
for k, v in qfunction.qtable.items():
    print(k, v)


print(root_node.get_value())

# i = 0
# while 1:
#     a, v = root_node.get_value()
#     print("I=", i, a, v)
#     # root_node = root_node.children[action]
#     for (child, _) in root_node.children[a]:
#         a, v = root_node.get_value()
#         print("I=", i, a, v)
#         i+=1
#     break

# for s in mdp.get_states():
#     print(s)
#     for a in mdp.get_actions(s):
#         print("     ", a)
#         print("         ", qfunction.get_q_value(s, a))
#         for s_prim, p in mdp.get_transitions(s, a):
#             print("         ", s_prim, p, mdp.get_reward(s, a, s_prim), mdp.is_terminal(s))

# print("Ok")