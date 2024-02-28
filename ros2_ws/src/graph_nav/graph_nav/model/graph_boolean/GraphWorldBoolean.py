import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from time import time
from graph_nav.model.interfaces.IWorld import IWorld
from graph_nav.model.graph_boolean.GraphActionBoolean import GraphActionBoolean
from graph_nav.model.graph_boolean.GraphStateBoolean import GraphStateBoolean
from scipy.spatial import distance
from itertools import combinations 

PENALITY_DISTANCE_GOAL = 0.8
PENALITY_COLLISION_HUMAN = 1000


class GraphWorldBoolean(IWorld):
    def __init__(self, networkx_graph, human_number, time_limit=1000):
        self.graph = networkx_graph
        self.human_number = human_number
        self.global_goal = 12
        self.time_limit = time_limit
        self.init()

    # |S| = N * 2^N
    def get_state_model(self):
        S = []
        
        numbers = self.graph.nodes()
        humans_presence_possibilities = []

        for count in range(0, self.human_number): # O(h)
            comb = combinations(numbers, count)   # O(n! / c! / (n-c)!)
            for t in comb:
                humans_presence_possibilities.append(t)

        for n in self.graph.nodes():
            for human_presence in humans_presence_possibilities:
                S.append(GraphStateBoolean(n, human_presence))

        print(len(S)) # |S| = sum()

        return S

    def get_action_model(self):
        A = {}
        for s in self.states:
            n = s.robot_node
            actions = []
            if not self.graph.nodes[n]['is_terminal']:
                for e in self.graph.edges([n]):
                    actions.append(GraphActionBoolean(e[1]))
            else:
                actions.append(GraphActionBoolean(n))
            A[s] = actions
        return A

    def get_state_from_pos(self, robot_node, human_node):
        for s in self.states:
            if s.robot_node == robot_node and s.human_node == human_node:
                return s
        return None

    def get_state_prim_model(self):
        S_prim = {}
        for s in self.states:
            S_prim[s] = {}
            for a in self.actions[s]:
                S_prim[s][a] = [GraphStateBoolean(a._node, s.humans_node)]
                #S_prim[s][a] = [self.get_state_from_pos(a._node, self.human_path[s.human_node])]
                #S_prim[s][a] = [self.get_state_from_pos(a._node, self.human_path[self.human_path.index(s.human_node) + 1]) if s.human_node != self.human_path[-1] else self.get_state_from_pos(a._node, self.human_path[-1])]
        return S_prim


    def get_reward_function(self):
        R = {}
        for s in self.states:
            R[s] = {}
            for a in self.actions[s]:
                R[s][a] = {}
                for s_prim in self.states_prim[s][a]:

                    R[s][a][s_prim] = self.graph.nodes[s_prim.robot_node]['weight'] - self.graph[s.robot_node][s_prim.robot_node]['weight'] - self.euclidean_distance_between_node(s.robot_node, s_prim.robot_node) - PENALITY_DISTANCE_GOAL * self.euclidean_distance_between_node(s_prim.robot_node, self.global_goal)
                    if s_prim.robot_node in s_prim.humans_node or s_prim.robot_node in s.humans_node:
                        R[s][a][s_prim] -= PENALITY_COLLISION_HUMAN
                    if s == GraphStateBoolean(0, ()):
                        print(s, a, s_prim)
                        print(R[s][a][s_prim])
                        print("    ", self.graph.nodes[s_prim.robot_node]['weight'], self.graph[s.robot_node][s_prim.robot_node]['weight'], self.euclidean_distance_between_node(s.robot_node, s_prim.robot_node), PENALITY_DISTANCE_GOAL * self.euclidean_distance_between_node(s_prim.robot_node, self.global_goal))
                            
        return R

    def get_transition_model(self):
        transitions = {}
        for s in self.states:
            transitions[s] = {}
            for a in self.actions[s]:
                transitions[s][a] = {}
                for s_prim in self.states_prim[s][a]:   
                    transitions[s][a][s_prim] = 1
        return transitions

    def get_pos_of_node(self, node):
        return self.graph.nodes(data=True)[node]['pos']

    def euclidean_distance_between_node(self, node1, node2):
        return distance.euclidean(self.get_pos_of_node(node1), self.get_pos_of_node(node2))

    def visualize_value_policy(self, policy, values, start_robot, start_human, fig_size=(24, 8)):
        fig, ax = plt.subplots(1, 1, figsize=fig_size)
        ax.axis('off')

        for n in self.graph.nodes(data=True):
            data = n[1]
            plt.scatter(data['pos'][0] , data['pos'][1] , s = 2000, c = "cornflowerblue")

        for n in self.graph.nodes():
            for e in self.graph.edges([n]):
                n1_pos = (self.graph.nodes(data=True)[e[0]]['pos'][0], self.graph.nodes(data=True)[e[0]]['pos'][1])
                n2_pos = (self.graph.nodes(data=True)[e[1]]['pos'][0], self.graph.nodes(data=True)[e[1]]['pos'][1])
                plt.plot([n1_pos[0], n2_pos[0]], [n1_pos[1], n2_pos[1]], '--', c = "black")

        s = self.get_state_from_pos(start_robot, start_human)
        total_reward = 0
        
        while s.robot_node != 24:
            a = policy[s]
            s_prim = self.states_prim[s][a][0]
            r = self.rewards[s][a][s_prim]
            total_reward += r

            x_s = self.graph.nodes(data=True)[s.robot_node]['pos'][0]
            y_s = self.graph.nodes(data=True)[s.robot_node]['pos'][1]

            x_s_prim = self.graph.nodes(data=True)[s_prim.robot_node]['pos'][0]
            y_s_prim = self.graph.nodes(data=True)[s_prim.robot_node]['pos'][1]

            

            dx = x_s_prim - x_s - 0.8
            dy = y_s_prim - y_s

            plt.plot([x_s, x_s_prim], [y_s, y_s_prim], color='red')

            if x_s == x_s_prim and y_s == y_s_prim:
                plt.text(x_s - 0.15, y_s + 0.1, "wait")

            s = s_prim 

        print("-----")
        print("Total reward:", total_reward)

            


        plt.tight_layout()
        plt.show()


























































    def generate_random_policy(self):
        return np.random.randint(self.num_actions, size=self.num_states)

    def execute_policy(self, policy, start_pos):
        s = self.get_state_from_pos(start_pos[0], start_pos[1])
        r = 0
        total_reward = r

        start_time = int(round(time() * 1000))
        overtime = False

        while r != self.reward[1] and r != self.reward[2]:
            s = np.random.choice(self.num_states, p=self.transition_model[s, policy[s]])
            r = self.reward_function[s]
            total_reward += r
            cur_time = int(round(time() * 1000)) - start_time
            if cur_time > self.time_limit:
                overtime = True
                break
        if overtime is True:
            print("Overtime !")
            return float('-inf')
        else:
            return total_reward

    def random_start_policy(self, policy, start_pos, n=100, plot=True):
        start_time = int(round(time() * 1000))
        overtime = False
        scores = np.zeros(n)
        i = 0
        while i < n:
            temp = self.execute_policy(policy=policy, start_pos=start_pos)
            print(f'i = {i} Random start result: {temp}')
            if temp > float('-inf'):
                scores[i] = temp
                i += 1
            cur_time = int(round(time() * 1000)) - start_time
            if cur_time > n * self.time_limit:
                overtime = True
                break

        print(f'max = {np.max(scores)}')
        print(f'min = {np.min(scores)}')
        print(f'mean = {np.mean(scores)}')
        print(f'std = {np.std(scores)}')

        if overtime is False and plot is True:
            bins = 100
            fig, ax = plt.subplots(1, 1, figsize=(6, 4), dpi=300)
            ax.set_xlabel('Total rewards in a single game')
            ax.set_ylabel('Frequency')
            ax.hist(scores, bins=bins, color='#1f77b4', edgecolor='black')
            plt.show()

        if overtime is True:
            print('Overtime!')
            return None
        else:
            return np.max(scores), np.min(scores), np.mean(scores)

    def blackbox_move(self, s, a):
        temp = self.transition_model[s, a]
        s_prime = np.random.choice(self.num_states, p=temp)
        r = self.reward_function[s_prime]
        return s_prime, r