import numpy as np
import matplotlib.pyplot as plt
from mbsn.model.graph.GraphState import GraphState


class ValueIteration:
    def __init__(self, problem, gamma):
        self.problem = problem
        self.gamma = gamma
        self.values = {}
        self.full_values = {}
        self.policy = None

    def one_iteration(self):
        delta = 0
        nb_state = 0
        for i in self.problem.states():
            s = GraphState(i[0], self.problem.goal_node, i[1])
            if nb_state % 1000 == 0:
                print(nb_state, s)
            temp = self.values[s] if s in self.values else 0
            v_list = dict.fromkeys(self.problem.action(s), 0)
            for a in self.problem.action(s):
                for s_prim in self.problem.states_prim(s, a):
                    p = self.problem.transition(s, a, s_prim)
                    # print(v_list, s, a, s_prim, p)
                    v_list[a] += self.problem.reward(s, a, s_prim) + self.gamma * np.sum(p * self.values[s_prim]) if s_prim in self.values else self.problem.reward(s, a, s_prim) 
            self.values[s] = max(v_list.values())
            self.full_values[s] = v_list
            delta = max(delta, abs(temp - self.values[s]))
            nb_state += 1
        
        return delta    

    def get_policy(self):
        pi = {}
        for i in self.problem.states():
            s = GraphState(i[0], self.problem.goal_node, i[1])
            v_list = dict.fromkeys(self.problem.action(s), 0)
            for a in self.problem.action(s):
                for s_prim in self.problem.states_prim(s, a):
                    p = self.problem.transition(s, a, s_prim)
                    v_list[a] += self.problem.reward(s, a, s_prim) + self.gamma * np.sum(p * self.values[s_prim]) if s_prim in self.values else self.problem.reward(s, a, s_prim) 

            max_index = []
            max_val = max(v_list.values())
            for a in self.problem.action(s):
                if v_list[a] == max_val:
                    max_index.append(a)
            pi[s] = np.random.choice(max_index)
        return pi

    def train(self, tol=1e-3, plot=False):
        epoch = 0
        delta = self.one_iteration()
        delta_history = [delta]
        while delta > tol:
            epoch += 1
            delta = self.one_iteration()
            delta_history.append(delta)
            print("Delta: {:.4f}".format(delta))
            if delta < tol:
                break
        self.policy = self.get_policy()

        if plot is True:
            fig, ax = plt.subplots(1, 1, figsize=(3, 2), dpi=200)
            ax.plot(np.arange(len(delta_history)) + 1, delta_history, marker='o', markersize=4,
                    alpha=0.7, color='#2ca02c', label=r'$\gamma= $' + f'{self.gamma}')
            ax.set_xlabel('Iteration')
            ax.set_ylabel('Delta')
            ax.legend()
            plt.tight_layout()
            plt.show()







