import numpy as np
import matplotlib.pyplot as plt


class ValueIteration:
    def __init__(self, problem, gamma):
        self.problem = problem
        self.gamma = gamma
        self.values = dict.fromkeys(self.problem.states, 0)
        self.full_values = dict.fromkeys(self.problem.states, [])
        self.policy = None

    def one_iteration(self):
        delta = 0
        for s in self.problem.states:
            # print(s)
            temp = self.values[s]
            v_list = dict.fromkeys(self.problem.actions[s], 0)
            for a in self.problem.actions[s]:
                # print("     ", a)
                for s_prim in self.problem.states_prim[s][a]:
                    # print("         ", s_prim)
                    p = self.problem.transition(s, a, s_prim)
                    v_list[a] += self.problem.reward(s, a, s_prim) + self.gamma * np.sum(p * self.values[s_prim])
                    # print("         ", p, v_list[a], "         ", self.problem.reward(s, a, s_prim), np.sum(p * self.values[s_prim]))
            self.values[s] = max(v_list.values())
            self.full_values[s] = v_list
            delta = max(delta, abs(temp - self.values[s]))
        
        return delta

    def get_policy(self):
        pi = dict.fromkeys(self.problem.states, -1)
        for s in self.problem.states:
            v_list = dict.fromkeys(self.problem.actions[s], 0)
            for a in self.problem.actions[s]:
                for s_prim in self.problem.states_prim[s][a]:
                    p = self.problem.transition(s, a, s_prim)
                    v_list[a] += self.problem.reward(s, a, s_prim) + self.gamma * np.sum(p * self.values[s_prim])
                    if s.robot_node == 3 and s.humans_node == (6,) and s.goal == 12:
                        print(s, a, s_prim, self.problem.reward(s, a, s_prim), self.values[s_prim])
                        print(v_list)

            max_index = []
            max_val = max(v_list.values())
            for a in self.problem.actions[s]:
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
            print(delta)
            if delta < tol:
                break
        self.policy = self.get_policy()

        # print(f'# iterations of policy improvement: {len(delta_history)}')
        # print(f'delta = {delta_history}')

        if plot is True:
            fig, ax = plt.subplots(1, 1, figsize=(3, 2), dpi=200)
            ax.plot(np.arange(len(delta_history)) + 1, delta_history, marker='o', markersize=4,
                    alpha=0.7, color='#2ca02c', label=r'$\gamma= $' + f'{self.gamma}')
            ax.set_xlabel('Iteration')
            ax.set_ylabel('Delta')
            ax.legend()
            plt.tight_layout()
            plt.show()







