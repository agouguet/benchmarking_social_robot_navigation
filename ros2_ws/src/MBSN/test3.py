

from matplotlib import pyplot as plt

# t = 1.0
x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
y = [9.52, 5.51, 4.06, 2.45, 1.11, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# t = 5.0
# x = [0, 1, 2, 3, 4, 5]
# y = [35.35, 18.4, 4.06, 2.45, 1.11, 1.0]

# plt.figure(num=None, figsize=(4, 2), dpi=80, facecolor='w')

plt.figure().set_figheight(2)

plt.plot(x, y)


plt.xlabel("Number of humans in the state")
plt.ylabel('Number of rollout \n executed per decision')
plt.title("Scalability of MCTS")

plt.show()





