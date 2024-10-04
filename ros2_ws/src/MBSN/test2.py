
import numpy as np 
import matplotlib.pyplot as plt 
  
N = 3
ind = np.arange(N)  
width = 0.25

ALPHA_SCORE = 0.9
ALPHA_GENERAL_SCORE = 0.8
  
xvals = [8, 9, 2] 
bar1 = plt.bar(ind, xvals, width, color = 'r', alpha=ALPHA_SCORE) 
  
yvals = [10, 20, 30] 
bar2 = plt.bar(ind+width, yvals, width, color='g', alpha=ALPHA_SCORE) 
  
zvals = [11, 12, 13] 
bar3 = plt.bar(ind+width*2, zvals, width, color = 'b', alpha=ALPHA_SCORE)

zvals = [9.5, 13, 14] 
bar4 = plt.bar(ind+width, zvals, width*3, color = 'orange', alpha=ALPHA_GENERAL_SCORE, zorder=-2) 
  
plt.xlabel("Dates")
plt.ylabel('Scores')
plt.title("Players Score")
  
plt.xticks(ind+width,['2021Feb01', '2021Feb02', '2021Feb03']) 
plt.legend( (bar1, bar2, bar3, bar4), ('Player1', 'Player2', 'Player3', "Mean") ) 
plt.show() 