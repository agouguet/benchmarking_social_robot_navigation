import matplotlib.pyplot as plt
from matplotlib import image
import json
import numpy as np

DWA = "DWA"
TEB = "TEB"
HATEB = "HATEB"
SARL = "SARL*"

def get_coord_x_and_y_robot(method):
    x = []
    y = []
    stp = []
    dictionary = json.load(open(method+"/json/door_passing.json", 'r'))
    for data in dictionary['data']:
        x.append(data['position']['x'])
        y.append(data['position']['y'])
        stp.append(data['secs'])
    return (x, y, stp)

def get_coord_x_and_y_humans(method):
    x = []
    y = []
    stp = []
    dictionary = json.load(open(method+"/json/door_passing_human_pos.json", 'r'))
    for data in dictionary['data']:
        x.append(data['human_position']['human_0']['position']['x'])
        y.append(data['human_position']['human_0']['position']['y'])
        stp.append(data['secs'])
    return (x, y, stp)


img = image.imread('./map/door_passing_map.jpg')

x_robot, y_robot, stp_robot = get_coord_x_and_y_robot(SARL)
stp_robot = np.array(stp_robot) - stp_robot[0]
x_human, y_human, stp_human = get_coord_x_and_y_humans(SARL)
stp_human = np.array(stp_human) - stp_human[0]

f = plt.figure()
f.set_figwidth(12)
f.set_figheight(2)
f.set_dpi(100)

plt.scatter(x_human, y_human, c = stp_human, label='Human', cmap='rainbow', s=1)
clb = plt.colorbar()
clb.set_label('Human Movement (s)')
plt.scatter(x_robot, y_robot, c = stp_robot, label='Robot', cmap='viridis', s=1)
clb = plt.colorbar()
clb.set_label('Robot Movement (s)')
plt.xlim(-12, 12)
plt.ylim(-1.6, 1.6)

plt.axis('off')
plt.imshow(img, aspect='auto', extent=(-12,12,-1.6,1.6), alpha=0.9, zorder=-1)
plt.savefig('result.png', dpi=300)
plt.show()
