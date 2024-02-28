import matplotlib.pyplot as plt
from matplotlib import image
import matplotlib
import matplotlib.animation as animation
import json
import numpy as np
import argparse
import sys
import os

# CMAPS
cmaps = ["Reds", "Blues", "Greens", "Purples", "Oranges", "cool", "hot"]

# METHOD
DWA = "DWA"
TEB = "TEB"
HATEB = "HATEB"
SARL = "SARL*"

# SCENARIO
DOOR = "door_passing"
INTERSECTION = "intersection"
OVERTAKING = "overtaking"
STATIC = "static"
CORNER = "corner"
FRONTAL = "frontal"

# TASK
LEAD = "lead"
FOLLOW = "follow"

DIMENSION = {
#   SCENARIO: [-X, X, -Y, Y]
    DOOR: [-12, 12, -12, 12],
    INTERSECTION: [-12, 12, -12, 12],
    OVERTAKING: [-12, 12, -12, 12],
    STATIC: [-12, 12, -12, 12],
    FRONTAL: [-12, 12, -12, 12],
}

FIG_DIMENSION = {
#   SCENARIO: [WIDTH, HEIGTH]
    DOOR: [16, 10],
    INTERSECTION: [16, 10],
    OVERTAKING: [16, 10],
    STATIC: [16, 10],
    FRONTAL: [16, 10],
}

SUBPLOT_DIMENSION = {
    DOOR: [-12, 12, -12, 12],
    INTERSECTION: [-12, 12, -12, 12],
    OVERTAKING: [-12, 12, -12, 12],
    STATIC: [-12, 12, -12, 12],
    FRONTAL: [-12, 12, -12, 12],
}

X_LIMIT_DIMENSION = {
    DOOR: [-12,12],
    INTERSECTION: [-12,12],
    OVERTAKING: [-12,12],
    STATIC: [-12,12],
    CORNER: [-12,3],
    FRONTAL: [-12,12],
}

Y_LIMIT_DIMENSION = {
    DOOR: [-1.5,1.5],
    INTERSECTION: [-12,12],
    OVERTAKING: [-1.5,1.5],
    STATIC: [-1.5,1.5],
    CORNER: [-1.5,11],
    FRONTAL: [-1.5,1.5],
}

def get_indices_of_duplicates_elements(arr):
    return {v: np.where(arr == v)[0] for v in np.unique(arr)}

def get_coord_x_and_y_robot(method, scenario):
    x = []
    y = []
    stp = []
    path_file = "data/"+method+"/json/"+scenario+".json"
    try:
        dictionary = json.load(open(path_file, 'r'))
        for data in dictionary['data']:
            x.append(data['position']['x'])
            y.append(data['position']['y'])
            stp.append(data['secs'])
        return (x, y, stp)
    except OSError as e:
        print(f"Unable to open {path_file}: {e}", file=sys.stderr)
        return

def get_coord_x_and_y_human(method, scenario, human_number):
    x = []
    y = []
    stp = []
    visible = []
    path_file = "data/"+method+"/json/"+scenario+"_human_pos.json"
    try:
        dictionary = json.load(open(path_file, 'r'))
        for data in dictionary['data']:
            if data['human_position'] != {}:
                x.append(data['human_position']['human_'+str(human_number)]['position']['x'])
                y.append(data['human_position']['human_'+str(human_number)]['position']['y'])
                visible.append(data['human_position']['human_'+str(human_number)]['visible_by_robot'])
            elif data['human_position'] == {}:
                x.append(-100)
                y.append(-100)
                visible.append(False)
            stp.append(data['secs'])
            
        return (x, y, stp, visible)
    except OSError as e:
        print(f"Unable to open {path_file}: {e}", file=sys.stderr)
        return

def get_human_number(method, scenario):
    path_file = "data/"+method+"/json/"+scenario+"_human_pos.json"
    try:
        dictionary = json.load(open(path_file, 'r'))
        max_human = 0
        for d in dictionary['data']:
            if len(d['human_position']) > max_human:
                max_human = len(d['human_position'])
        return max_human
    except OSError as e:
        print(f"Unable to open {path_file}: {e}", file=sys.stderr)
        return 1

def path(method, scenario, task, show=False, animate=False):
    from mpl_toolkits.axes_grid1 import make_axes_locatable

    scenario_and_task = scenario
    if task != "":
        scenario_and_task = task + "_" +  scenario

    path_map_file = 'map/'+scenario+'_map.jpg'

    try:
        img = image.imread(path_map_file)
    except OSError as e:
        print(f"Unable to open {path_map_file}: {e}", file=sys.stderr)
        return

    #plt.figure(figsize=(1.80, 0.2), dpi=1000)
    
    x_humans = []
    y_humans = []
    stp_humans = []
    visible_humans = []

    for i in range(get_human_number(method, scenario_and_task)-1, -1, -1):
        x_human, y_human, stp_human, visible_human = get_coord_x_and_y_human(method, scenario_and_task, i)
        stp_human = np.array(stp_human) - stp_human[0]
        x_humans.append(x_human)
        y_humans.append(y_human)
        stp_humans.append(stp_human)
        visible_humans.append(visible_human)

    x_robot, y_robot, stp_robot = get_coord_x_and_y_robot(method, scenario_and_task)
    stp_robot = np.array(stp_robot) - stp_robot[0]

    if animate :

        f = plt.figure()
        f.set_figwidth(14)
        f.set_figheight(8)
        f.set_dpi(300)
        ax = plt.gca()
        divider = make_axes_locatable(ax)
        
        y_subplot = 2
        ratio = [50,1]
        x_robot = np.array(x_robot)
        y_robot = np.array(y_robot)

        for i in range(len(x_humans)):
            x_humans[i] = np.array(x_humans[i])
            y_humans[i] = np.array(y_humans[i])
            y_subplot += 1
            ratio.append(1)

        # axtext = fig.add_axes([0.0,0.84,0.27,0.05])
        time = ax.text(-11,1.4, str(0), ha="left", va="top")

        scatters = []
        cmaps_anim = []
        norms_anim = []

        norm = matplotlib.colors.Normalize(vmin=min(stp_robot), vmax=max(stp_robot))

        #ROBOT
        sc = ax.scatter([], [], s = 300, marker="o", c = [], label='Robot', cmap=cmaps[0], norm=norm)
        cax = divider.append_axes("right", size="2%", pad=0.5)
        clb = plt.colorbar(sc, cax=cax)
        clb.set_label('Robot Movement (s)')

        cmap = matplotlib.cm.get_cmap(cmaps[0])
        cmaps_anim.append(cmap)
        norms_anim.append(norm)

        scatters.append(sc)

        for i in range(len(x_humans)):
            stp_humans[i], indices = np.unique(stp_humans[i], return_index=True)
            x_humans[i] = x_humans[i][indices]
            y_humans[i] = y_humans[i][indices]

            cmap = matplotlib.cm.get_cmap(cmaps[i+1])
            cmaps_anim.append(cmap)
            norm = matplotlib.colors.Normalize(vmin=min(stp_humans[i]), vmax=max(stp_humans[i]))
            sc = ax.scatter([], [], s = 30, c = [], label='Human ' + str(i+1), cmap=cmaps[i+1], norm=norm)
            

            cax = divider.append_axes("right", size="2%", pad=0.7)
            clb = plt.colorbar(sc, cax=cax)
            clb.set_label('Human '+ str(i+1) +' Movement (s)')
            norms_anim.append(norm)
            scatters.append(sc)
        
        scatters.append(time)

        def init():
            ax.tick_params(left = False, right = False , labelleft = False , labelbottom = False, bottom = False)
            ax.set_xlim(-12,12)
            ax.set_ylim(-1.5,1.5)
            ax.axis("off")
            ax.imshow(img, extent=SUBPLOT_DIMENSION[scenario], alpha=0.9, zorder=-1)
            f.tight_layout()
            return scatters 
        
        def update(frame):
            print(frame)
            time.set_text("Time:" + str(frame) + "(s)")

            for i in range(len(x_humans)):
                im = ax.scatter(x_humans[i][:frame,np.newaxis], y_humans[i][:frame,np.newaxis], label='Human', s=30)
                im.set_color(cmaps_anim[i+1](norms_anim[i+1](stp_humans[i][:frame])))

            im = ax.scatter(x_robot[:frame,np.newaxis], y_robot[:frame,np.newaxis], label='Robot', s=30)
            im.set_color(cmaps_anim[0](norms_anim[0](stp_robot[:frame])))

            return []

        ani = animation.FuncAnimation(f, update, frames=np.arange(max(stp_robot)), interval=20, init_func=init, blit=True, repeat=False)

        # To save the animation using Pillow as a gif
        writer = animation.PillowWriter(fps=20)
        ani.save('results/paths/'+method+'/result_'+method+'_'+scenario_and_task+'.gif', writer=writer, dpi=300)

        # ani.save('results/paths/'+method+'/result_'+method+'_'+scenario_and_task+'.mp4', fps=20, extra_args=['-vcodec', 'libx264'])
        if(show):
            plt.show()
        plt.close()


    else:
        f = plt.figure()
        f.set_figwidth(14)
        f.set_figheight(8)
        ax = plt.gca()

        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="2%", pad=0.1)

        for i in range(len(x_humans)):
            alphas = np.array(np.array(visible_humans[i]) * 1, dtype=float)
            alphas[alphas == 0] = 0.1
            alphas = alphas * 10
            im = plt.scatter(x_humans[i], y_humans[i], c = stp_humans[i], label='Human', cmap=cmaps[i+1], s=0)
            clb = plt.colorbar(im, cax=cax)
            clb.set_label('Human '+ str(i+1) +' Movement (s)', size=9)
            im = ax.scatter(x_humans[i], y_humans[i], c = stp_humans[i] * alphas, label='Human', cmap=cmaps[i+1], s=alphas)

        cax = divider.append_axes("right", size="2%", pad=0.75)

        im = ax.scatter(x_robot, y_robot, c = stp_robot, label='Robot', cmap=cmaps[0], s=10)
        clb = plt.colorbar(im, cax=cax)
        clb.set_label('Robot Movement (s)', size=9)

        ax.imshow(img, extent=[-12, 12, -12, 12], alpha=0.9, zorder=-1)
        ax.set_xlim(X_LIMIT_DIMENSION[scenario][0], X_LIMIT_DIMENSION[scenario][1])
        ax.set_ylim(Y_LIMIT_DIMENSION[scenario][0], Y_LIMIT_DIMENSION[scenario][1])
        ax.axis('off')
        if(show):
            plt.show()
        f.savefig('results/paths/'+method+'/result_'+method+'_'+scenario_and_task+'.png', dpi=300)
        
        plt.close()

def all_method(scenario, task, show=True):
    fig, axs = plt.subplots(3, 1)
    m = 0
    for method in [HATEB]:
        scenario_and_task = scenario
        if task != "":
            scenario_and_task = task + "_" +  scenario

        path_map_file = 'map/'+scenario+'_map.jpg'

        try:
            img = image.imread(path_map_file)
        except OSError as e:
            print(f"Unable to open {path_map_file}: {e}", file=sys.stderr)
            return
        
        fig.suptitle(scenario_and_task + " scenario with "+ method +" navigation method")
        fig.set_figwidth(10)
        fig.set_figheight(20)
        fig.subplots_adjust(wspace=0, hspace=0)
        #fig.set_dpi(200)

        for i in range(get_human_number(method, scenario_and_task)-1, -1, -1):
            x_human, y_human, stp_human = get_coord_x_and_y_human(method, scenario_and_task, i)
            stp_human = np.array(stp_human) - stp_human[0]

            axs[m].scatter(x_human, y_human, c = stp_human, label='Human', cmap=cmaps[i+1], s=1)

        x_robot, y_robot, stp_robot = get_coord_x_and_y_robot(method, scenario_and_task)
        stp_robot = np.array(stp_robot) - stp_robot[0]

        axs[m].scatter(x_robot, y_robot, c = stp_robot, label='Robot', cmap=cmaps[0], s=1)
        axs[m].set_xlim(DIMENSION[scenario][0], DIMENSION[scenario][1])
        axs[m].set_ylim(DIMENSION[scenario][2], DIMENSION[scenario][3])
        axs[m].axis('off')
        axs[m].set_title(method)

        axs[m].imshow(img, aspect='auto', extent=DIMENSION[scenario], alpha=0.9, zorder=-1)
        
        m += 1
    
    if(show):
        plt.tight_layout()
        plt.show()
    fig.savefig('results/paths/result_all_'+scenario_and_task+'.png', dpi=300)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Show paths data given by social sim unity.')
    parser.add_argument('--method', help="name of robot navigation method.", required=False)
    parser.add_argument('--scenario', help="name of scenario/map.", required=False)
    parser.add_argument('--task', help="name of robot task.", default="", required=False)
    parser.add_argument('--all', default=False, action="store_true", help="Allow to analyze all files.")
    parser.add_argument('--show', action="store_true", help="Show figure.")
    parser.add_argument('--animate', action="store_true", help="Show animation figure.")
    args = parser.parse_args()


    if(args.all):
        for method in [HATEB]:
            for scenario in [DOOR]:
                for task in [""]:
                    print("Create path image for", method+"_"+scenario+"_"+task+"...", end = '')
                    path(method, scenario, task, args.show, args.animate)
                    print("Done.")
    else:
        #all_method(args.scenario, args.task)
        path(args.method, args.scenario, args.task, args.show, args.animate)


