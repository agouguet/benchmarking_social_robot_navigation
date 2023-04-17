# Benchmarking Off-the-shelf Social Robot Navigation Solutions

<p align="center">
    <img src="./assets/illustration.png">
</p>

The objective of this project is to use the [SEAN](https://sean.interactive-machines.com) simulator to benchmark off-the-shelf social robot navigation. Off-the-shelf means that we only compared methods that were already available, open source and without any external modifications or additions needed.
We have already made a comparison of different solutions with a somewhat modified version of the SEAN simulator, you will find below how to reproduce our experience.

 - [Dynamic Window Approach (DWA)](http://wiki.ros.org/dwa_local_planner). DWA is a collision avoidance strategy.
The objective is to find the optimal direction and velocity that brings the robot to the
goal without any collision.
 - [Timed Elastic Band (TEB)](http://wiki.ros.org/teb_local_planner). TEB uses the given path by the global planner
(like A*), thus it can create a more realistic path in local scope.
 - [Human-Aware Timed Elastic Band (HATEB-2)](https://github.com/sphanit/cohan_planner_multi). HATEB is a navigation
planner capable of planning cooperative trajectories. It offers co-navigation solu-
tions by jointly calculating the trajectories of humans and robots using TEB.
 - [Socially Attentive Reinforcement Learning* (SARL*)](https://github.com/LeeKeyu/sarl_star). SARL* model is
an improvement of SARL method, which is proposed to rethink pairwise interac-
tions with a self-attention mechanism and joint Human-Robot and Human-Human
interaction model using deep reinforcement learning framework.

## Experiments

We experimented with these 4 navigation modes through 4 typical scenarios encountered in everyday life, which are: crossing a corridor with static humans, overtaking a person, ending up in an intersection and a doorway.

We have calculated performance metrics and social metrics that we compare in the paper associated with this benchmark.
Additionally, we provide all the raw videos of all experiments, in the ```~/benchmark_social_navigation/social_sim_unity/recording ``` folder.

We can also represent robot and human paths like this (example of HATEB method on doorway scenario) :

<p align="center">
    <img src="./sim_ws/data/results/door_passing_all.png"  width="400" height="420">
</p>


## Installation

The experiments were conducted in Ubuntu 20.04 with ROS Noetic and Unity 2020.3.21f1.

```console
git clone https://github.com/agouguet/benchmarking_social_robot_navigation.git ~/benchmark_social_navigation
```

### ROS packages

The ```~/benchmark_social_navigation/sim_ws``` folder is the workspace containing the ROS packages. So we need to build all packages in this workspace, but before we need to install all necessary package.

```
cd ~/benchmark_social_navigation/sim_ws
./scripts/setup.sh
```

Thus we can build all package with ```catkin_make``` command.

```
cd ~/benchmark_social_navigation/sim_ws
catkin_make
```

Add source to your .bashrc file :

```
echo "source <path_to_sim_ws>/devel/setup.sh" > <path_to_your_home>/.bashrc
```

### Unity project

In Unity Hub, select "Open" and choose ```~/benchmark_social_navigation/social_sim_unity``` directory. You can now open project in unity.

### SARL* solution

In order to use the SARL* navigation method, which was used under Ubuntu 16 and ROS kinetic, we provide a docker image for ease of use:

```
docker pull agouguet/benchmark-social-navigation:sarl_star
```

## Usage

### In Unity

You have to choose the scenario you want in Unity, in the scenes folder  ```<UnityProject>/Assets/Scenes/Scenario/<Scenario wanted>``` and you can play the scene normally.

### In ROS

We recommend installing ```Tmux``` which is a terminal multiplexer and ```Tmuxinator``` which is a tool that allows you to easily manage tmux sessions using yaml files, we have provided different yaml files for different navigation methods in the folder ```~/benchmark_social_navigation/sim_ws/tmux/sean_<navigation method>```. For example, to use DWA method :

```
cd ~/benchmark_social_navigation/sim_ws/tmux/sean_dwa
tmuxinator
```

*Specification for SARL\*: You must use the ```~/benchmark_social_navigation/sim_ws/tmux/sean/.yaml``` file with Tmuxinator and manually launch the navigation node with the docker listed above.*



## Questions

If you have any questions, please contact adam.gouguet@imt-nord-europe.fr