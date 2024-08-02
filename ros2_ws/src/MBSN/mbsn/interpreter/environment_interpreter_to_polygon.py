from shapely import Point
from mbsn.interpreter.environment_interpreter import EnvironmentInterpreter
from mbsn.model.polygon.NavPolygonMDP import NavPolygonMDP
from mbsn.model.polygon.NavRoomByVisibilityWithHumanMDP import NavRoomByVisibilityWithHumanMDP
from mbsn.model.polygon.State import State
from mbsn.polygon.nav_map import NavMap
from mbsn.solver.ValueIteration import ValueIteration
from mbsn.utils.util import astar


class EnvironmentInterpreterToPolygon(EnvironmentInterpreter):

    def __init__(self, name_node='EnvironmentInterpreterToPolygon'):
        super().__init__(name_node)

        self.map_polygon = None

        self.global_policy = None
        self.global_state = None

    def scene_info_callback(self, msg_scene_info):
        if self.scenario != msg_scene_info.environment.lower():
            self.scenario = msg_scene_info.environment.lower()

            self.map_polygon = NavMap(self.scenario)
            rooms = self.map_polygon.rooms
            self.global_mdp = NavPolygonMDP(rooms)
            self.global_solver = ValueIteration(self.global_mdp, 0.99)

    def goal_callback(self, msg_goal):
        pos = msg_goal.pose.position
        if self.global_mdp is not None and self.goal != pos:
            self.goal = pos
            self.global_mdp.goal = self.goal
            self.global_solver.train()
            self.global_policy = self.global_solver.policy

    def robot_odom_callback(self, msg_odom):
        if self.global_policy is not None:
            robot_position = msg_odom.pose.pose.position

            global_state = self.global_mdp.get_state_from_continuous_position(robot_position)

            if global_state != self.global_state:
                self.global_state = global_state
                self.global_state_updated()

            self.local_state_updated()
            
        pass

    def global_state_updated(self):
        self.global_action = self.global_policy[self.global_state]

    def local_state_updated(self):
        
        self.local_mdp = NavRoomByVisibilityWithHumanMDP(self.map_polygon, self.current_pos, social_factor=5000.0)

        # A* to find the local goal
        grid = self.map_polygon.grid
        current_state = get_cell_id_in_dict_from_continuous_position(grid, self.current_pos)
        goal_state = get_cell_id_in_dict_from_continuous_position(grid, self.global_goal)
        astar_path = astar(current_state, goal_state, grid)
        local_goal_id = astar_path[0]
        for cell_id in astar_path:
            if cell_id in self.local_mdp.polygons:
                local_goal_id = cell_id
        self.local_goal_id = local_goal_id
        self.local_mdp.goal = self.local_mdp.polygons[local_goal_id][0].centroid

        # Human
        occupied_polygon = {poly_id:0 for poly_id in self.local_mdp.polygons.keys()}
        for h in self.humans:
            human_state = self.local_mdp.get_state_from_continuous_position(h.position)
            if human_state is not None:
                occupied_polygon[human_state] = 1


        local_state = State(self.local_mdp.get_state_from_continuous_position(self.current_pos), occupied_polygon)

        # print(self.map_polygon.grid[155].neighbors)

        print(local_state)
        # for a in self.local_mdp.get_actions(local_state):
        #     print("    ", a)
        #     for (new_state, probability) in self.local_mdp.get_transitions(local_state, a):
        #         print("        ", new_state, probability, self.local_mdp.get_reward(local_state, a, new_state))


        # time.sleep(20)

        # MCTS
        # self.local_solver = MBSNAgentMCTS(self.local_mdp, self.local_qfunction, self.local_bandit)
        
        # root_node, _ = self.local_solver.mcts(local_state, timeout=0.5)
        # self.local_action, _ = root_node.get_value()
        # self.robot_path.append(self.local_action)

        # print(self.local_solver.qfunction.qtable)

        # for (s, a), v in self.local_solver.qfunction.qtable.items():
        #     print(s, a, v)

    def humans_position_callback(self, msg_position):
        pass

    def humans_callback(self, msg_agents):
        pass
    
    def update(self, new_state):
        pass

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.buffer(0.1).contains(position):
                return id
    return None