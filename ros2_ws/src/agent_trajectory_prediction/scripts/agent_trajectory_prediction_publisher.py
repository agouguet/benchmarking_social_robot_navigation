#!/usr/bin/env python3
from collections import defaultdict, deque
import time
import numpy as np
import torch
from agent_trajectory_prediction.utils.models import PECNet
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from agents_msgs.msg import AgentArray, AgentTrajectories, AgentTrajectory

from ament_index_python.packages import get_package_share_directory

TIME_INTERVAL = 1.0 # seconds
PREDICT_RATE = 0.1 # seconds

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

class AgentTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('AgentTrajectoryPublisher')

        package_share_directory = get_package_share_directory('agent_trajectory_prediction')

        dtype = torch.float64
        torch.set_default_dtype(dtype)
        self.device = torch.device('cuda', index=0) if torch.cuda.is_available() else torch.device('cpu')
        if torch.cuda.is_available():
            torch.cuda.set_device(0)

        checkpoint = torch.load(package_share_directory+'/agent_trajectory_prediction/saved_models/{}'.format("PECNET_social_model.pt"), map_location=self.device, weights_only=True)
        hyper_params = self.hyper_params = checkpoint["hyper_params"]
        self.prediction_model = PECNet(hyper_params["enc_past_size"], 
                                       hyper_params["enc_dest_size"], 
                                       hyper_params["enc_latent_size"],  
                                       hyper_params["dec_size"], 
                                       hyper_params["predictor_hidden_size"], 
                                       hyper_params['non_local_theta_size'], 
                                       hyper_params['non_local_phi_size'], 
                                       hyper_params['non_local_g_size'],
                                       hyper_params["fdim"], 
                                       hyper_params["zdim"], 
                                       hyper_params["nonlocal_pools"], 
                                       hyper_params['non_local_dim'], 
                                       hyper_params["sigma"], 
                                       hyper_params["past_length"], 
                                       hyper_params["future_length"],
                                       False)
        self.prediction_model = self.prediction_model.double().to(self.device)
        self.prediction_model.load_state_dict(checkpoint["model_state_dict"])
        self.prediction_model.eval()

        self.agents_subscription_ = self.create_subscription(AgentArray, 'social_sim/agents', self.agents_callback, 10)
        self.agents_trajectories_publisher_ = self.create_publisher(AgentTrajectories, 'agent/trajectories', 10)

        self.human_trajectories = defaultdict(lambda: deque(maxlen=5))
        self.human_last_time_trajectory = defaultdict(int)

        self.published_human_trajectories = deque(maxlen=3)
        self.last_time_predict = time.time()


    def predict(self, ids, trajectories, masks, positions, best_of_n=100):
        with torch.no_grad():
            for i, (id, traj, mask, initial_pos) in enumerate(zip(ids, trajectories, masks, positions)):
                traj, mask, initial_pos_torch = torch.DoubleTensor(traj).to(self.device), torch.DoubleTensor(mask).to(self.device), torch.DoubleTensor(initial_pos).to(self.device)

                x = traj[:, :self.hyper_params["past_length"], :]
                y = traj[:, self.hyper_params["past_length"]:, :]
                y = y.cpu().numpy()
                # reshape the data
                x = x.contiguous().view(-1, x.shape[1]*x.shape[2])
                x = x.to(self.device)

                dest = initial_pos #y[:, -1, :]
                all_l2_errors_dest = []
                all_guesses = []
                for index in range(best_of_n):
                    dest_recon = self.prediction_model.forward(x, initial_pos_torch, device=self.device)
                    dest_recon = dest_recon.cpu().numpy()
                    all_guesses.append(dest_recon)

                    l2error_sample = np.linalg.norm(dest_recon - dest, axis = 1)
                    all_l2_errors_dest.append(l2error_sample)

                all_l2_errors_dest = np.array(all_l2_errors_dest)
                all_guesses = np.array(all_guesses)

                # choosing the best guess
                indices = np.argmin(all_l2_errors_dest, axis = 0)

                best_guess_dest = all_guesses[indices,np.arange(x.shape[0]),  :]

                # back to torch land
                best_guess_dest = torch.DoubleTensor(best_guess_dest).to(self.device)

                # using the best guess for interpolation
                interpolated_future = self.prediction_model.predict(x, best_guess_dest, mask, initial_pos_torch)
                interpolated_future = interpolated_future.cpu().numpy()
                best_guess_dest = best_guess_dest.cpu().numpy()

                # final overall prediction
                predicted_future = np.concatenate((interpolated_future, best_guess_dest), axis = 1)
                predicted_future = np.reshape(predicted_future, (-1, self.hyper_params["future_length"], 2))

        predicted_future_traj = {}
        for i in range(len(predicted_future)):
            predicted_future_traj[ids[i]] = predicted_future[i] / 1000

        return predicted_future_traj


    def agents_callback(self, msg_agents):

        agents_pos = {}
        for agent in msg_agents.agents:
            agents_pos[agent.id] = agent.pose

        agents_visibility = {}
        for agent in msg_agents.agents:
            agents_visibility[agent.id] = agent.visible_by_robot

        # Add new position to trajectories
        for agent in msg_agents.agents:
            if(agent.visible_by_robot):
                if self.human_last_time_trajectory[agent.id] + TIME_INTERVAL <= time.time():
                    self.human_last_time_trajectory[agent.id] = time.time()
                    self.human_trajectories[agent.id].append([agent.pose.position.x, agent.pose.position.y])

        if self.last_time_predict + PREDICT_RATE <= time.time():
            self.last_time_predict = time.time()
            trajectories=[]
            masks = [[]]
            initial_pos = []
            ids = []

            for id, queue in self.human_trajectories.items():
                traj = np.round(np.array(queue) * 1000)
                if len(traj) >= self.hyper_params["past_length"]:
                    trajectories.append(traj)
                    masks[0].append([0])
                    initial_pos.append(queue[-1])
                    ids.append(id)

            trajectories = np.array([trajectories])
            initial_pos = np.array([initial_pos])

            if len(trajectories[0]) == 0:
                return

            for traj in trajectories:
                traj -= traj[:, :1, :]


            predicted_future = self.predict(ids, trajectories, masks, initial_pos)

            self.published_human_trajectories.append(predicted_future)

            trajectories_msg = AgentTrajectories()
            trajectories = []

            for id, _ in predicted_future.items():
                if agents_visibility[id]:
                    trajectory = AgentTrajectory()
                    trajectory.header =  Header()
                    trajectory.header.stamp = self.get_clock().now().to_msg()
                    trajectory.id = id
                    poses = []
                    
                    three_last_traj = np.array([traj for d in self.published_human_trajectories for id2, traj in d.items() if id2 == id])
                    mean_traj = np.mean(three_last_traj, axis=0)

                    for i in range(0, len(mean_traj), 1):
                        p = mean_traj[i]
                        pose = Point()
                        pose.x = agents_pos[id].position.x + p[0]
                        pose.y = agents_pos[id].position.y + p[1]
                        poses.append(pose)
                    trajectory.poses = poses
                    trajectories.append(trajectory)

            trajectories_msg.header =  Header()
            trajectories_msg.header.stamp = self.get_clock().now().to_msg()
            trajectories_msg.trajectories = trajectories
            self.agents_trajectories_publisher_.publish(trajectories_msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = AgentTrajectoryPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()