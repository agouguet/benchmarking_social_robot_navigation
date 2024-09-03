import copy
import math
import torch
from mbsn.model.polygon.State import State
from mbsn.model.polygon.human import Human
from mbsn.model.trajectory_prediction.utils.models import PECNet
from mbsn.model.trajectory_prediction.trajectory_prediction_model import TrajectoryPredictionModel

from ament_index_python.packages import get_package_share_directory


class PECNETTrajectoryPrediction(TrajectoryPredictionModel):

    def __init__(self) -> None:
        super().__init__()
        
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

    def predict(self, state, action):
        if len(state.humans) == 0:
            return [(State(action, state.humans), 1.0)]
        
        pair_future_state_probabilities = []

        return pair_future_state_probabilities