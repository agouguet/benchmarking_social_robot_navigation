import os, yaml, cv2, numpy as np, matplotlib

from mbsn.utils.segmentation.py_floor_plan_segmenter.modules import do_segment
from .util import image_to_polygon, scale_list_vertices, scale_polygon
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('assets')
MAPS_DIRECTORY = package_share_directory+"/maps/"

script_dir = os.path.dirname(__file__)
rel_path = "segmentation/py_floor_plan_segmenter/custom.yml"
abs_file_path = os.path.join(script_dir, rel_path)

with open(abs_file_path) as f:
    config = yaml.load(f, Loader=yaml.SafeLoader)
    sigma_start = config["compute_labels_list"]["sigma_start"]
    sigma_step = config["compute_labels_list"]["sigma_step"]

def get_rank_map(map):
    gray_map = cv2.cvtColor(map.copy(), cv2.COLOR_BGR2GRAY)
    return np.float32(np.invert(gray_map).astype(float) / 255)

def get_map_config(scenario_name):
    if not os.path.exists(MAPS_DIRECTORY + scenario_name + "/map.yaml"):
        return False, None
    with open(MAPS_DIRECTORY + scenario_name + "/map.yaml", 'r') as file:
        map_config = yaml.safe_load(file)
        return True, map_config

def load_map(scenario_name):
    map_config_exist, map_config = get_map_config(scenario_name)
    
    if map_config_exist:
        if not os.path.exists(MAPS_DIRECTORY + scenario_name + "/" + map_config["image"]):
            return False, None
        map = cv2.imread(MAPS_DIRECTORY + scenario_name + "/" + map_config["image"])
        return True, map
    
    return False, None
    
def load_segmented_map(scenario_name):
    map_config_exist, map_config = get_map_config(scenario_name)
    
    if map_config_exist:
        if not os.path.exists(MAPS_DIRECTORY + scenario_name + "/segmented.png"):
            map_exist, map = load_map(scenario_name)
            if map_exist:
                rank_map = get_rank_map(map)
                segmented_map = do_segment(rank_map, **config)
                save_segmented_map(scenario_name, segmented_map)
                return True, segmented_map
            return False, None
        segmented_map = cv2.cvtColor(cv2.imread(MAPS_DIRECTORY + scenario_name + "/segmented.png"), cv2.COLOR_BGR2GRAY)
        return True, segmented_map
    
def save_segmented_map(scenario_name, segmented_map):
    cv2.imwrite(MAPS_DIRECTORY + scenario_name + "/segmented.png", segmented_map) # Save the image

def load_map_as_polygon(scenario_name, scale=True):
    map_config_exist, map_config = get_map_config(scenario_name)
    
    if map_config_exist:
        if not os.path.exists(MAPS_DIRECTORY + scenario_name + "/" + map_config["image"]):
            return False, None
        map = cv2.imread(MAPS_DIRECTORY + scenario_name + "/" + map_config["image"])
        vertices, polygon, inners = image_to_polygon(map)
        if scale:
            return scale_list_vertices(vertices, map_config), scale_polygon(polygon, map_config), inners
        return vertices, polygon, inners
    
def load_map_and_segmented_map(scenario_name):
    map_exist, map = load_map(scenario_name)
    segmented_map_exist, segmented_map = load_segmented_map(scenario_name)
    return map_exist and segmented_map_exist, map, segmented_map
        