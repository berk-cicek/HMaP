import torch
import torchvision.transforms as T
from torch.utils.data import Dataset
from tqdm import tqdm
import os
import pickle
import json
from PIL import Image
import numpy as np


# class WaypointDataset(Dataset):
#     def __init__(
#         self,
#         root_dir,
#         relative_poses=False,
#         transform=T.ToTensor(),
#         device="cpu",
#         is_instance=False,
#     ):
#         self.root_dir = root_dir
#         self.device = device
#         self.transform = transform

#         self.info = None
#         self.rgb_paths = []
#         self.depth_paths = []
#         self.waypoints = []
#         self.contacts = []

#         if not is_instance:
#             # Load data: iterate through subdirectories to get class names and image paths
#             for class_name in tqdm(os.listdir(root_dir), desc="Loading data"):
#                 class_dir = os.path.join(root_dir, class_name)
#                 for scene_name in os.listdir(class_dir):
#                     scene_dir = os.path.join(class_dir, scene_name)
#                     if scene_name.endswith("info.pkl"):
#                         with open(scene_dir, "rb") as file:
#                             self.info = pickle.load(file)
#                     else:
#                         if os.path.isdir(scene_dir):
#                             rgb = []
#                             depth = []
#                             for data_name in sorted(os.listdir(scene_dir)):
#                                 data_path = scene_dir + "/" + data_name
#                                 # if data_name.endswith(".g"):
#                                 #     self.config_path.append(data_path)
#                                 if data_name.endswith("rgb.png"):
#                                     rgb.append(data_path)
#                                 if data_name.endswith("depth.png"):
#                                     depth.append(data_path)
#                                 if data_name.endswith("contactpoints.pkl"):
#                                     with open(data_path, "rb") as contact_file:
#                                         self.contacts.append(pickle.load(contact_file))
#                                 if data_name.endswith("waypoints.pkl"):
#                                     with open(data_path, "rb") as waypoint_file:
#                                         self.waypoints.append(
#                                             pickle.load(waypoint_file)
#                                         )
#                         self.rgb_paths.append(rgb)
#                         self.depth_paths.append(depth)

#     def _get_relative_transform(self, position, rotation):
#         pass

#     def __getitem__(self, idx):
#         rgbs = np.array([Image.open(img) for img in self.rgb_paths[idx]])
#         depths = np.array(
#             [Image.open(img).convert("L") for img in self.depth_paths[idx]]
#         )
#         waypoints = self.waypoints[idx]
#         contacts = self.contacts[idx]

#         return rgbs, depths, waypoints, contacts

#     def __len__(self):
#         return len(self.rgb_paths)
