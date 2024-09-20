import os
import torch
from torch.utils.data import Dataset
from torch.nn.utils.rnn import pad_sequence
import pickle
import numpy as np
from tqdm import tqdm
import pandas as pd

import seqwaynet.dataset.preprocess as preprocess
from seqwaynet.model.backbone.rgb_informed import RGBInformed
from typing import List
from PIL import Image


class ConfigDataset(Dataset):
    def __init__(
        self,
        root_dir: str,
        device: torch.device = "cpu",
        return_type: str = "vector",  # "vector", "config", or "image"
    ):
        self.root_dir = root_dir
        self.device = device
        self.return_type = return_type
        self.image_paths = []
        self.image_shape = None
        self.shape_types = []
        self.initial_head_tip = []
        self.goals = []

        self.info = None  # contains common info for the dataset such as camera poses, robot pose etc.
        self.configs = (
            []
        )  # N x self.config_size, if needed you can extract individual item features by using self.frame_size
        self.waypoints = []  # N x 3
        self.contacts = []  # N x 3

        # Load data
        for class_name in tqdm(os.listdir(root_dir), desc="Loading data"):
            class_dir = os.path.join(root_dir, class_name)
            for scene_name in sorted(os.listdir(class_dir)):
                scene_dir = os.path.join(class_dir, scene_name)
                if scene_name.endswith("info.pkl"):
                    with open(scene_dir, "rb") as file:
                        self.info = pickle.load(file)
                else:
                    if os.path.isdir(scene_dir):
                        for data_name in sorted(os.listdir(scene_dir)):
                            # print(data_name)
                            data_path = scene_dir + "/" + data_name
                            # if data_name.endswith(".g"):
                            #     self.config_path.append(data_path)

                            if return_type == "image" and data_name.endswith(".png"):
                                # img = np.asarray(Image.open(data_path))
                                # if self.image_shape is None:
                                #     self.image_shape = img.shape
                                #     self.mean_image = np.zeros(self.image_shape)
                                # else:
                                #     self.mean_image += img

                                if data_name.endswith("angle1_rgb.png"):
                                    self.image_paths.append(data_name)
                            if data_name.endswith("config.pkl"):
                                with open(data_path, "rb") as config_file:
                                    config = pickle.load(config_file)
                                    for item in config:
                                        if item["shape"] not in self.shape_types:
                                            self.shape_types.append(item["shape"])
                                    self.configs.append(config)
                            if data_name.endswith("contactpoints.pkl"):
                                with open(data_path, "rb") as contact_file:
                                    raw_contacts = pickle.load(contact_file)
                                self.contacts.append(
                                    ConfigDataset.vectorize_points(raw_contacts)
                                )
                            if data_name.endswith("waypoints.pkl"):
                                with open(data_path, "rb") as contact_file:
                                    raw_waypoints = pickle.load(contact_file)
                                self.waypoints.append(
                                    ConfigDataset.vectorize_points(raw_waypoints)
                                )

        print(len(self.waypoints))
        # Preprocess data
        if True:  # self.return_type == "vector":
            # print(self[0], self.configs[0], self.waypoints[0], self.contacts[0])

            item, _, _ = self.configs[0], self.waypoints[0], self.contacts[0]
            item = item[0]
            self.frame_size = (
                len(item["pos"])
                + len(item["quat"])
                + 4  # we make each object have same length size vectors
                + len(self._one_hot_shape(item["shape"]))
            )

            for i in range(len(self.configs)):
                vec = []

                # print(self.waypoints[i].shape)
                self.goals.append(self.waypoints[i][-3:])

                for item in self.configs[i]:

                    if "head_tip" in item["name"]:
                        self.initial_head_tip.append(
                            item["pos"] + np.random.normal(0, 0.00001, (3,))
                        )

                    vec.append(torch.tensor(item["pos"]))
                    vec.append(torch.tensor(item["quat"]))
                    # rai size vector length depends on shape, make them all equal length so every frame has equal length features
                    vec.append(
                        torch.cat(
                            [
                                torch.tensor(item["size"]),
                                torch.zeros((4 - len(item["size"]))),
                            ]
                        )
                    )
                    vec.append(torch.tensor(self._one_hot_shape(item["shape"])))

                self.configs[i] = torch.concat(vec, dim=0)

            self.config_size = self.configs[0].shape[0]
            # print("goals", len(self.goals), self.goals)

        elif self.return_type == "image":
            print("image_paths", len(self.image_paths))
            # self.mean_image = self.mean_image / len(self.image_paths)

    def __getitem__(self, idx, zero_mean=True):
        if self.return_type == "config":
            return self.configs[idx], self.waypoints[idx], self.contacts[idx]
        elif self.return_type == "image":
            img = np.asarray(Image.open(self.image_paths[idx]))
            img = img - self.mean_image
            return img, self.waypoints[idx], self.contacts[idx]

    def __len__(self):
        return len(self.configs)

    def _one_hot_shape(self, shape: str) -> List[int]:
        ret = [0] * len(self.shape_types)
        ret[self.shape_types.index(shape)] = 1
        return ret

    def vectorize_points(points):
        points = np.concatenate(points)
        points = torch.tensor(points)
        points = torch.flatten(points)
        return points

    def to_dataframe(self) -> pd.DataFrame:
        """Return the data in this dataset as a Pandas dataframe (not suitable for Autogluon)"""
        df = {
            "config": self.configs,
            "waypoints": self.waypoints,
            "contacts": self.contacts,
        }

        df = pd.DataFrame(df)

        return df

    def to_autogluon_dataframe(self) -> pd.DataFrame:
        if self.return_type == "vector":
            # Transform configs to configs_i
            config_transformed = {
                f"configs_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.configs))
            }

            # Transform waypoints to waypoints_i
            waypoint_transformed = {
                f"waypoints_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.waypoints))
            }

            # Transform contacts to contacts_i
            contact_transformed = {
                f"contacts_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.contacts))
            }

            # Combine all dictionaries
            df = pd.DataFrame(
                {**config_transformed, **waypoint_transformed, **contact_transformed}
            )

        if self.return_type == "image":
            # # Transform configs to configs_i
            # config_transformed = {
            #     f"configs_{i}": [x.item() for x in tensor_list]
            #     for i, tensor_list in enumerate(zip(*self.configs))
            # }

            image_path_transformed = {f"image_path": self.image_paths}

            initial_head_tip_transformed = {
                f"initial_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.initial_head_tip))
            }

            goal_transformed = {
                f"goal_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.goals))
            }

            # Transform waypoints to waypoints_i
            waypoint_transformed = {
                f"waypoints_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.waypoints))
            }

            # Transform contacts to contacts_i
            contact_transformed = {
                f"contacts_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.contacts))
            }

            # Combine all dictionaries
            df = pd.DataFrame(
                {
                    # **config_transformed,
                    **image_path_transformed,
                    **initial_head_tip_transformed,
                    **goal_transformed,
                    **waypoint_transformed,
                    **contact_transformed,
                }
            )

        return df


class RGBImageDataset(Dataset):
    def __init__(
        self,
        root_dir,
        backbone="None",
        device="cpu",
        is_instance=False,
    ):
        # Initialize data
        self.contacts = []
        self.waypoints = []
        self.image_paths = []
        self.trajectories = []
        self.image_features = []
        self.config_path = []
        self.trajectories_global_max = None
        self.trajectories_global_min = None
        self.images_global_max = None
        self.images_global_min = None
        self.start_token = np.full((1, 6), -1.0)
        self.end_token = np.full((1, 6), -2.0)
        self.device = device

        if not is_instance:
            # Load data: iterate through subdirectories to get class names and image paths
            for class_name in tqdm(os.listdir(root_dir), desc="Loading data"):
                class_dir = os.path.join(root_dir, class_name)
                for scene_name in os.listdir(class_dir):
                    scene_dir = os.path.join(class_dir, scene_name)
                    if os.path.isdir(scene_dir):
                        images = []
                        for data_name in sorted(os.listdir(scene_dir)):
                            data_path = scene_dir + "/" + data_name
                            if data_name.endswith(".g"):
                                self.config_path.append(data_path)
                            if data_name.endswith("rgb.png"):
                                images.append(data_path)
                            if data_name.endswith("contactpoints.pkl"):
                                with open(data_path, "rb") as contact_file:
                                    self.contacts.append(pickle.load(contact_file))
                            if data_name.endswith("waypoints.pkl"):
                                with open(data_path, "rb") as waypoint_file:
                                    self.waypoints.append(pickle.load(waypoint_file))
                        self.image_paths.append(images)

            if backbone == "None":
                self.find_globals()
            elif backbone == "resnet18":
                self.image_backbone = RGBInformed(device=device)
                self.find_globals(self.image_backbone)

    def get_instance(self, index=0):
        # Load and normalize theimage
        normalized_image_feature = self.normalize_images(self.image_features[index])

        # Randomly select a trajectory from the list of trajectories
        label_index = np.random.randint(0, len(self.contacts[index]))

        # Load and normalize the trajectory
        normalized_trajectory = self.normalize_trajectories(
            self.trajectories[index][label_index]
        )
        normalized_trajectory = self.add_start_end_tokens(normalized_trajectory)
        normalized_source = self.get_source_informed(normalized_trajectory)
        normalized_trajectory = torch.from_numpy(normalized_trajectory).float()
        normalized_source = torch.from_numpy(normalized_source).float()

        # Return single instance
        return {
            "config_path": self.config_path[index],
            "source": normalized_source,
            "image_features": normalized_image_feature,
            "target_seq": normalized_trajectory,
        }

    def find_globals(self, backbone=None):
        self.trajectories_global_min = np.full(6, np.inf)
        self.trajectories_global_max = np.full(6, -np.inf)
        for i in tqdm(
            range(len(self.contacts)), desc="Finding global min/max for trajectories"
        ):
            trajectories = []
            for j in range(len(self.contacts[i])):
                trajectory = np.concatenate(
                    (self.contacts[i][j], self.waypoints[i][j]), axis=1
                )
                trajectories.append(trajectory)

                traj_min = np.min(trajectory, axis=0)
                traj_max = np.max(trajectory, axis=0)

                self.trajectories_global_min = np.minimum(
                    self.trajectories_global_min, traj_min
                )
                self.trajectories_global_max = np.maximum(
                    self.trajectories_global_max, traj_max
                )
            self.trajectories.append(trajectories)
        if backbone is None:
            raise ValueError("This part of the code is not implemented yet")
        else:
            for each in tqdm(
                self.image_paths, desc="Finding global min/max for images"
            ):
                images = [preprocess.preprocess_image(image) for image in each]
                images_tensor = torch.stack(images, dim=0).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    feature = (
                        backbone.forward_features_multiple(
                            images_tensor[:, 0],
                            images_tensor[:, 1],
                            images_tensor[:, 2],
                        )
                        .view(-1)
                        .cpu()
                    )
                    self.image_features.append(feature)
            self.image_features = torch.stack(self.image_features, dim=0)
            self.images_global_min = self.image_features.min(dim=0)[0]
            self.images_global_max = self.image_features.max(dim=0)[0]

    def normalize_images(self, image_features):
        eps = 1e-6
        return (image_features - self.images_global_min) / (
            self.images_global_max - self.images_global_min + eps
        )

    def normalize_trajectories(self, trajectories):
        eps = 1e-6
        return (trajectories - self.trajectories_global_min) / (
            self.trajectories_global_max - self.trajectories_global_min + eps
        )

    def denormalize_images(self, image_features):
        return (
            image_features * (self.images_global_max - self.images_global_min)
            + self.images_global_min
        )

    def denormalize_trajectories(self, trajectories):
        return (
            trajectories * (self.trajectories_global_max - self.trajectories_global_min)
            + self.trajectories_global_min
        )

    def add_start_end_tokens(self, trajectory):
        return np.vstack([self.start_token, trajectory, self.end_token])

    def get_source_informed(self, trajectory):
        src = np.zeros((4, trajectory.shape[-1]))
        src[0, :] = trajectory[0, :]  # Start token
        src[1, :] = trajectory[1, :]  # Initial position
        src[2, :] = trajectory[-2, :]  # Final position
        src[3, :] = trajectory[-1, :]  # End
        return src

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        # Load and normalize images
        normalized_image_feature = self.normalize_images(self.image_features[idx])

        # Randomly select a trajectory from the list of trajectories
        label_index = np.random.randint(0, len(self.contacts[idx]))

        # Load and normalize trajectories
        normalized_trajectory = self.normalize_trajectories(
            self.trajectories[idx][label_index]
        )
        normalized_trajectory = self.add_start_end_tokens(normalized_trajectory)
        normalized_source = self.get_source_informed(normalized_trajectory)
        normalized_trajectory = torch.from_numpy(normalized_trajectory).float()
        normalized_source = torch.from_numpy(normalized_source).float()

        # Return images and trajectories
        return (
            normalized_source,
            normalized_image_feature,
            normalized_trajectory,
        )


def collate_fn_trajectories(batch):
    # Assuming each element in 'batch' is a tuple (image_features, trajectory)
    source, image_features, trajectories = zip(*batch)

    # Pad the sequences in the batch to have the same length
    trajectories_padded = pad_sequence(
        trajectories, batch_first=True, padding_value=-5.0
    )

    # Stack the image features to create a batch tensor
    source = torch.stack(source)
    image_features = torch.stack(image_features)

    return source, image_features, trajectories_padded


def collate_fn_cw(batch):
    # Separate images and labels
    images, contacts, waypoints = zip(*batch)

    # Assuming images are already tensors from __getitem__
    # Stack images to create a batch
    images = torch.stack(images)

    # Pad contacts and waypoints to the longest in the batch
    # pad_sequence stacks and pads sequences, set batch_first=True if you want batch dimension first
    contacts_padded = pad_sequence(contacts, batch_first=True, padding_value=0.0)
    waypoints_padded = pad_sequence(waypoints, batch_first=True, padding_value=0.0)

    return images, contacts_padded, waypoints_padded
