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
import robotic as ry
import pathlib


class ConfigDataset(Dataset):
    VALID_TASKS = ["bolt", "tunnel", "tunnel_tool"]

    def __init__(
        self,
        root_dir: str,
        task: str,  # "bolt" or "tunnel_tool"
        device: torch.device = "cpu",
        prediction_type: str = "single",  # "single" (next step prediction) or "full" (predict entire sequence)
        return_type: str = "vector",  # "vector" (vectorized config), "config" (raw config, not vectorized), or "image" (initial image, start and goal positions)
        multiview_images: bool = False,
    ):
        if task not in ConfigDataset.VALID_TASKS:
            raise AttributeError(
                f"Task {task} is not in valid tasks: {ConfigDataset.VALID_TASKS}"
            )

        self.root_dir = root_dir
        self.task = task
        self.device = device
        self.prediction_type = prediction_type
        self.return_type = return_type
        self.multiview_images = multiview_images

        self.image_paths = []
        self.image_shape = None
        self.shape_types = []
        self.initial_target = []
        self.goals = []

        self.info = None  # contains common info for the dataset such as camera poses, robot pose etc.
        self.configs = (
            []
        )  # N x self.config_size, if needed you can extract individual item features by using self.frame_size
        if self.prediction_type == "single":
            self.config_timesteps = []
            self.config_to_episode = []
        self.waypoints = []  # N x 3
        self.contacts = []  # N x 3

        # Load data
        if self.task == "bolt":
            self.load_bolt_data()
        elif self.task == "tunnel":
            self.load_tunnel_data()
        elif self.task == "tunnel_tool":
            self.load_tunnel_tool_data()

        # Process data
        if self.task == "bolt":
            self._process_bolt()
        elif self.task == "tunnel_tool":
            self._process_tunnel_tool()

        # Normalize mean image
        if self.return_type == "image":
            # print("image_paths", len(self.image_paths))
            self.mean_image = self.mean_image / len(self.image_paths)
            # print(self.mean_image.max)
            if self.mean_image.max() > 1:
                self.mean_image = self.mean_image / 255

    def __getitem__(self, idx):
        if self.return_type == "config":
            return self.configs[idx], self.waypoints[idx], self.contacts[idx]
        elif self.return_type == "image":
            img = np.asarray(Image.open(self.image_paths[idx]))
            if img.max() > 1:
                img = img / 255
            img = img - self.mean_image
            return img, self.waypoints[idx], self.contacts[idx]

    def __len__(self):
        try:
            assert (
                len(self.configs)
                if self.return_type == "vector"
                else len(self.image_paths)
                == len(self.image_paths)
                == len(self.contacts)
                == len(self.waypoints)
            ), "sample numbers don't match up"
        except:
            print(
                "sample numbers don't match up",
                len(self.configs) if self.return_type == "vector" else None,
                len(self.image_paths),
                len(self.contacts),
                len(self.waypoints),
            )
        return len(self.contacts)

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
        if self.task == "tunnel_tool":
            return self._tunnel_tool_to_autogluon()
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

            # Combine all dictionariesimport robotic as ry
            df = pd.DataFrame(
                {**config_transformed, **waypoint_transformed, **contact_transformed}
            )

        elif self.return_type == "image":
            # # Transform configs to configs_i
            # config_transformed = {
            #     f"configs_{i}": [x.item() for x in tensor_list]
            #     for i, tensor_list in enumerate(zip(*self.configs))
            # }
            image_path_transformed = {f"image_path": self.image_paths}

            initial_target_transformed = {
                f"initial_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.initial_target))
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
                    **initial_target_transformed,
                    **goal_transformed,
                    **waypoint_transformed,
                    **contact_transformed,
                }
            )

        return df

    def load_bolt_data(self):
        for class_name in tqdm(os.listdir(self.root_dir), desc="Loading data for bolt"):
            class_dir = os.path.join(self.root_dir, class_name)
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

                            if self.return_type == "image" and data_name.endswith(
                                ".png"
                            ):
                                img = np.asarray(Image.open(data_path))

                                if self.image_shape is None:
                                    self.image_shape = img.shape
                                    self.mean_image = np.zeros(self.image_shape)
                                else:
                                    self.mean_image += img

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

    def load_tunnel_data(self, preprocessed_configs: bool = True):
        class_dir = os.path.join(self.root_dir, self.task)
        print(class_dir)
        self.ep_to_dir_idx = []

        with open(pathlib.Path(class_dir) / "total_timesteps.csv") as file:
            self.episode_total_timesteps = (
                pd.read_csv(file).set_index("episode")["timesteps"].to_dict()
            )
            # print(self.episode_total_timesteps)

        for episode, scene_name in enumerate(
            tqdm(sorted(os.listdir(class_dir)), desc="Loading data for tunnel_tool")
        ):
            scene_dir = os.path.join(class_dir, scene_name)
            # print(scene_dir)
            if os.path.isdir(scene_dir):
                dir_idx = int(scene_name.replace("tunnel_", ""))
                self.ep_to_dir_idx.append(dir_idx)
                for data_name in sorted(os.listdir(scene_dir)):
                    data_path = scene_dir + "/" + data_name

                    if self.multiview_images:
                        current_sample_imgs = []

                    # Load images (if needed)
                    if self.return_type == "image" and data_name.endswith(".png"):
                        img = np.asarray(Image.open(data_path))

                        if "image" in data_name:
                            if self.image_shape is None:
                                self.image_shape = img.shape
                                self.mean_image = np.zeros(self.image_shape)
                            if self.multiview_images or data_name.endswith(
                                "angle_0_image.png"
                            ):
                                self.mean_image += img

                            if not self.multiview_images:
                                if data_name.endswith("angle_0_image.png"):
                                    self.image_paths.append(data_path)
                            else:
                                current_sample_imgs.append(data_path)

                    # Load RAI config (if needed) - currently not implemented except for bolt task
                    if not preprocessed_configs and data_name.endswith(".g"):
                        config_timestep = int(
                            data_name.replace("config_", "").replace(".g", "")
                        )

                        if (
                            config_timestep
                            != self.episode_total_timesteps[scene_name] - 1
                        ):  # skip last timestep, no need to predict there
                            C = ry.Config()
                            C.addFile(data_path)

                            config = self._read_config_objs(C)

                            for item in config:
                                if item["shape"] not in self.shape_types:
                                    self.shape_types.append(item["shape"])
                            self.configs.append(config)
                            if self.prediction_type == "single":
                                self.config_to_episode.append(episode - 1)
                                self.config_timesteps.append(config_timestep)

                    elif preprocessed_configs and data_name.endswith(".pkl"):
                        config_timestep = int(
                            data_name.replace("config_", "").replace(".pkl", "")
                        )

                        if (
                            config_timestep
                            != self.episode_total_timesteps[scene_name] - 1
                        ):  # skip last timestep, no need to predict there
                            with open(data_path, "rb") as config_file:
                                config = pickle.load(config_file)
                                for item in config:
                                    if item["shape"] not in self.shape_types:
                                        self.shape_types.append(item["shape"])
                                self.configs.append(config)
                                self.config_timesteps.append(config_timestep)
                                self.config_to_episode.append(episode - 1)

                    # Load contact/waypoints
                    if "contact_point" in data_name:
                        # print("here")
                        raw_contacts = pd.read_csv(
                            data_path, header=None, usecols=[0, 1, 2]
                        )
                        self.contacts.append(
                            ConfigDataset.vectorize_points(raw_contacts.to_numpy())
                        )
                        # print("contact", self.contacts)
                    if "waypoint" in data_name:
                        raw_waypoints = pd.read_csv(
                            data_path, header=None, usecols=[0, 1, 2, 3, 4, 5, 6]
                        )
                        self.waypoints.append(
                            ConfigDataset.vectorize_points(raw_waypoints.to_numpy())
                        )
            if self.multiview_images:
                self.image_paths.append(current_sample_imgs)

    def load_tunnel_tool_data(self, preprocessed_configs: bool = True):
        class_dir = os.path.join(self.root_dir, self.task)
        print(class_dir)
        self.ep_to_dir_idx = []

        with open(pathlib.Path(class_dir) / "total_timesteps.csv") as file:
            self.episode_total_timesteps = (
                pd.read_csv(file).set_index("episode")["timesteps"].to_dict()
            )
            # print(self.episode_total_timesteps)

        for episode, scene_name in enumerate(
            tqdm(sorted(os.listdir(class_dir)), desc="Loading data for tunnel_tool")
        ):
            scene_dir = os.path.join(class_dir, scene_name)
            # print(scene_dir)
            if os.path.isdir(scene_dir):
                dir_idx = int(scene_name.replace("tunnel_tool_", ""))
                self.ep_to_dir_idx.append(dir_idx)
                for data_name in sorted(os.listdir(scene_dir)):
                    data_path = scene_dir + "/" + data_name

                    if self.multiview_images:
                        current_sample_imgs = []

                    # Load images (if needed)
                    if self.return_type == "image" and data_name.endswith(".png"):
                        img = np.asarray(Image.open(data_path))

                        if "image" in data_name:
                            if self.image_shape is None:
                                self.image_shape = img.shape
                                self.mean_image = np.zeros(self.image_shape)
                            if self.multiview_images or data_name.endswith(
                                "angle_0_image.png"
                            ):
                                self.mean_image += img

                            if not self.multiview_images:
                                if data_name.endswith("angle_0_image.png"):
                                    self.image_paths.append(data_path)
                            else:
                                current_sample_imgs.append(data_path)

                    # Load RAI config (if needed) - currently not implemented except for bolt task
                    if not preprocessed_configs and data_name.endswith(".g"):
                        config_timestep = int(
                            data_name.replace("config_", "").replace(".g", "")
                        )

                        if (
                            config_timestep
                            != self.episode_total_timesteps[scene_name] - 1
                        ):  # skip last timestep, no need to predict there
                            C = ry.Config()
                            C.addFile(data_path)

                            config = self._read_config_objs(C)

                            for item in config:
                                if item["shape"] not in self.shape_types:
                                    self.shape_types.append(item["shape"])
                            self.configs.append(config)
                            if self.prediction_type == "single":
                                self.config_to_episode.append(episode - 1)
                                self.config_timesteps.append(config_timestep)

                    elif preprocessed_configs and data_name.endswith(".pkl"):
                        config_timestep = int(
                            data_name.replace("config_", "").replace(".pkl", "")
                        )

                        if (
                            config_timestep
                            != self.episode_total_timesteps[scene_name] - 1
                        ):  # skip last timestep, no need to predict there
                            with open(data_path, "rb") as config_file:
                                config = pickle.load(config_file)
                                for item in config:
                                    if item["shape"] not in self.shape_types:
                                        self.shape_types.append(item["shape"])
                                self.configs.append(config)
                                self.config_timesteps.append(config_timestep)
                                self.config_to_episode.append(episode - 1)

                    # Load contact/waypoints
                    if "contact_point" in data_name:
                        # print("here")
                        raw_contacts = pd.read_csv(
                            data_path, header=None, usecols=[0, 1, 2]
                        )
                        self.contacts.append(
                            ConfigDataset.vectorize_points(raw_contacts.to_numpy())
                        )
                        # print("contact", self.contacts)
                    if "waypoint" in data_name:
                        raw_waypoints = pd.read_csv(
                            data_path, header=None, usecols=[0, 1, 2, 3, 4, 5, 6]
                        )
                        self.waypoints.append(
                            ConfigDataset.vectorize_points(raw_waypoints.to_numpy())
                        )
            if self.multiview_images:
                self.image_paths.append(current_sample_imgs)

        # print(len(self.contacts))
        # raise Exception()

    def _process_bolt(self):
        # Process data
        if self.return_type == "vector":
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
                        self.initial_target.append(
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

    def _process_tunnel_tool(self):
        # Process data
        if self.return_type == "vector":
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
                if self.prediction_type == "single":
                    # self.goals.append(
                    #     self.waypoints[self.config_to_episode[i]][
                    #         3 * self.config_timesteps[i] : 3 * self.config_timesteps[i]
                    #         + 3
                    #     ]
                    # )
                    self.goals.append(self.waypoints[self.config_to_episode[i]][-3:])
                else:
                    self.goals.append(self.waypoints[i][-3:])

                for item in self.configs[i]:
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

        # if self.mean_image.max() > 1:
        #     self.mean_image.max() = self.mean_image.max() / 255

    def _bolt_to_autogluon(self):
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

        elif self.return_type == "image":
            # # Transform configs to configs_i
            # config_transformed = {
            #     f"configs_{i}": [x.item() for x in tensor_list]
            #     for i, tensor_list in enumerate(zip(*self.configs))
            # }

            if not self.multiview_images:
                image_path_transformed = {f"image_path": self.image_paths}
            else:
                raise NotImplementedError()

            initial_target_transformed = {
                f"initial_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.initial_target))
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
                    **initial_target_transformed,
                    **goal_transformed,
                    **waypoint_transformed,
                    **contact_transformed,
                }
            )

            return df

    def _tunnel_tool_to_autogluon(self):
        print(
            f"Converting task {self.task} to dataframe for {self.return_type} ({self.prediction_type})"
        )
        if self.return_type == "vector":
            if self.prediction_type == "single":
                print("test")
                config_transformed = {
                    f"configs_{i}": [x.item() for x in tensor_list]
                    for i, tensor_list in enumerate(zip(*self.configs))
                }

                # Transform waypoints to waypoints_i
                waypoint_transformed = {
                    f"waypoints_{i}": [
                        self.waypoints[self.config_to_episode[j]][7 * t + i].item()
                        for j, t in enumerate(self.config_timesteps)
                    ]
                    for i in range(7)
                }

                # Transform waypoints to waypoints_i
                contact_transformed = {
                    f"contacts_{i}": [
                        self.waypoints[self.config_to_episode[j]][3 * t + i].item()
                        for j, t in enumerate(self.config_timesteps)
                    ]
                    for i in range(3)
                }

                step_transformed = {"step": self.config_timesteps}

                next_waypoint_transformed = {
                    f"next_waypoints_{i}": [
                        self.waypoints[self.config_to_episode[j]][
                            7 * (t + 1) + i
                        ].item()
                        for j, t in enumerate(self.config_timesteps)
                    ]
                    for i in range(7)
                }

                # Transform waypoints to waypoints_i
                next_contact_transformed = {
                    f"next_contacts_{i}": [
                        self.contacts[self.config_to_episode[j]][3 * (t + 1) + i].item()
                        for j, t in enumerate(self.config_timesteps)
                    ]
                    for i in range(3)
                }

                waypoint_displacement_transformed = {
                    f"waypoint_displacement_{i}": [
                        self.waypoints[self.config_to_episode[j]][
                            7 * (t + 1) + i
                        ].item()
                        - self.waypoints[self.config_to_episode[j]][7 * t + i].item()
                        for j, t in enumerate(self.config_timesteps)
                    ]
                    for i in range(7)
                }

                contact_displacement_transformed = {
                    f"contact_displacement_{i}": [
                        self.contacts[self.config_to_episode[j]][3 * (t + 1) + i].item()
                        - self.contacts[self.config_to_episode[j]][3 * t + i].item()
                        for j, t in enumerate(self.config_timesteps)
                    ]
                    for i in range(3)
                }

                goal_transformed = {
                    f"goal_{i}": [x.item() for x in tensor_list]
                    for i, tensor_list in enumerate(zip(*self.goals))
                }

                df = pd.DataFrame(
                    {
                        **config_transformed,
                        **goal_transformed,
                        **step_transformed,
                        **waypoint_transformed,
                        **contact_transformed,
                        **next_waypoint_transformed,
                        **next_contact_transformed,
                        **waypoint_displacement_transformed,
                        **contact_displacement_transformed,
                    }
                )

                # data = []

                # for i in range(len(self.configs)):
                #     # Current config and timestep
                #     config = self.configs[i]
                #     timestep = self.config_timesteps[i]

                #     # print(config)
                #     # Get the corresponding index for waypoints and contacts
                #     episode_index = self.config_to_episode[i]
                #     raise Exception()
                #     # Extract the waypoints and contacts
                #     # waypoint = self.waypoints[index]  # This will be a list of 7 vectors
                #     # contact = self.contacts[index]  # This will be a list of 3 vectors

                #     # Flatten the waypoints and contacts lists
                #     flattened_waypoints = [
                #         item for sublist in waypoint for item in sublist
                #     ]
                #     flattened_contacts = [
                #         item for sublist in contact for item in sublist
                #     ]

                #     # Combine all elements for the row
                #     row = (
                #         list(config)
                #         + [timestep]
                #         + flattened_waypoints
                #         + flattened_contacts
                #     )
                #     data.append(row)

                # # Create column names
                # config_cols = [
                #     f"config_{i}" for i in range(len(configs[0]))
                # ]  # One column for each element in the config
                # waypoint_cols = [
                #     f"waypoint_{i}" for i in range(7)
                # ]  # 7 waypoint columns
                # contact_cols = [f"contact_{i}" for i in range(3)]  # 3 contact columns
                # columns = config_cols + ["timestep"] + waypoint_cols + contact_cols

                # # Create the DataFrame
                # df = pd.DataFrame(data, columns=columns)

            else:  # predict entire sequence
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
                    {
                        **config_transformed,
                        **waypoint_transformed,
                        **contact_transformed,
                    }
                )

        elif self.return_type == "image":
            # # Transform configs to configs_i
            # config_transformed = {
            #     f"configs_{i}": [x.item() for x in tensor_list]
            #     for i, tensor_list in enumerate(zip(*self.configs))
            # }

            if not self.multiview_images:
                image_path_transformed = {f"image_path": self.image_paths}
            else:
                raise NotImplementedError()

            initial_target_transformed = {
                f"initial_{i}": [x.item() for x in tensor_list]
                for i, tensor_list in enumerate(zip(*self.initial_target))
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
                    **initial_target_transformed,
                    **goal_transformed,
                    **waypoint_transformed,
                    **contact_transformed,
                }
            )

        return df

    def get_sorted_config_indices(self, episode_idx):
        config_to_episode = np.array(self.config_to_episode)
        config_timesteps = np.array(self.config_timesteps)
        data_config_indices = np.where(config_to_episode == episode_idx)[0].tolist()
        data_timesteps = config_timesteps[data_config_indices].tolist()

        sorted_config_timesteps, sorted_config_indices = zip(
            *sorted(zip(data_timesteps, data_config_indices))
        )
        sorted_config_indices = list(sorted_config_indices)
        sorted_config_timesteps = list(sorted_config_timesteps)

        return sorted_config_indices

    def _read_config_objs(self, C: ry.Config):
        frames = C.getFrameNames()
        objs = []
        for frame_name in frames:
            obj = C.getFrame(frame_name)
            obj_info = obj.info()

            if (
                "shape" in obj_info.keys()
                and obj_info["shape"] != "marker"
                # and "panda" not in obj_info["name"]
                and "table" not in obj_info["name"]
                and "contact" not in obj_info["name"]
                and "wp" not in obj_info["name"]
                and "waypoint" not in obj_info["name"]
            ):
                # print(obj_info)
                objs.append(
                    {
                        "name": frame_name,
                        "pos": obj.getPosition(),
                        "quat": obj.getQuaternion(),
                        "size": obj.getSize(),
                        "shape": obj_info["shape"],
                    }
                )

        return objs
