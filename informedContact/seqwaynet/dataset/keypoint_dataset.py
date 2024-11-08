import os
import torch
import pandas as pd
from torch.utils.data import Dataset
import re
import robotic as ry
from tqdm import tqdm

import os
import torch
import json


class KeypointDataset(Dataset):
    def __init__(self, dataset_root, stats_file="stats.json"):
        """
        Args:
            dataset_root (string): Root directory where all the keypoint folders and preprocessed configuration files are located.
            stats_file (string): Filename to store or load the mean and std values.
        """
        self.dataset_root = dataset_root
        self.folders = sorted(
            os.listdir(dataset_root), key=self._numerical_sort_key
        )  # List of "keypoints_0", "keypoints_1", ...

        self.stats_file = (
            os.path.join(dataset_root, stats_file) if stats_file is not None else None
        )

        # Load all data from disk during initialization
        self.data = self._load_all_data()

        if stats_file is not None:
            # Load or compute mean and std
            self.config_mean, self.config_std, self.contact_mean, self.contact_std = (
                self._load_or_compute_stats()
            )
        else:
            self.config_mean, self.config_std, self.contact_mean, self.contact_std = (
                None,
                None,
                None,
                None,
            )

    def _load_or_compute_stats(self):
        """Load mean and std from file, or compute them if not available."""
        if os.path.exists(self.stats_file):
            # Load mean and std from file
            with open(self.stats_file, "r") as f:
                stats = json.load(f)
            config_mean = torch.tensor(stats["config_mean"])
            config_std = torch.tensor(stats["config_std"])
            contact_mean = torch.tensor(stats["contact_mean"])
            contact_std = torch.tensor(stats["contact_std"])
            print(f"Loaded mean and std from {self.stats_file}")
        else:
            # Compute mean and std
            all_configs = []
            all_contacts = []

            for data in tqdm(self.data, desc="Computing data mean and std:"):
                config, contact_current, _, contact_next, _, _ = data
                all_configs.append(config)
                all_contacts.append(contact_current)
                all_contacts.append(contact_next)

            all_configs = torch.stack(all_configs)
            all_contacts = torch.stack(all_contacts)

            config_mean = all_configs.mean(dim=0)
            config_std = all_configs.std(dim=0)
            contact_mean = all_contacts.mean(dim=0)
            contact_std = all_contacts.std(dim=0)

            # Replace NaNs in std with 1
            config_std[config_std == 0] = 1.0
            contact_std[contact_std == 0] = 1.0

            # Save to file
            stats = {
                "config_mean": config_mean.tolist(),
                "config_std": config_std.tolist(),
                "contact_mean": contact_mean.tolist(),
                "contact_std": contact_std.tolist(),
            }
            with open(self.stats_file, "w") as f:
                json.dump(stats, f)
            print(f"Computed and saved mean and std to {self.stats_file}")

        return config_mean, config_std, contact_mean, contact_std

    def _numerical_sort_key(self, filename):
        """Extract the integer from the filename for sorting"""
        match = re.search(r"\d+", filename)
        return int(match.group()) if match else -1

    def _load_all_data(self):
        """Load all preprocessed config and .csv keypoint file paths into a list for efficient indexing"""
        data = []
        for folder in tqdm(self.folders, desc="Loading all data:"):
            folder_path = os.path.join(self.dataset_root, folder)

            # Find matching config and keypoint CSV files
            config_files = sorted(
                [
                    f
                    for f in os.listdir(folder_path)
                    if f.startswith("config_") and f.endswith(".csv")
                ],
                key=self._numerical_sort_key,
            )
            contact_files = sorted(
                [
                    f
                    for f in os.listdir(folder_path)
                    if f.startswith("contacts_") and f.endswith(".csv")
                ],
                key=self._numerical_sort_key,
            )

            # Ensure there are equal numbers of config and keypoint files
            assert len(config_files) == len(
                contact_files
            ), f"Mismatched config and keypoint files in {folder} ({len(config_files)} vs {len(contact_files)})"

            for i in range(len(config_files) - 1):
                config_current_path = os.path.join(folder_path, config_files[i])
                contact_current_path = os.path.join(folder_path, contact_files[i])
                config_next_path = os.path.join(folder_path, config_files[i + 1])
                contact_next_path = os.path.join(folder_path, contact_files[i + 1])

                # Load the CSV data
                config_current = self._load_csv(config_current_path)
                contact_current = self._load_csv(contact_current_path)
                config_next = self._load_csv(config_next_path)
                contact_next = self._load_csv(contact_next_path)

                data.append(
                    (
                        config_current,
                        contact_current,
                        config_next,
                        contact_next,
                        config_current_path,
                        contact_current_path,
                    )
                )
        return data

    def _load_csv(self, csv_file_path):
        """Read the CSV file and return the content as a vector (torch.Tensor)"""
        data = pd.read_csv(csv_file_path, header=None).values[0]
        return torch.tensor(data, dtype=torch.float32).view(
            -1
        )  # Ensures it is always a 1D tensor

    def __len__(self):
        """Return the total number of samples in the dataset"""
        return len(self.data)

    def __getitem__(self, idx):
        """Return the normalized current config, current keypoint, and next keypoint"""
        (
            config_current,
            contact_gripper_current,
            config_next,
            contact_gripper_next,
            config_current_path,
            contact_current_path,
        ) = self.data[idx]

        contact_current = contact_gripper_current[:3]
        contact_next = contact_gripper_next[:3]

        if self.config_mean is not None and self.contact_mean is not None:
            # Extract and normalize the config and contact
            config_current = (config_current - self.config_mean) / self.config_std
            config_next = (config_next - self.config_mean) / self.config_std
            contact_current = (contact_current - self.contact_mean) / self.contact_std
            contact_next = (contact_next - self.contact_mean) / self.contact_std

        gripper_idx_current = contact_gripper_current[3]
        gripper_idx_next = contact_gripper_next[3]

        return (
            config_current,
            contact_current,
            gripper_idx_current,
            contact_next,
            gripper_idx_next,
            config_current_path,
            contact_current_path,
        )

    def get_episode_indices(self, episode_index):
        """Given an episode index, return the indices of the corresponding data points."""
        episode_folder = f"keypoints_{episode_index}"
        indices = [
            i
            for i, (
                config_current,
                contact_current,
                config_next,
                contact_next,
                _,
                _,
            ) in enumerate(self.data)
            if os.path.basename(os.path.dirname(config_current)) == episode_folder
        ]
        return indices

        # return torch.cat([config_current, keypoint_current[:6]]), keypoint_next[:6]


# class KeypointDataset(Dataset):
#     # TASK_PATH_PREFIXES = {"tunnel_tool": "tunnel_tool"}

#     def __init__(self, config_dataset: ConfigDataset):
#         self.task = config_dataset.task
#         self.root_dir = config_dataset.root_dir
#         self.config_paths = []
#         self.config_embs = []

#         self.valid_ep_idxs = []

#         self._config_dataset = config_dataset

#         num_episodes = len(config_dataset.episode_total_timesteps)
#         for ep in tqdm(range(num_episodes)):
#             sorted_config_indices = config_dataset.get_sorted_config_indices(ep)
#             ep_dir_idx = config_dataset.ep_to_dir_idx[ep]

#             #

#             self.config_paths.append(
#                 f"{self.root_dir}/{self.task}/{self.task}_{ep_dir_idx}"
#             )

#     def extract_keypoint_timesteps(self):
#         if self.task == "tunnel_tool":
#             return self._tunnel_tool_extract_keypoint_timesteps()
#         else:
#             raise NotImplementedError()

#     def _tunnel_tool_extract_keypoint_timesteps(self, write_to_disk: bool = False):
#         """
#         for tunnel tool there are no contact switches, so extract keypoints with heuristic
#         for tunnel tool a step is a keypoint step if:
#             - initial step
#             - object is out: height starts increasing for the first time (average height for a k step window centered at now is some epsilon higher than starting step)
#             - height peaks, i.e. average height for k steps before and after current step are both lower than some epsilon (there can be multiple peaks per trajectory)
#             - end step

#         peaks should be spaced out sufficiently
#         """
#         k = 5
#         min_steps = 10
#         min_steps_between_keypoints = 30

#         for ep in range(self.num_episodes()):
#             sorted_config_indices = self._config_dataset.get_sorted_config_indices(ep)
#             ep_dir_idx = self._config_dataset.ep_to_dir_idx[ep]
#             ep_num_steps = self._config_dataset.episode_total_timesteps[
#                 f"tunnel_{ep_dir_idx}"
#             ]
#             ep_dir_path = (
#                 f"{self.root_dir}/{self.task}/{self.task}_{ep_dir_idx}/config_{1}.g"
#             )

#             if ep_num_steps < min_steps:  # assume failed trajectory
#                 continue

#             keypoint_step_indices = []

#             # C = ry.Config()
#             # C.addFile(ep_dir_path)

#             # keypoint = C.getFrame("box").getPosition()
#             last_k_points = []
#             # last_k_movement_dirs = []
#             avg_movement_dir = np.zeros(3)

#             for t in range(ep_num_steps):
#                 # initial or end step
#                 if t == 0 or t == ep_num_steps - 1:
#                     keypoint_step_indices.append(t)
#                     continue
#                 elif (
#                     t - keypoint_step_indices[-1] < min_steps_between_keypoints
#                 ):  # make sure keypoints are spaced out
#                     continue

#                 # check for movement direction
#                 C = ry.Config()
#                 C.addFile(ep_dir_path)
#                 keypoint = C.getFrame("box").getPosition()

#                 if t < k:  # populate moving avg array
#                     if t > 0:
#                         cur_movement_dir = (
#                             np.array(keypoint) - np.array(last_k_points[0])
#                         ) / np.linalg.norm(
#                             np.array(keypoint) - np.array(last_k_points[0])
#                         )
#                         avg_movement_dir = (
#                             (t - 1) * avg_movement_dir + cur_movement_dir
#                         ) / t

#                     last_k_points.insert(0, keypoint)

#                 else:  # check movement direction
#                     cur_movement_dir = (
#                         np.array(keypoint) - np.array(last_k_points[0])
#                     ) / np.linalg.norm(np.array(keypoint) - np.array(last_k_points[0]))

#                     avg_movement_dir = (
#                         (k - 1) * avg_movement_dir + cur_movement_dir
#                     ) / k

#                     last_k_points.insert(0, keypoint)
#                     last_k_points.pop()

#             if write_to_disk:
#                 df = pd.DataFrame(keypoint_step_indices)
#                 df.to_csv(
#                     f"{self.root_dir}/{self.task}/{self.task}_{ep_dir_idx}/keypoint_step_indices.csv"
#                 )

#     def num_episodes(self):
#         return len(self._config_dataset.ep_to_dir_idx)
