import pickle
from seqwaynet.dataset.dataset import ConfigDataset
import pandas as pd
import pathlib
import numpy as np
import os
import robotic as ry
import time

task = "tunnel_tool"

# Load test dataset and inference results
cur_dir = pathlib.Path(__file__).resolve().parent

# dataset = ConfigDataset(f"{cur_dir}/../test_Data", task="tunnel", return_type="image")

with open(f"{cur_dir}/test_dataset.pkl", "rb") as file:
    dataset = pickle.load(file)
print(dataset[0])

with open(f"{cur_dir}/predictions.pkl", "rb") as file:
    predictions = pickle.load(file)

print(predictions)

pred_0 = predictions.iloc[0]


# Process inference data

config_to_episode = np.array(dataset.config_to_episode)
config_timesteps = np.array(dataset.config_timesteps)

episode_waypoints = []

for ep in range(len(dataset)):
    # _, data, _ = dataset[k]

    data_config_indices = np.where(config_to_episode == ep)[0].tolist()
    data_timesteps = config_timesteps[data_config_indices].tolist()

    sorted_config_timesteps, sorted_config_indices = zip(
        *sorted(zip(data_timesteps, data_config_indices))
    )
    sorted_config_indices = list(sorted_config_indices)
    sorted_config_timesteps = list(sorted_config_timesteps)
    # print(predictions)
    data = predictions.iloc[sorted_config_indices]

    cur_waypoints = []
    # print("data len", len(data))
    for i, row in data.iterrows():
        # print(i)
        cur_waypoints.append(row.tolist())  # 7d points

    episode_waypoints.append(cur_waypoints)


# Save processed data to csv
if not os.path.isdir(f"{cur_dir}/../output/{task}"):
    os.mkdir(f"{cur_dir}/../output/{task}")

output_dir = f"{cur_dir}/../output/{task}"

for i in range(len(episode_waypoints)):
    df = pd.DataFrame(
        episode_waypoints[i],
        columns=[f"next_waypoint_{j}" for j in range(7)]
        + [f"next_contact_{j}" for j in range(3)],
    )
    # print(df.head())

    df.to_csv(f"{output_dir}/tunnel_tool_{i}.csv")
