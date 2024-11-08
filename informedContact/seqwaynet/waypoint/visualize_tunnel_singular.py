import pathlib
import pickle
import robotic as ry
import time
from seqwaynet.dataset.dataset import ConfigDataset
import numpy as np

cur_dir = pathlib.Path(__file__).resolve().parent

# dataset = ConfigDataset(f"{cur_dir}/../test_Data", task="tunnel", return_type="image")

with open(f"{cur_dir}/test_dataset.pkl", "rb") as file:
    dataset = pickle.load(file)
print(dataset[0])

with open(f"{cur_dir}/predictions.pkl", "rb") as file:
    predictions = pickle.load(file)

print(predictions)

pred_0 = predictions.iloc[0]

test_path = f"{cur_dir}/../data/test/tunnel"
config_to_episode = np.array(dataset.config_to_episode)
config_timesteps = np.array(dataset.config_timesteps)
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

    waypoints = []
    print("data len", len(data))
    for i, row in data.iterrows():
        # print(i)
        waypoints.append(row.tolist())  # 7d points

    # print(data)
    C = ry.Config()
    f = C.addFrame("cam_frame")
    f.setPose("[-0.4, 0.1, 1, 0.0007963, -1, 0, 0]")
    f.setAttribute("focalLength", 0.5)  # wide angle
    f.setAttribute("width", 600)  # super wide angle
    f.setAttribute("height", 600)  # super wide angle
    C.view_setCamera(f)
    print(f"{test_path}/tunnel_tool_{ep}/config_{0}.g")
    C.addFile(f"{test_path}/tunnel_tool_{ep}/config_{0}.g")

    # C.view(pause=True, message=f"episode {ep} {0}/{len(waypoints)}")

    # for t, i in enumerate(sorted_config_indices):
    #     print(t, i, dataset.waypoints[ep])
    #     marker = C.addFrame(f"marker_{t}")
    #     marker.setShape(ry.ST.marker, size=[0.05])
    #     tmp = dataset.waypoints[ep].tolist()
    #     marker.setPosition(dataset.waypoints[ep][7 * t : 7 * t + 3])
    #     marker.setQuaternion(dataset.waypoints[ep][7 * t + 3 : 7 * t + 7])
    #     C.view(message=f"episode {ep} {i}/{len(waypoints)}")
    #     time.sleep(0.05)

    C.view(pause=True, message=f"episode {ep} {0}/{len(waypoints)}")

    for t, waypoint in enumerate(waypoints):
        # marker = C.addFrame(f"waypoint_{i}", "world")
        box = C.getFrame("box")
        box.setPosition(waypoint[:3])
        box.setQuaternion(waypoint[3:7])

        obstacle = C.getFrame("obstacle")
        C_t = ry.Config()
        C_t.addFile(f"{test_path}/tunnel_tool_{ep}/config_{t}.g")
        obstacle_t = C_t.getFrame("obstacle")
        print(f"{ep}, {t}, {obstacle_t.getPosition()}")
        obstacle.setPosition(obstacle_t.getPosition())
        obstacle.setQuaternion(obstacle_t.getQuaternion())
        # marker.setShape(type=ry.ST.marker, size=[0.05])
        C.view(message=f"episode {ep} {t}/{len(waypoints)}")
        time.sleep(0.02)
    C.view(pause=True, message=f"episode {ep} {t}/{len(waypoints)}")

with open(f"{cur_dir}/evaluations.pkl", "rb") as file:
    predictions = pickle.load(file)

print(predictions)

waypoint_sum_ae = 0
waypoint_count = 0
contact_sum_ae = 0
contact_count = 0
for k, v in predictions.items():
    if "waypoint" in k:
        waypoint_count += 1
        waypoint_sum_ae += v["mean_absolute_error"]
    elif "contact" in k:
        contact_count += 1
        contact_sum_ae += v["mean_absolute_error"]

print(waypoint_sum_ae / waypoint_count)
