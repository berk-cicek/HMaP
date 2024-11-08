import pathlib
import pickle
import robotic as ry
import time
from seqwaynet.dataset.dataset import ConfigDataset

cur_dir = pathlib.Path(__file__).resolve().parent

dataset = ConfigDataset(f"{cur_dir}/../test_Data", task="tunnel", return_type="image")
print(dataset[0])

with open(f"{cur_dir}/predictions.pkl", "rb") as file:
    predictions = pickle.load(file)

print(predictions)

pred_0 = predictions.iloc[0]

# waypoints = []

# for i in range(0, 396, 3):
#     waypoints.append(
#         [
#             pred_0[f"waypoints_{i}"],
#             pred_0[f"waypoints_{i + 1}"],
#             pred_0[f"waypoints_{i + 2}"],
#         ]
#     )


# config = ry.Config()

# config.addFile(f"{cur_dir}/../data/bolt/bolt_0/0_bolt.g")

# for i, waypoint in enumerate(waypoints):
#     marker = config.addFrame(f"waypoint_{i}", "world")
#     marker.setPosition(waypoint)
#     marker.setShape(type=ry.ST.marker, size=[0.05])

# config.view(pause=True)

for k in range(0, 4):
    # _, data, _ = dataset[k]
    data = predictions.iloc[k]

    config = ry.Config()
    print(f"{cur_dir}/../test_Data/tunnel/tunnel_tool_{k + 2048}/config_{k + 2048}.g")

    config.addFile(
        f"{cur_dir}/../test_Data/tunnel/tunnel_tool_{k + 2048}/config_{k + 2048}.g"
    )

    # print(dataset[0])

    waypoints = []
    print("data len", len(data))
    for i in range(0, len(data), 3):
        print(data[i])
        waypoints.append(
            [
                data[i].item(),
                data[i + 1].item(),
                data[i + 2].item(),
            ]
        )

    print(len(waypoints))
    config.view()
    time.sleep(2)

    for i, waypoint in enumerate(waypoints):
        marker = config.addFrame(f"waypoint_{i}", "world")
        marker.setPosition(waypoint)
        # marker.setQuaternion([-1, 0, 0, 0])
        marker.setShape(type=ry.ST.marker, size=[0.05])
        config.view(message=f"{i}/{len(waypoints)}")
        time.sleep(0.2)
    print(waypoints[0])
    print("rel", marker.getRelativePosition())
    config.view(pause=True)

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


def single_instance_visualization(
    self,
    checkpoint_filename,
):
    config = ry.Config()
    sample = self.test_dataset.get_instance(
        np.random.randint(0, len(self.test_dataset))
    )
    config.addFile(sample["config_path"])

    denorm_target, denorm_generated = self.evaluate(
        single_instance=sample,
        checkpoint_filename=checkpoint_filename,
    )
    denorm_target = denorm_target.squeeze(0)
    denorm_generated = denorm_generated.squeeze(0)

    for i, each in enumerate(denorm_generated):
        contact = each[:3]
        waypoint = each[3:]
        tmp_c = config.addFrame(f"contact_{i}", "world")
        tmp_c.setPosition(contact)
        tmp_c.setShape(type=ry.ST.marker, size=[0.05])
        tmp_w = config.addFrame(f"waypoint_{i}", "world")
        tmp_w.setPosition(waypoint)
    config.view(pause=True)
