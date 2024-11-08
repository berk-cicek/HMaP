import robotic as ry
from seqwaynet.waypoint.autogluon_tunnel import MultilabelPredictor
from seqwaynet.dataset.dataset import ConfigDataset
from autogluon.tabular import TabularPredictor
import pathlib
import pickle
import pandas as pd
import time
import numpy as np

pd.options.mode.chained_assignment = None  # default='warn'

cur_dir = pathlib.Path(__file__).resolve().parent

model: MultilabelPredictor = MultilabelPredictor.load(
    f"{cur_dir}/../ag_models/tunnel_tool"
)

# with open(f"{cur_dir}/../output/pkls/train_dataset.pkl", "rb") as file:
#     train_dataset: ConfigDataset = pickle.load(file)

# print(train_dataset.to_autogluon_dataframe().head())

with open(f"{cur_dir}/../output/pkls/test_dataset.pkl", "rb") as file:
    test_dataset: ConfigDataset = pickle.load(file)
# print(dataset[0])

with open(f"{cur_dir}/../output/pkls/predictions.pkl", "rb") as file:
    predictions = pickle.load(file)


test_data = test_dataset.to_autogluon_dataframe()
for col in test_data.columns:
    print(col)
labels = [f"next_waypoints_{i}" for i in range(7)] + [
    f"next_contacts_{i}" for i in range(3)
]
test_data = test_data.drop(
    columns=labels
)  # unnecessary, just to demonstrate we're not cheating here

episode_waypoints = []

for ep in range(len(test_dataset)):
    sorted_config_indices = test_dataset.get_sorted_config_indices(ep)

    gt_waypoints = test_dataset.waypoints[ep].view(-1, 7).tolist()
    gt_contacts = test_dataset.contacts[ep].view(-1, 3).tolist()
    gt_goal = test_dataset.goals[ep]

    cur_dir_episode = test_dataset.config_to_dir_idx[ep]
    cur_waypoint = test_dataset.waypoints[ep][:7].tolist()  # first waypoint
    cur_contact = test_dataset.contacts[ep][:3].tolist()  # first contact

    print(f"Episode {ep} (dir idx: {cur_dir_episode})")

    cur_input = test_data.iloc[sorted_config_indices[0]]
    # print("before", cur_input[[f"waypoints_{i}" for i in range(7)]])
    # print(type(cur_input))
    cur_input[[f"waypoints_{i}" for i in range(7)]] = cur_waypoint
    cur_input[[f"contacts_{i}" for i in range(3)]] = cur_contact
    cur_input["step"] = 0

    waypoints = []
    contacts = []
    waypoints.append(cur_waypoint)
    contacts.append(cur_contact)
    # print("after", cur_input[[f"waypoints_{i}" for i in range(7)]])
    # print(f"Dataset idx {ep}, dir idx {cur_dir_episode}")

    for i in range(500):
        cur_df = cur_input.to_frame().T
        pred = model.predict(cur_input.to_frame().T, verbosity=0)
        pred_next_waypoint = pred.iloc[0][
            [f"next_waypoints_{i}" for i in range(7)]
        ].to_list()
        pred_next_contact = pred.iloc[0][
            [f"next_contacts_{i}" for i in range(3)]
        ].to_list()
        pred_next_waypoint_displacement = pred.iloc[0][
            [f"waypoint_displacement_{i}" for i in range(7)]
        ].to_list()
        pred_next_contact_displacement = pred.iloc[0][
            [f"contact_displacement_{i}" for i in range(3)]
        ].to_list()
        # cur_waypoint = pred.iloc[0][[f"next_waypoints_{i}" for i in range(7)]].to_list()
        # cur_contact = pred.iloc[0][[f"next_contacts_{i}" for i in range(3)]].to_list()
        cur_waypoint = [
            cur_waypoint[i] + pred_next_waypoint_displacement[i] for i in range(7)
        ]
        cur_contact = [
            cur_contact[i] + pred_next_contact_displacement[i] for i in range(3)
        ]
        print(i, cur_waypoint, pred_next_waypoint_displacement)

        waypoints.append(cur_waypoint)

        if (
            np.linalg.norm(np.array(cur_waypoint[:3]) - np.array(gt_goal[:3])
            < 0.5
        ):
            break
        # cur_waypoint_ball.setPosition(cur_waypoint[:3])
        # if i < len(gt_waypoints):
        #     gt_waypoint_ball.setPosition(gt_waypoints[i][:3])
        # time.sleep(0.05)
        cur_input[[f"waypoints_{i}" for i in range(7)]] = cur_waypoint
        cur_input[[f"contacts_{i}" for i in range(3)]] = cur_contact
        cur_input["step"] = i + 1
        # C.view(message=f"{i}/500 (max)")

    C = ry.Config()
    C.addFile(f"{cur_dir}/../data/test/tunnel/tunnel_tool_{cur_dir_episode}/config_0.g")
    f = C.addFrame("cam_frame")
    f.setPose("[-0.4, 0.1, 1, 0.0007963, -1, 0, 0]")
    f.setAttribute("focalLength", 0.5)  # wide angle
    f.setAttribute("width", 600)
    f.setAttribute("height", 600)
    C.view_setCamera(f)

    cur_waypoint_ball = C.addFrame("cur_waypoint")
    cur_waypoint_ball.setShape(ry.ST.sphere, size=[0.02])
    cur_waypoint_ball.setColor([1.0, 0.0, 0.0])
    cur_waypoint_ball.setPosition(cur_waypoint[:3])
    gt_waypoint_ball = C.addFrame("gt_waypoint")
    gt_waypoint_ball.setShape(ry.ST.sphere, size=[0.02])
    gt_waypoint_ball.setColor([0.0, 1.0, 0.0])
    gt_waypoint_ball.setPosition(cur_waypoint[:3])
    C.addFrame("gt_waypoint")
    C.view(pause=True)

    for waypoint in waypoints:
        cur_waypoint_ball.setPosition(waypoint[:3])
        if i < len(gt_waypoints):
            gt_waypoint_ball.setPosition(gt_waypoints[i][:3])
        time.sleep(0.05)
        cur_input[[f"waypoints_{i}" for i in range(7)]] = cur_waypoint
        cur_input[[f"contacts_{i}" for i in range(3)]] = cur_contact
        cur_input["step"] = i + 1
        C.view(message=f"{i}/{len(waypoints)} (500 max)")

    C.view(pause=True)

    C.view(pause=True)
