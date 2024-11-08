from seqwaynet.dataset.dataset import ConfigDataset
import robotic as ry
import pathlib
import time

# task = "tunnel"

# cur_dir = pathlib.Path(__file__).resolve().parent
# dataset_root = f"{cur_dir}/../data/test"
# dataset = ConfigDataset(dataset_root, task)

# for ep, dir_idx in enumerate(dataset.ep_to_dir_idx):
#     C = ry.Config()
#     C.addFile(f"{dataset_root}/{task}/{task}_{dir_idx}/config_{1}.g")
#     for t in range(dataset.episode_total_timesteps[f"{task}_{dir_idx}"]):
#         # time.sleep(0.5)
#         C = ry.Config()

#         # C = ry.Config()
#         C.addFile(f"{dataset_root}/{task}/{task}_{dir_idx}/config_{t + 1}.g")

#         # obje düzgün gidiyor da robotların şekilden şekile girmesi normal mi
#         C.view(
#             pause=True,
#             message=f"{t}/{dataset.episode_total_timesteps[f'{task}_{dir_idx}']}",
#         )

#     C.view(pause=True)
