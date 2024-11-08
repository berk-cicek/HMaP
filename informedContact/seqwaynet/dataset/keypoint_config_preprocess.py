import os
import robotic as ry
from pathlib import Path
from seqwaynet.dataset.utils import utils
from tqdm import tqdm
import torch
import numpy as np
import argparse

from scipy.spatial.transform import Rotation


def read_config_objs(C: ry.Config):
    frames = sorted(C.getFrameNames())
    objs = []

    for frame_name in frames:

        obj = C.getFrame(frame_name)
        obj_info = obj.info()

        if (
            "shape" in obj_info.keys()
            and obj_info["shape"] != "marker"
            and "contact" not in obj_info["name"]
        ):
            rot = Rotation.from_quat(obj.getQuaternion()).as_quat().tolist()
            objs.append(
                {
                    "name": frame_name,
                    "pos": obj.getPosition(),
                    "quat": rot,
                    "size": obj.getSize() if obj_info["shape"] != "mesh" else [],
                    "shape": obj_info["shape"],
                }
            )

    return objs


def vectorize_config(objs):
    shape_types = ["ssBox", "box", "marker", "mesh", "capsule", "ssCylinder"]

    def one_hot_shape(shape: str):
        ret = [0] * len(shape_types)
        ret[shape_types.index(shape)] = 1
        return ret

    item = objs[0]

    frame_size = (
        len(item["pos"])
        + len(item["quat"])
        + 4  # we make each object have same length size vectors
        + len(one_hot_shape(item["shape"]))
    )

    vec = []

    for item in objs:
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
        vec.append(torch.tensor(one_hot_shape(item["shape"])))

    vec = torch.concat(vec, dim=0)
    assert len(vec) == len(objs) * frame_size, f"{len(vec)} {frame_size}"
    return vec


def process_configs(root_folder: str):
    # Use pathlib to easily work with paths
    root_path = Path(root_folder)

    # Walk through the directory tree
    for dirpath, _, filenames in tqdm(os.walk(root_path)):
        for filename in filenames:
            # Check if the file matches the pattern 'config_X.g'
            if filename.startswith("config_") and filename.endswith(".g"):
                file_path = Path(dirpath) / filename
                # print(f"Processing file: {file_path}")

                config_timestep = int(filename.replace("config_", "").replace(".g", ""))

                C = ry.Config()
                C.addFile(str(file_path))

                objs = read_config_objs(C)

                config = vectorize_config(objs)
                np.savetxt(
                    f"{dirpath}/config_{config_timestep}.csv",
                    config.numpy()[None],
                    delimiter=",",
                )


if __name__ == "__main__":
    # Example usage

    parser = argparse.ArgumentParser(description="Process a directory path.")
    parser.add_argument("dir_path", type=str, help="Path to the directory")
    args = parser.parse_args()

    root_folder = "/home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/bookshelf_mini/sample"

    root_folder = args.dir_path

    # Call the function to replace in all files
    process_configs(root_folder)
