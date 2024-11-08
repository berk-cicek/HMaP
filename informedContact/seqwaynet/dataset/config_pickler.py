import os
import robotic as ry
from pathlib import Path
from seqwaynet.dataset.utils import utils
from tqdm import tqdm


def read_config_objs(C: ry.Config, task: str):
    if task == "tunnel":
        frames = C.getFrameNames()
        objs = []
        for frame_name in frames:
            obj = C.getFrame(frame_name)
            obj_info = obj.info()

            if (
                "shape" in obj_info.keys()
                and obj_info["shape"] != "marker"
                # and "panda" not in obj_info["name"]
                # and "table" not in obj_info["name"]
                and "contact" not in obj_info["name"]
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
    else:
        raise NotImplementedError()

    return objs


def process_configs(root_folder: str, task: str):
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

                objs = read_config_objs(C, task)

                utils.save_as_pickle(objs, dirpath + f"/config_{config_timestep}.pkl")


if __name__ == "__main__":
    # Example usage
    root_folder = "/home/kutay/repos/ProgressiveNet/seqwaynet/data/test/tunnel"
    task = "tunnel"

    # Call the function to replace in all files
    process_configs(root_folder, task)
