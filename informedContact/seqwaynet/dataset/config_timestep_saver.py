import os
from pathlib import Path
import pandas as pd


def count_files_and_subdirectories_per_folder(root_folder):
    result = {}

    # Walk through the top-level directory (root_folder)
    for dirpath, dirnames, filenames in os.walk(root_folder):
        # Skip the root directory itself
        if Path(dirpath) == Path(root_folder):
            continue

        # Get the current directory name (subdirectory name)
        subdir_name = os.path.basename(dirpath)

        # Count the number of files and subdirectories within this subdirectory
        total_files = len(filenames)
        print(subdir_name, total_files)
        # total_subdirs = len(dirnames)

        # Add to the result dictionary
        result[subdir_name] = (
            total_files - 12
        ) // 2  # there should be 12 non-config files, and for each timestep we have 1 .g file and 1 .pkl file
    return result


if __name__ == "__main__":
    # Example usage
    root_folder = "/home/kutay/repos/ProgressiveNet/seqwaynet/data/train/tunnel"
    folder_stats = count_files_and_subdirectories_per_folder(root_folder)

    df = pd.DataFrame(list(folder_stats.items()), columns=["episode", "timesteps"])
    df = df.sort_values(by="episode")
    df.to_csv(Path(root_folder) / "total_timesteps.csv", index=False)
