import os
from pathlib import Path


def rename_folders(root_folder):
    # Get a sorted list of subdirectory paths
    subdirs = sorted([d for d in Path(root_folder).iterdir() if d.is_dir()])

    # Loop through the subdirectories and rename them sequentially
    for i, subdir in enumerate(subdirs):
        # Extract the new folder name with a sequential number
        new_folder_name = f"keypoints_{i}"

        # Create the full path for the new name
        new_folder_path = subdir.parent / new_folder_name

        # Rename the subdirectory
        subdir.rename(new_folder_path)
        print(f"Renamed {subdir} to {new_folder_path}")


# Example usage
root_folder = (
    "/home/kutay/repos/ProgressiveNet/seqwaynet/data/test/keypoint/tunnel_tool/sample"
)
rename_folders(root_folder)
