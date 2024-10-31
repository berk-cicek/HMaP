import os
from pathlib import Path


def replace_in_file(file_path, old_str, new_str):
    """Replace all occurrences of old_str with new_str in a file."""
    # Read the content of the file
    with open(file_path, "r") as file:
        content = file.read()

    # Replace the old string with the new one
    new_content = content.replace(old_str, new_str)

    # Write the updated content back to the file
    with open(file_path, "w") as file:
        file.write(new_content)


def replace_in_files(root_folder, old_str, new_str):
    """Traverse through the root folder and subfolders, and replace old_str with new_str in all 'config_X.g' files."""
    # Use pathlib to easily work with paths
    root_path = Path(root_folder)

    # Walk through the directory tree
    for dirpath, _, filenames in os.walk(root_path):
        for filename in filenames:
            # Check if the file matches the pattern 'config_X.g'
            if filename.startswith("config_") and filename.endswith(".g"):
                file_path = Path(dirpath) / filename
                print(f"Processing file: {file_path}")

                # Replace the old string with the new one in the file
                replace_in_file(file_path, old_str, new_str)


if __name__ == "__main__":
    # Example usage
    root_folder = "/home/bora/git/hmapBiman/tunnel_task/sample_tunnel/sample/"
    new_str = "/home/bora/git/HMAP/config/tunnel/../../../botop/"
    old_str = "/home/kutay/repos/ProgressiveNet/seqwaynet/dataset/resources/"

    # Call the function to replace in all files
    replace_in_files(root_folder, old_str, new_str)
