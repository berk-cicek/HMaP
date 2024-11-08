import os
import re


def rename_g_files_with_temp(root_directory):
    """
    Traverse the root directory and rename '.g' files by reducing the number in the filename by 1.
    First, rename to temporary filenames to avoid overwriting, and then rename to final filenames.
    """
    files_to_rename = []

    # Step 1: Collect the files to rename
    for subdir, _, files in os.walk(root_directory):
        for file in files:
            match = re.match(r"config_(\d+)\.g", file)
            if match:
                old_number = int(match.group(1))
                new_number = old_number - 1
                temp_filename = f"temp_{new_number}.g"
                final_filename = f"config_{new_number}.g"

                old_filepath = os.path.join(subdir, file)
                temp_filepath = os.path.join(subdir, temp_filename)
                final_filepath = os.path.join(subdir, final_filename)

                # Add the old, temp, and final file paths to the list
                files_to_rename.append((old_filepath, temp_filepath, final_filepath))

    # Step 2: Rename to temporary filenames
    for old_filepath, temp_filepath, _ in files_to_rename:
        os.rename(old_filepath, temp_filepath)
        print(f"Temporarily renamed {old_filepath} to {temp_filepath}")

    # Step 3: Rename temporary files to final filenames
    for _, temp_filepath, final_filepath in files_to_rename:
        os.rename(temp_filepath, final_filepath)
        print(f"Renamed {temp_filepath} to {final_filepath}")


# Example usage
root_directory = "/home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/tunnel/sample"  # Replace with the actual root directory
rename_g_files_with_temp(root_directory)
