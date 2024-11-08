import os
import csv


def save_contacts_from_csv(input_csv_file):
    """
    Extract the first 4 columns from each row in the CSV file and save them as 'contacts_i.csv'
    where i is the row number (starting from 1). The 4th column is converted such that
    nonzero values become 1, and zero values remain 0.
    """
    # Get the directory where the input CSV file is located
    directory = os.path.dirname(input_csv_file)

    # Open the input CSV file for reading
    with open(input_csv_file, newline="") as csvfile:
        csv_reader = csv.reader(csvfile)

        # Iterate over each row, keeping track of the row number
        for row_num, row in enumerate(csv_reader):
            # Extract the first 4 columns of the current row
            contacts = row[:4]

            # Convert the 4th column (index 3) to 1 if it's non-zero, else 0
            if float(contacts[3]) != 0:
                contacts[3] = 1
            else:
                contacts[3] = 0

            # Prepare the filename for saving this row's contacts
            output_csv_file = os.path.join(directory, f"contacts_{row_num}.csv")

            # Write the extracted and modified contacts to the new CSV file
            with open(output_csv_file, mode="w", newline="") as outputfile:
                csv_writer = csv.writer(outputfile)
                csv_writer.writerow(contacts)

            print(f"Saved row {row_num} to {output_csv_file}")


def traverse_and_process(root_directory):
    """
    Traverse the root directory, find all CSV files containing 'contact_point' in their names,
    and process them to extract contact points and save them in separate files.
    """
    for subdir, _, files in os.walk(root_directory):
        for file in files:
            if "contact_point" in file and file.endswith(".csv"):
                csv_file_path = os.path.join(subdir, file)
                print(f"Processing {csv_file_path}...")
                save_contacts_from_csv(csv_file_path)


# Example usage
root_directory = "/home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/tunnel_tool/sample"  # Replace with the actual root directory
traverse_and_process(root_directory)
