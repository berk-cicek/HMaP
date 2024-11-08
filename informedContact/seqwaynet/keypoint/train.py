import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from seqwaynet.dataset.keypoint_dataset import KeypointDataset
from seqwaynet.keypoint.keypoint_model import NextKeypointPredictor
from tqdm import tqdm
import json
import pathlib


def create_train_val_splits(dataset, val_split_ratio=0.2):
    """
    Split the dataset into training and validation sets.

    Args:
        dataset (torch.utils.data.Dataset): The full dataset to split.
        val_split_ratio (float): The fraction of data to use for validation.

    Returns:
        train_dataset, val_dataset: The split datasets.
    """
    # Compute lengths for the splits
    dataset_size = len(dataset)
    val_size = int(val_split_ratio * dataset_size)
    train_size = dataset_size - val_size

    # Randomly split the dataset into train and validation sets
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

    return train_dataset, val_dataset


def train_model(
    model: NextKeypointPredictor,
    dataloader,
    val_dataloader,
    num_epochs,
    patience,
    output_root,
):
    best_loss = float("inf")  # To track the best validation loss
    no_improvement_counter = 0  # To count epochs without improvement
    early_stop = False

    # Lists to track average losses per epoch
    train_losses = []
    val_losses = []

    for epoch in tqdm(range(num_epochs), desc=f"Epochs:"):
        if early_stop:
            print(f"Stopping early at epoch {epoch+1}")
            break

        model.train()  # Set the model to training mode
        epoch_train_loss = 0.0

        for batch in tqdm(dataloader, desc=f"Batches: "):
            input_configs, input_keypoints, targets, _, _ = batch

            input_configs = input_configs.cuda()
            input_keypoints = input_keypoints.cuda()
            targets = targets.cuda()

            optimizer.zero_grad()

            # Forward pass
            outputs = model.forward(input_configs, input_keypoints)

            # Compute loss
            loss = criterion(outputs, targets)

            # Backward pass and optimization
            loss.backward()
            optimizer.step()

            # Accumulate the batch loss
            epoch_train_loss += loss.item()

        # Compute the average training loss for the epoch
        avg_train_loss = epoch_train_loss / len(dataloader)
        train_losses.append(avg_train_loss)  # Store the average training loss

        # Validation step to compute validation loss
        val_loss = evaluate_model(model, val_dataloader)
        val_losses.append(val_loss)  # Store the average validation loss

        print(
            f"Epoch [{epoch+1}/{num_epochs}], Training Loss: {avg_train_loss:.4f}, Validation Loss: {val_loss:.4f}"
        )

        # Early stopping condition
        if val_loss < best_loss:
            best_loss = val_loss  # Update best validation loss
            no_improvement_counter = 0  # Reset the counter
            # Save the best model
            torch.save(model.state_dict(), f"{output_root}/best_model.pth")
        else:
            no_improvement_counter += 1
            if no_improvement_counter >= patience:
                print(f"Early stopping after {epoch+1} epochs")
                early_stop = True

    # Save the losses to a JSON file after training
    save_losses(train_losses, val_losses, filename=f"{output_root}/losses.json")


def evaluate_model(model, val_dataloader):
    """Evaluate the model on the validation set and return the average validation loss."""
    model.eval()  # Set model to evaluation mode
    val_loss = 0.0

    with torch.no_grad():  # Disable gradient computation
        for batch in val_dataloader:
            input_configs, input_keypoints, targets, _, _ = batch

            input_configs = input_configs.cuda()
            input_keypoints = input_keypoints.cuda()
            targets = targets.cuda()

            # Forward pass
            outputs = model.forward(input_configs, input_keypoints)

            # Compute loss
            loss = criterion(outputs, targets)
            val_loss += loss.item()

    return val_loss / len(val_dataloader)  # Return average validation loss


def evaluate_model(model: NextKeypointPredictor, val_dataloader: DataLoader):
    """Evaluate the model on the validation set and return the average validation loss."""
    model.eval()  # Set model to evaluation mode
    val_loss = 0.0

    with torch.no_grad():  # Disable gradient computation
        for batch in tqdm(val_dataloader, desc="Validation Batches:"):
            input_configs, input_keypoints, targets, _, _ = batch

            input_configs = input_configs.cuda()
            input_keypoints = input_keypoints.cuda()
            targets = targets.cuda()

            # Forward pass
            outputs = model.forward(input_configs, input_keypoints)

            # Compute loss
            loss = criterion(outputs, targets)
            val_loss += loss.item()

    return val_loss / len(val_dataloader)  # Return average validation loss


def save_losses(train_losses, val_losses, filename="losses.json"):
    """Save the training and validation losses to a JSON file."""
    loss_data = {"train_losses": train_losses, "val_losses": val_losses}

    with open(filename, "w") as f:
        json.dump(loss_data, f)

    print(f"Training and validation losses saved to {filename}.")


if __name__ == "__main__":

    task = "bookshelf"

    keypoint_dataset = KeypointDataset(
        f"/home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/{task}/keypoints"
    )

    train_dataset, val_dataset = create_train_val_splits(
        keypoint_dataset, val_split_ratio=0.2
    )

    cur_dir = pathlib.Path(__file__).resolve().parent
    output_root = f"{cur_dir}/output/{task}"

    # Create DataLoaders for training and validation
    train_dataloader = DataLoader(train_dataset, batch_size=128, shuffle=True)
    val_dataloader = DataLoader(val_dataset, batch_size=128, shuffle=False)

    # dataloader = DataLoader(keypoint_dataset, batch_size=256, shuffle=True)

    config_vec, cur_waypoint, next_waypoint, _, _ = keypoint_dataset[0]

    print(config_vec.shape, cur_waypoint.shape, next_waypoint.shape)
    print(cur_waypoint, next_waypoint)
    model = NextKeypointPredictor(len(config_vec)).cuda()

    # Loss and optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)  # Adam optimizer

    num_epochs = 300

    train_model(
        model,
        train_dataloader,
        val_dataloader,
        num_epochs,
        patience=10,
        output_root=output_root,
    )
