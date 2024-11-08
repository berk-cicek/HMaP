import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from seqwaynet.dataset.keypoint_dataset import KeypointDataset
from seqwaynet.keypoint.keypoint_model import NextKeypointPredictor
from tqdm import tqdm
import json
import pathlib
import numpy as np
import os
import re
import pandas as pd
import argparse


def create_train_val_splits(dataset: NextKeypointPredictor, val_split_ratio=0.2):
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

    if dataset.contact_mean is not None:
        (
            train_dataset.config_mean,
            train_dataset.config_std,
            train_dataset.contact_mean,
            train_dataset.contact_std,
        ) = (
            dataset.config_mean,
            dataset.config_std,
            dataset.contact_mean,
            dataset.contact_std,
        )

        (
            val_dataset.config_mean,
            val_dataset.config_std,
            val_dataset.contact_mean,
            val_dataset.contact_std,
        ) = (
            dataset.config_mean,
            dataset.config_std,
            dataset.contact_mean,
            dataset.contact_std,
        )

    return train_dataset, val_dataset


def train_model(
    model: NextKeypointPredictor,
    dataloader,
    val_dataloader,
    num_epochs,
    patience,
    output_root,
    criterion_contact,  # Loss function for keypoint prediction (e.g., MSELoss or HuberLoss)
    criterion_gripper,  # Loss function for gripper index prediction (e.g., BCELoss)
    optimizer,
):
    best_loss = float("inf")  # To track the best validation loss
    no_improvement_counter = 0  # To count epochs without improvement
    early_stop = False

    # Lists to track average losses per epoch
    train_losses = []
    train_losses_mse = []
    train_losses_bce = []
    val_losses = []
    val_losses_mse = []
    val_losses_bce = []

    for epoch in tqdm(range(num_epochs), desc=f"Epochs:"):
        # if early_stop:
        #     print(f"Stopping early at epoch {epoch+1}")
        #     break

        model.train()  # Set the model to training mode
        epoch_train_loss_mse = 0.0
        epoch_train_loss_bce = 0.0

        for batch in tqdm(dataloader, desc=f"Batches: "):
            (
                input_configs,
                contact_current,
                gripper_idx_current,
                contact_next,
                gripper_idx_next,
                _,
                _,
            ) = batch

            # Move data to GPU if available
            input_configs = input_configs.cuda()
            contact_current = contact_current.cuda()
            gripper_idx_current = gripper_idx_current.view(-1, 1).cuda()
            contact_next = contact_next.cuda()
            gripper_idx_next = gripper_idx_next.view(-1, 1).cuda()

            # Label smoothing for gripper
            smoothing = 0.1 * torch.rand(gripper_idx_next.shape).cuda()
            gripper_idx_next = (
                torch.ones_like(smoothing) - smoothing
            ) * gripper_idx_next + smoothing

            # print(gripper_idx_current)

            # print(
            #     input_configs.shape,
            #     contact_current.shape,
            #     gripper_idx_current.shape,
            #     contact_next.shape,
            #     gripper_idx_next.shape,
            # )
            optimizer.zero_grad()

            # Forward pass
            contact_pred, gripper_pred = model(
                input_configs, contact_current, gripper_idx_current
            )

            # Compute losses
            loss_contact = criterion_contact(
                contact_pred, contact_next
            )  # MSELoss for keypoint
            loss_gripper = criterion_gripper(
                gripper_pred, gripper_idx_next
            )  # BCELoss for gripper index
            total_loss = loss_contact + loss_gripper  # Combine both losses

            # Backward pass and optimization
            total_loss.backward()
            optimizer.step()

            # Accumulate the batch loss
            epoch_train_loss_mse += loss_contact.item()
            epoch_train_loss_bce += loss_gripper.item()

        # Compute the average training loss for the epoch
        avg_train_loss_mse = epoch_train_loss_mse / len(dataloader)
        avg_train_loss_bce = epoch_train_loss_bce / len(dataloader)
        avg_train_loss = avg_train_loss_mse + avg_train_loss_bce

        train_losses.append(avg_train_loss)  # Store the average training loss
        train_losses_mse.append(avg_train_loss_mse)
        train_losses_bce.append(avg_train_loss_bce)

        # Validation step to compute validation loss
        avg_val_loss, avg_val_loss_mse, avg_val_loss_bce = evaluate_model(
            model, val_dataloader, criterion_contact, criterion_gripper
        )
        val_losses.append(avg_val_loss)  # Store the average validation loss
        val_losses_mse.append(avg_val_loss_mse)
        val_losses_bce.append(avg_val_loss_bce)

        print(
            f"Epoch [{epoch+1}/{num_epochs}], Training Loss: {avg_train_loss:.4f} ({avg_train_loss_mse:.4f} + {avg_train_loss_bce:.4f}), Validation Loss: {avg_val_loss:.4f} ({avg_train_loss_mse:.4f} + {avg_val_loss_bce:.4f})"
        )

        # Early stopping condition
        if avg_val_loss < best_loss:
            best_loss = avg_val_loss  # Update best validation loss
            no_improvement_counter = 0  # Reset the counter
            # Save the best model
            torch.save(model.state_dict(), f"{output_root}/best_model.pth")
        else:
            no_improvement_counter += 1
            if no_improvement_counter >= patience:
                print(f"Early stopping after {epoch+1} epochs")
                early_stop = True

    # Save the losses to a JSON file after training
    save_losses(
        train_losses,
        train_losses_mse,
        train_losses_bce,
        val_losses,
        val_losses_mse,
        val_losses_bce,
        filename=f"{output_root}/losses.json",
    )


def evaluate_model(
    model: NextKeypointPredictor,
    val_dataloader: DataLoader,
    criterion_contact,
    criterion_gripper,
):
    """Evaluate the model on the validation set and return the average validation loss."""
    model.eval()  # Set model to evaluation mode
    val_loss_mse = 0.0
    val_loss_bce = 0.0

    with torch.no_grad():  # Disable gradient computation
        for batch in tqdm(val_dataloader, desc="Validation Batches:"):
            (
                input_configs,
                contact_current,
                gripper_idx_current,
                contact_next,
                gripper_idx_next,
                config_path,
                contact_path,
            ) = batch

            contact_config_save_path = replace_keypoint_with_pred(contact_path[0])

            # Move data to GPU if available
            input_configs = input_configs.cuda()
            contact_current = contact_current.cuda()
            contact_next = contact_next.cuda()
            gripper_idx_current = gripper_idx_current.view(-1, 1).cuda()
            gripper_idx_next = gripper_idx_next.view(-1, 1).cuda()

            # Forward pass
            contact_pred, gripper_pred = model(
                input_configs, contact_current, gripper_idx_current
            )

            # Compute losses
            loss_contact = criterion_contact(contact_pred, contact_next)
            loss_gripper = criterion_gripper(gripper_pred, gripper_idx_next)
            total_loss = loss_contact + loss_gripper

            val_loss_mse += loss_contact.item()
            val_loss_bce += loss_gripper.item()

            gripper_pred = torch.round(gripper_pred)
            outputs = torch.cat((contact_pred, gripper_pred), dim=1).cpu()
            outputs = pd.DataFrame(outputs).to_csv(
                contact_config_save_path, header=None, index=None
            )

    avg_val_loss_mse = val_loss_mse / len(val_dataloader)
    avg_val_loss_bce = val_loss_bce / len(val_dataloader)
    avg_val_loss = avg_val_loss_mse + avg_val_loss_bce
    return (
        avg_val_loss,
        avg_val_loss_mse,
        avg_val_loss_bce,
    )  # Return average validation loss


def save_losses(
    train_losses,
    train_losses_mse,
    train_losses_bce,
    val_losses,
    val_losses_mse,
    val_losses_bce,
    filename="losses.json",
):
    """Save the training and validation losses to a JSON file."""
    loss_data = {
        "train_losses": train_losses,
        "train_losses_mse": train_losses_mse,
        "train_losses_bce": train_losses_bce,
        "val_losses": val_losses,
        "val_losses_mse": val_losses_mse,
        "val_losses_bce": val_losses_bce,
    }

    with open(filename, "w") as f:
        json.dump(loss_data, f)

    print(f"Training and validation losses saved to {filename}.")


def replace_keypoint_with_pred(path):
    # Extract the directory and filename parts
    dir_name, file_name = os.path.split(path)

    # Use regex to match keypoint_X.csv and replace it with config_X.csv
    new_file_name = re.sub(r"contacts_(\d+)\.csv", r"pred_\1.csv", file_name)

    # Join the directory back with the new filename
    new_path = os.path.join(dir_name, new_file_name)

    return new_path


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("task", type=str, help="task")

    args = parser.parse_args()

    task = "puck_obs_around"
    # task = args.task

    dataset_root = (
        f"/home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/{task}/sample"
    )
    cur_dir = pathlib.Path(__file__).resolve().parent
    output_root = f"{cur_dir}/output_contact/{task}"
    # stats_file = f"{output_root}/stats.json"
    stats_file = None

    dataset = KeypointDataset(dataset_root, stats_file)

    config_vec, cur_contact, cur_gripper, next_contact, next_gripper, _, _ = dataset[0]

    # print(config_vec)
    # print(cur_contact, cur_gripper)
    # print(next_contact, next_gripper)

    train_dataset, val_dataset = create_train_val_splits(dataset, val_split_ratio=0.2)

    # Create DataLoaders for training and validation
    train_dataloader = DataLoader(train_dataset, batch_size=128, shuffle=True)
    val_dataloader = DataLoader(val_dataset, batch_size=128, shuffle=False)

    # dataloader = DataLoader(keypoint_dataset, batch_size=256, shuffle=True)

    config_vec, cur_contact, cur_gripper, next_contact, next_gripper, _, _ = dataset[0]

    # print(config_vec)
    # print(cur_contact, cur_gripper)
    # print(next_contact, next_gripper)

    # print(config_vec.shape, cur_waypoint.shape, next_waypoint.shape)
    # print(cur_waypoint, next_waypoint)
    model = NextKeypointPredictor(len(config_vec)).cuda()

    print("config vec len:", len(config_vec))

    # Loss and optimizer
    criterion_contact = nn.MSELoss()
    criterion_gripper = nn.BCELoss()
    optimizer = optim.Adam(
        model.parameters(), lr=1e-3, weight_decay=1e-4
    )  # Adam optimizer

    num_epochs = 300

    train_model(
        model,
        train_dataloader,
        val_dataloader,
        num_epochs,
        patience=10,
        output_root=output_root,
        criterion_contact=criterion_contact,
        criterion_gripper=criterion_gripper,
        optimizer=optimizer,
    )

    print("config vec len:", len(config_vec))

    # test here (notebook crashes for some reason)

    # test_dataset = KeypointDataset(
    #     f"/home/kutay/repos/ProgressiveNet/seqwaynet/data/test/keypoint/{task}/sample",
    #     stats_file=None,
    # )
    # test_dataloader = DataLoader(test_dataset, shuffle=False)

    # test_loss, test_loss_mse, test_loss_bce = evaluate_model(
    #     model, test_dataloader, criterion_contact, criterion_gripper
    # )

    # print(test_loss, test_loss_mse, test_loss_bce)
