import numpy as np
import os
import pickle
import cv2
import torch
import seqwaynet.dataset.preprocess as preprocess


def spherical_to_cartesian(r, theta, phi):
    """
    Convert spherical coordinates to Cartesian coordinates.
    """
    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.sin(phi) * np.sin(theta)
    z = r * np.cos(phi)
    return x, y, z


def rotate_points_around_x(xyz, theta):
    """
    Rotate points around the X axis by theta radians.
    """
    R_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)],
        ]
    )

    rotated_points = np.dot(xyz, R_x.T)

    return rotated_points


def save_config_to_file(config, filename):
    """
    Save a configuration to a file.
    """
    config_file = open(filename, "w")
    config_file.write(config.write())
    config_file.close()


def save_as_pickle(target, filename):
    with open(filename, "wb") as target_file:
        pickle.dump(target, target_file)


def save_images_to_file(rgb, depth, points, filename):
    cv2.imwrite(
        filename + "_rgb.png",
        cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
    )
    cv2.imwrite(
        filename + "_depth.png",
        (depth / depth.max() * 255).astype(np.uint8),
    )
    np.save(
        filename + "_points.npy",
        points,
    )


def create_directory(directory_path, directory_name):
    if not os.path.exists(directory_path + directory_name):
        os.makedirs(directory_path + directory_name)
        print(f"Directory '{directory_name}' created successfully")
    else:
        print(f"Directory '{directory_name}' already exists")

    return directory_path


# TODO: Deprecated, refactor usage to dataset is_instance
def get_sample_from_path(sample_path):
    if os.path.isdir(sample_path):
        image_path = []
        for data_name in sorted(os.listdir(sample_path)):
            data_path = sample_path + "/" + data_name
            if data_name.endswith(".g"):
                config_path = data_path
            if data_name.endswith("rgb.png"):
                image_path.append(data_path)
            if data_name.endswith("contactpoints.pkl"):
                with open(data_path, "rb") as contact_file:
                    contacts = pickle.load(contact_file)
            if data_name.endswith("waypoints.pkl"):
                with open(data_path, "rb") as waypoint_file:
                    waypoints = pickle.load(waypoint_file)

        images = [preprocess.preprocess_image(image) for image in image_path]
        images_tensor = torch.stack(images, dim=0)
        label_index = np.random.randint(0, len(contacts))
        contact = torch.from_numpy(contacts[label_index]).float()
        waypoint = torch.from_numpy(waypoints[label_index]).float()

        return {
            "config_path": config_path,
            "images_tensor": images_tensor,
            "contact": contact,
            "waypoint": waypoint,
        }
