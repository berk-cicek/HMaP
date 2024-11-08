import cv2
import torch
from torchvision import transforms


def read_image(image_path):
    return cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)


def preprocess_image(image_path):
    image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(image).float()  # Ensure the tensor is of type float
    tensor = tensor.permute(2, 0, 1)
    normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )
    tensor = normalize(tensor)
    return tensor
