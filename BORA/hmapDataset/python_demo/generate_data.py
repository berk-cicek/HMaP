from seqwaynet.dataset.dataset_generation import DatasetGenerator
from seqwaynet.dataset.dataset import ConfigDataset
import pathlib
import matplotlib.pyplot as plt
import numpy as np

root_dir = pathlib.Path(__file__).resolve().parent.parent
# resources_dir = f"{root_dir}/resources"
gen = DatasetGenerator(f"{root_dir}/data")
gen.generate("bolt", count=4096, number_of_waypoints=[10])

dataset = ConfigDataset(f"{root_dir}/data", return_type="vector", device="cpu")
print("info", dataset.info)

config, waypoint, contact = dataset[0]

print(dataset.config_size, dataset.frame_size)
