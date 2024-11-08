from seqwaynet.dataset.dataset import ConfigDataset

import pathlib
import matplotlib.pyplot as plt

cur_dir = pathlib.Path(__file__).resolve().parent
print(f"{cur_dir}/../data/")
train_dataset = ConfigDataset(
    f"{cur_dir}/../data/", task="tunnel", return_type="image", device="cpu"
)

print(len(train_dataset))

print(train_dataset[0][0] + train_dataset.mean_image)
print(train_dataset.mean_image)

plt.imshow((train_dataset[0][0] + train_dataset.mean_image))
plt.show()

train_df = train_dataset.to_autogluon_dataframe()

print(train_df.head())
