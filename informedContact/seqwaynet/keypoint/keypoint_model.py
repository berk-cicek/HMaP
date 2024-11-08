import torch
import torch.nn as nn
import torch.nn.functional as F

"""
data generation notes:

if there are contact switches: keypoints are where contact switch occurs
if not (e.g. puck, bolt): sample keypoints with a heuristic
"""


class NextKeypointPredictor(nn.Module):
    """
    input:
        - config encoding (might be a vector or some image embedding)
        - current keypoint (i.e. where the object is right now) (3d)
        - possibly time step (optional, maybe keypoint index, maybe rrt time)
    output:
        - next keypoint (3d)
    """

    CONTACT_SIZE = 3
    GRIPPER_IDX_SIZE = 1

    def __init__(self, config_emb_size: int, hidden_size: int = 512):
        super(NextKeypointPredictor, self).__init__()
        self.config_emb_size = config_emb_size
        self.input_size = (
            config_emb_size
            + NextKeypointPredictor.CONTACT_SIZE
            + NextKeypointPredictor.GRIPPER_IDX_SIZE
        )

        self.block1 = nn.Sequential(
            nn.Linear(self.input_size, hidden_size),
            nn.BatchNorm1d(hidden_size),
            nn.ReLU(),
            nn.Dropout(0.3),
        )

        self.block2 = nn.Sequential(
            nn.Linear(hidden_size, hidden_size // 2),
            nn.BatchNorm1d(hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(0.3),
        )

        self.block3 = nn.Sequential(
            nn.Linear(hidden_size // 2, hidden_size // 4),
            nn.BatchNorm1d(hidden_size // 4),
            nn.ReLU(),
        )

        self.block4 = nn.Sequential(
            nn.Linear(hidden_size // 4, hidden_size // 8),
            nn.BatchNorm1d(hidden_size // 8),
            nn.ReLU(),
        )

        self.output_contact = nn.Linear(
            hidden_size // 8, NextKeypointPredictor.CONTACT_SIZE
        )
        self.output_gripper = nn.Sequential(
            nn.Linear(hidden_size // 8, hidden_size // 16),
            nn.BatchNorm1d(hidden_size // 16),
            nn.ReLU(),
            nn.Linear(
                hidden_size // 16, NextKeypointPredictor.GRIPPER_IDX_SIZE
            ),  # Next gripper index (binary)
        )

    def forward(self, config_emb, cur_contact, cur_gripper):
        x = torch.cat((config_emb, cur_contact, cur_gripper), dim=1)
        x = self.block1(x)
        x = self.block2(x)
        x = self.block3(x)
        x = self.block4(x)

        # Separate outputs
        contact_pred = self.output_contact(x)  # 3D vector
        gripper_pred = torch.sigmoid(self.output_gripper(x))  # Binary variable (0 or 1)
        return contact_pred, gripper_pred
