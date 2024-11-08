import torch
import torchvision.models as models
import torch.nn as nn


class RGBInformed(nn.Module):
    def __init__(self, pretrained=True, train=False, device="cpu"):
        super(RGBInformed, self).__init__()
        if pretrained:
            self.resnet18 = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        else:
            self.resnet18 = models.resnet18(weights=None)

        if not train:
            for param in self.resnet18.parameters():
                param.requires_grad = False

        self.resnet18.to(device)

    def forward_features(self, x):
        for layer in list(self.resnet18.children())[:-1]:
            x = layer(x)
        return x

    def forward_features_multiple(self, cam1, cam2, cam3):
        return torch.cat(
            [
                self.forward_features(cam1),
                self.forward_features(cam2),
                self.forward_features(cam3),
            ],
            dim=1,
        )

    def forward(self, x):
        return self.resnet18(x)
