import os
import torch
from seqwaynet.keypoint.keypoint_model import NextKeypointPredictor


def get_input_size(model):
    """Extract the input size from the first layer of the MLP model."""
    # Assuming the first layer is a `torch.nn.Linear` layer in an MLP
    input_layer = next(model.children())  # Get the first layer
    if isinstance(input_layer, torch.nn.Linear):
        input_size = input_layer.in_features
        return input_size
    else:
        raise ValueError("Model's first layer is not a Linear layer.")


def convert_to_torchscript(model_path, output_path, input_size):
    """Converts the PyTorch model to TorchScript and saves it."""
    # Load the PyTorch model to the CPU, even if it was saved in CUDA
    model = NextKeypointPredictor(config_emb_size=input_size)
    model.load_state_dict(torch.load(model_path, map_location="cpu"))

    # Set the model to evaluation mode
    model.eval()

    # Create a dummy input based on the input size
    dummy_input = torch.randn(1, input_size)  # MLPs usually take a 1D input
    dummy_keypoint = torch.randn(1, 6)

    # Trace the model with the dummy input
    traced_model = torch.jit.trace(model, (dummy_input, dummy_keypoint))

    # Save the traced model to the output path
    traced_model.save(output_path)
    print(f"Saved converted model to {output_path}")


def traverse_and_convert(root_dir):
    """Traverse through the root directory and convert .pth model files to TorchScript format."""
    for subdir, _, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".pth"):  # Look for .pth model files
                model_path = os.path.join(subdir, file)

                # Generate the output path by adding '_cpp' before the '.pth' extension
                output_path = model_path.replace(".pth", "_cpp.pth")

                print(f"Converting {model_path} to {output_path}...")
                convert_to_torchscript(model_path, output_path)


if __name__ == "__main__":
    # Set the root directory to traverse (modify this to your directory)
    # 576, 992, 595, 510, 544, 800, 935

    # 612,

    model_path = "/home/kutay/repos/ProgressiveNet/seqwaynet/keypoint/output/tunnel_tool/best_model.pth"
    output_path = model_path.replace(".pth", "_cpp.pth")
    input_size = 935
    # Traverse the directory and convert the models
    convert_to_torchscript(model_path, output_path, input_size)
