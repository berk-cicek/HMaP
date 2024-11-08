#!/bin/bash

# python ../dataset/keypoint_config_preprocess.py /home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/puck_obs_around/sample
# python ../dataset/keypoint_config_preprocess.py /home/kutay/repos/ProgressiveNet/seqwaynet/data/test/keypoint/puck_obs_around/sample

# python train_contact.py puck_obs_around

python ../dataset/keypoint_config_preprocess.py /home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/tunnel_tool/sample
python ../dataset/keypoint_config_preprocess.py /home/kutay/repos/ProgressiveNet/seqwaynet/data/test/keypoint/tunnel_tool/sample

python train_contact.py tunnel_tool

python ../dataset/keypoint_config_preprocess.py /home/kutay/repos/ProgressiveNet/seqwaynet/data/train/keypoint/puck_obs_move/sample
python ../dataset/keypoint_config_preprocess.py /home/kutay/repos/ProgressiveNet/seqwaynet/data/test/keypoint/puck_obs_move/sample

python train_contact.py puck_obs_move