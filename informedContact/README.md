# Instructions for model training

1. Put the samples in ```seqwaynet/data/train/{task}/sample```
2. Run ```cd ./seqwaynet/keypoint```
3. Run ```python contact_unpacker.py``` with the directory above
4. Run ```python keypoint_config_preprocess.py``` with the directory above
5. (Optional) Run ```python config_path_renamer.py``` with the directory above (if you want to do RAI visualization later)
5. Separate your test set (remaining files will be automatically split as training and validation set with 4:1 ratio) and put it to ``````seqwaynet/data/test/{task}/sample``````
6. Run ```python train_contact.py``` to train your models (outputs will be saved to ```seqwaynet/keypoint/output/{task}```)
7. (Optional) Use the ```eval.ipynb``` notebook to do inference on the test set & visualize
