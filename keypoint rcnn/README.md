# Conda setup for training keypoint rcnn

## This tutorial helps you setup a conda virtual python environement containing pytorch.  

- Create a new conda environemnet 
```
conda create -n my-torch python=3.7 -y
```

- Actiavte the new conda environment

```
conda activate my-torch  
```

- Install pytorch, Jupyter and other dependencies required for training the model
```
conda install python=3.6 pytorch torchvision matplotlib pandas -c pytorch  
conda install jupyter
```

- Pycocotools contains helper functions for various evaluation metrics used to train and validate a deep learning model

```
conda install -c conda-forge/label/cf202003 pycocotools
```

- If you wish to run a ROS node inside my-torch python environment install pyyaml which helps the virtual env use rospy installed in the base machine
```
pip install pyyaml
```

#### All set to train

- Open jupyter notebook using the below command. Make sure you open jupyter notebook from within the conda virtual environment
```
jupyter notebook
```

- Run [Keypoint_Rcnn_main](https://gitlab.com/barczyk-mechatronic-systems-lab/keypoint_detection/-/blob/main/keypoint%20rcnn/Keypoint_Rcnn_main.ipynb) notebook 
- After completion of training, the model will be saved in the same working directory as real_keypoint.pt pytorch file

- You can deploy the model using the following command in python
```
model = torch.load('real_keypoint.pt')
```
