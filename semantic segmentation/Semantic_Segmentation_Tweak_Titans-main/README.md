# Information

This is an implementation of Semantic Segmentation and also Lane Regression with Bird's Eye View.
The repo has two codes for training one which calculates mIoU of every class and the other which gives only the overall mIoU. We have trained for 15 epochs for testing in unknown data and we have used the code with only overall mIoU for faster training. On the other hand we have trained for 5 epochs to show the mIoU calculation for every class and plotted the results _(which can be found in /logs_5)_.

# Dataset

The dataset will be downloaded by calling the following commands

```console
cd dataset_helper
python3 run_freicar_segreg_dataloader.py
```

This downloads a `.zip` file and extracts it content in the `dataloader` directory.
After extraction is completed, the zipfile will be automatically removed.

# Training

You can start your training by calling the command

```console
python3 train_segmentation.py
```
Change the number of epochs appropriately.

 

# ROS Node
To initiate ROS node, start the core
```console
roscore
```
Next, launch the freicar simulator using
```console
roslaunch freicar_launch local_comp_launch.launch
```
Next, run the agent node using
```console
roslaunch freicar_agent sim_agent.launch name:=freicar_1 tf_name:=freicar_1 spawn/x:=0 spawn/y:=0 spawn/z:=0 spawn/heading:=20 use_yaml_spawn:=true sync_topic:=!
```


# Lane Segmentation, Regression, Bird's Eye View Script and ROS publish


If you want to have a look at the lane segmentation, regression and bird's eye view, you can run
```console
python3 lane_center_pub.py
```
Also, it publishes the MarkerArrays in ROS.
