# Sensor Fusion (Udacity)
## Lidar Obstacle Detection

Process raw lidar data with filtering, segmentation, and clustering to detect other vehicles on the road. Implemented Ransac with planar model fitting to segment point clouds. Euclidean clustering with a KD-Tree was implemented to cluster and distinguish vehicles and obstacles.

Main components of this project: 
- Plane Segmentation with RANSAC
- Euclidean clustering with KD-Tree
- Reading and streaming PCDs
  
<img src="https://github.com/user-attachments/assets/72341690-405a-4cf1-9a37-781ed8d739b6"  width="500" height="350"/>
<img src="https://github.com/user-attachments/assets/238bfead-790f-489b-90a2-08fe432849d2"  width="500" height="300"/>

## Kalman Filter
Fuse data from multiple sources using Kalman filters. Merge data together using the prediction-update cycle of Kalman filters, which accurately track object moving along straight lines. Built extended and unscented Kalman filters for tracking nonlinear movement. The Constant Turn Rate and Velocity Magnitude (CTRV) model is a motion model used in robotics and autonomous vehicle systems, particularly for object tracking, which assumes that an object is moving with a constant velocity magnitude and a constant turn rate, and the Unscented Kalman Filter implementation is based on CTRV model. In the animation below, the green path represents the predicted path by the Kalman Filter. The Root Mean Squared Error (RMSE) between estimation and ground truth values is successfully minimized.

Main components of this section: 
- Normal Kalman Filter (not used in the project)
- Extended Kalman Filter (not used in the project)
- Unscented Kalman Filter (used in the project)
<img src="https://github.com/user-attachments/assets/5c641796-f89e-43bb-b4e8-c08946a91581"  width="788" height="450"/>

![UKF](https://github.com/user-attachments/assets/a2b8f13f-f61f-4b5e-8c9e-98297d7d6643)
