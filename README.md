# Tracking Dynamic Points

This document outlines a robust and efficient pipeline for detecting moving objects using a 2D LiDAR scanner on a moving robot. The process includes point cloud prediction, registration, and filtering techniques to isolate dynamic objects.

## Steps in the Pipeline

### 1. Prediction of the Next Frame Point Cloud
```math
P_p = T(\mathbf{v}) \cdot P_c
```
Compute the predicted point cloud \( P_p \) by applying the transformation \( T(\mathbf{v}) \) to the current point cloud \( P_c \).

### 2. Bounding Box Cropping
```math
P_p = P_p \cap 	ext{BBox}
```
Crop \( P_p \) to retain only the points within the bounding box \( 	ext{BBox} \), which corresponds to the robot's expected field of view.

### 3. Radius Outlier Removal on Predicted Points
```math
P_p = \left\{ \mathbf{p}_p \in P_p \mid 	ext{count}(\{\mathbf{p} \in P_p \mid d(\mathbf{p}_p, \mathbf{p}) \leq r\}) \geq k 
ight\}
```
Apply a radius outlier removal filter to \( P_p \), keeping only those points that have at least \( k \) neighbors within a radius \( r \).

### 4. Registration of the Actual Next Frame
```math
P_a' = 	ext{ICP}(P_a, P_p)
```
Use ICP to align the actual next frame point cloud \( P_a \) with the processed \( P_p \).

### 5. Matching Points (Static)
```math
	ext{Static Points} = \left\{ \mathbf{p}_a \in P_a \mid \exists \mathbf{p}_p \in P_p, \, d(\mathbf{p}_a, \mathbf{p}_p) < \epsilon 
ight\}
```
Identify points in \( P_a \) that match closely with points in \( P_p \) within the threshold \( \epsilon \).

### 6. Non-Matching Points (Dynamic)
```math
	ext{Dynamic Points} = P_a \setminus 	ext{Static Points}
```
The remaining points in \( P_a \) are considered dynamic as they do not match any point in \( P_p \).

### 7. Radius Outlier Removal on Dynamic Points
```math
	ext{Dynamic Points} = \left\{ \mathbf{p}_d \in 	ext{Dynamic Points} \mid 	ext{count}(\{\mathbf{p} \in 	ext{Dynamic Points} \mid d(\mathbf{p}_d, \mathbf{p}) \leq r\}) \geq k 
ight\}
```
Apply the radius outlier removal filter to the **Dynamic Points** to eliminate isolated points and small clusters that might be noise.
