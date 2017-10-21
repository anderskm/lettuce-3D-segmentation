# pcl-extract-lettuce-features/
`pcl-extract-lettuce-features` extracts features from a segmented lettuce-of-interest  3D point clouds.
The extracted features are:

- Volume
   - Convex hull
   - Convex hull, incl. stem point
   - Concave hull
- Surface area
   - Convex hull
   - Convex hull, incl. stem point
   - Concave hull
   - Voxel count
- Leaf cover area
   - Convex hull
   - Concave hull
   - Voxel count
- Height

See the paper for more information.

## Usage
The following section show how to use `pcl-extract.lettuce-features`. The example is based on the provided cos-example and assumes, that a terminal window is opened in the `data` folder.

### Extract features and save to csv-file

```
pcl-extract-lettuce-features cos-example/pc-plant.pcd cos-example/features.csv
```

