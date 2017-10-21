# pcl-select-center
`pcl-select-center` lets the user select the center of the lettuce-of-interest of a point cloud. The center is then stored in a csv-file.

## Usage
The following section shows how to use `pcl-select-center`. The example is based on the provided cos-example and assumes, that a terminal window is opened in the `data` folder.

### Select center of lettuce-of-interest
In terminal, type:
```
pcl-select-center cos-example/pc-scaled.pcd cos-example/center.csv
```
This will load the point cloud `pc-scaled.pdc` and display it. Use `shift` + `left mouse` to select a point as the center of the lettuce-of-interest. A magenta/pink rectangle will be displayed around the selected point. The point can be moved by selecting a new point. Press `q` to close the window and save the selected point to the csv-file.
