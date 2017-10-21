# pcl-select-neighbor-centers
`pcl-select-neighbor-centers` lets the user select the center of the neighbouring plants of lettuce-of-interest of a point cloud. The centers are then stored in a pcd-file.

## Usage
The following section shows how to use `pcl-select-neighbor-centers`. The example is based on the provided examples and assumes, that a terminal window is opened in one of the data example folder.

### Select neighbour centers
In terminal, type:
```
pcl-select-neighbor-centers pc-scaled.pcd center.csv pc-neighbor-centers.pcd
```
This will load and display the point cloud `pc-scaled.pdc` as well as the coordinates of the center of the lettuce-of-interest specified by `center.csv`. Use `shift` + `left mouse` to select a point as the center of a neighbouring lettuce. A magenta/pink rectangle will be displayed around the selected point. The point can be moved by selecting a new point in the same fasion. Press `a` to "lock-in" a point as a neighbour. Once a neighbour center is locked-in it will turn blue and an ID will be displayed above it. Start by selecting the two neighbouring lettuce in the same row followed by the two corresponding lettuce in the opposite row. Select the neighbouring lettuce in either a clockwise or counter-clockwise manner (Do not make a "cross"). Press `q` to close the window and save the selected points to the specified pcd-file `pc-neighbor-centers.pcd`.
