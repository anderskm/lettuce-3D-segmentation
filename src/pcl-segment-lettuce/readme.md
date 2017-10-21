# pcl-segment-lettuce
`pcl-segment-lettuce` segments lettuce-of-interest from coloured 3D point clouds.
Below, the most common usage of `pcl-segment-lettuce` is presented.

## Usage
The following sections show examples of how to use `pcl-segment-lettuce`. The examples are based on the provided cos-example and assumes, that a terminal window is opened in the `data` folder.

### Help
Display help and input arguments:
```
pcl-segment-lettuce --help
```

### Show segmentation steps
Show the point cloud and the segmentation steps in a interactive window, before closing the program.

```
pcl-segment-lettuce --input-file cos-example/pc-scaled.pcd --plant-csv cos-example/center.csv --bed-neighbor-locations cos-example/pc-neighbor-centers.pcd --show-clouds 1
```

Use `page-up`, `page-down`, `home` and `end` to skip between the steps in the segmentation process.
A short description of each step/point cloud is displayed in the window title.
For more information about the functionality of the visualizer, press `h` while it is open.

### Segment and save lettuce-of-interest
Segment lettuce-of-interest from a point cloud (cos-example/pc-scaled.pcd) and save the segmented cloud (cos-example/pc-plant.pcd).
```
pcl-segment-lettuce --input-file cos-example/pc-scaled.pcd --plant-csv cos-example/center.csv --bed-neighbor-locations cos-example/pc-neighbor-centers.pcd --output-file cos-example/pc-plant.pcd
```

### Segment and compare to ground truth labels
Segment a point cloud and compare it to a labelled point cloud.
Providing a labelled point cloud, outputs a confusion matrix as well as precision, recall and F1-score for the provided point cloud.

```
pcl-segment-lettuce --input-file cos-example/pc-scaled.pcd --plant-csv cos-example/center.csv --bed-neighbor-locations cos-example/pc-neighbor-centers.pcd --labels cos-example/labels.pcd
```

Example output:
```
Confusion matrix:
         :    Lettuce      Other   :     Total 
.........:. .......... .......... .:............
Other    :        759      95741   :     96500 
Lettuce  :      99359      22549   :    121908 
Neighbor :       3444     128712   :    132156 
.........:. .......... .......... .:............
Total    :     103562     247002   :    350564 

Precision: 0.959416
Recall   : 0.815033
F1-score : 0.881350

```

### Segment using optimized settings
To segment the provided examples using the same settings used in the paper, use the following commands for Cos and Iceberg, respectively. Replace `<other-input-arguments>` with .

** Cos: **
```
pcl-segment-lettuce <other-input-arguments> --plant-height-th 4.8723 --leaf-smooth-th 4.1861 --leaf-inner-dist-th 0.15214 --leaf-outer-dist-th 0.18838 --leaf-min-size 542
```

** Iceberg: **
```
pcl-segment-lettuce <other-input-arguments> --plant-height-th 3.8647 --leaf-smooth-th 4.1719 --leaf-inner-dist-th 0.15304 --leaf-outer-dist-th 0.16766 --leaf-min-size 717
```