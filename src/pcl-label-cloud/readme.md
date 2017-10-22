# pcl-label-cloud
`pcl-label-cloud` lets the user label a 3D coloured point. Points can be labelled as one of the following three classes:
<table>
<tr>
<th>Class</th>
<th>Colour</th>
<th>Description</th>
</tr>
<tr>
<td>Background</td>
<td style="color:Red">Red</td>
<td style="padding-bottom:10px;">Any point, which do not fall in to one of the following categories.<br>
By default, all points are labelled as background.</td>
</tr>
<tr>
<td>Lettuce-of-interest</td>
<td style="color:Green">Green</td>
<td style="padding-bottom:10px;">The lettuce-of-interest is the lettuce, which are segmented specified by the center.csv. All points belonging to the lettuce-of-interest should be labelled with this class.</td>
</tr>
<tr>
<td>Adjacent lettuce</td>
<td style="color:Blue">Blue</td>
<td style="padding-bottom:10px;">Points belonging to lettuce directy adjacent to the lettuce-of-interest.</td>
</tr>
</table>

## Usage
The following section shows how to use `pcl-label-cloud`. The example is based on the provided examples and assumes, that a terminal window is opened in one of the data example folders.

### Label point cloud
In a terminal window, type:
```
pcl-label-cloud pc-scaled.pcd labels.pcd
```
This will load the point cloud `pc-scaled.pdc` and display it ready for being labelled.
If the output `labels.pcd` already exists, it will be loaded as well to allow the user to continue labelling.
Selecting a point using `shift` + `left mouse` label all points within the currently specified radius as the currently selected class. The current radius and class is displayed in the lower left corner.


To increase or decrease the radius by a factor of 2, use `w` or `s`. respectively.

To change class, use `a` or `d`.