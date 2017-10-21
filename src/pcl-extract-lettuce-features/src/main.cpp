#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkMassProperties.h>

#include "pcloader.h"

#include <iostream>
#include <fstream>

#include <stdio.h>

float extractConvexHullVolume(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloudConvHull, pcl::PolygonMesh::Ptr meshConvHull, bool addStemPoint)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  copyPointCloud(*cloud, *_cloud);

  if (addStemPoint)
    {
      pcl::PointXYZRGBNormal stemPoint;
      stemPoint.x = 0;
      stemPoint.y = 0;
      stemPoint.z = 0;
      _cloud->points.push_back (stemPoint);
    }

  _cloud->width = _cloud->points.size();
  _cloud->height = 1;

  pcl::ConvexHull<pcl::PointXYZRGBNormal> convHull;
  convHull.setInputCloud(_cloud);
  convHull.setComputeAreaVolume(true);
  convHull.reconstruct(*cloudConvHull); // Reconstruct must called before getting volume and area
  convHull.reconstruct(*meshConvHull);
  return convHull.getTotalVolume();
}
float extractConvexHullArea(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloudConvHull, bool addStemPoint)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  copyPointCloud(*cloud, *_cloud);

  if (addStemPoint)
    {
      pcl::PointXYZRGBNormal stemPoint;
      stemPoint.x = 0;
      stemPoint.y = 0;
      stemPoint.z = 0;
      _cloud->points.push_back (stemPoint);
    }

  _cloud->width = _cloud->points.size();
  _cloud->height = 1;

  pcl::ConvexHull<pcl::PointXYZRGBNormal> convHull;
  convHull.setInputCloud(_cloud);
  convHull.setComputeAreaVolume(true);
  convHull.reconstruct(*cloudConvHull); // Reconstruct must called before getting volume and area

  return convHull.getTotalArea();
}

float getHeight(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud)
{
  // Get the highest point in the cloud
  float height = 0;
  for (int i = 0; i < cloud->points.size(); i++)
    {
      if (cloud->points[i].z > height)
        {
          height = cloud->points[i].z;
        }
    }
  return height;
}

void addMirrorY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloudOut)
{
  pcl::copyPointCloud(*cloudIn, *cloudOut);

  pcl::PointXYZRGBNormal point;
  for (int i = 0; i < cloudIn->points.size(); i++)
    {
      point = cloudIn->points[i];
      point.x = -point.x;
      cloudOut->points.push_back(point);
    }
  cloudOut->width = cloudOut->points.size();
  cloudOut->height = 1;
}

void project2ground(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloudOut)
{
  pcl::copyPointCloud(*cloudIn, *cloudOut);


  for (int i = 0; i < cloudOut->points.size(); i++)
    {
      cloudOut->points[i].z = 0;
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> dualVis (
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud1, pcl::PolygonMesh::ConstPtr mesh)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);


  int v1 = 0;
  viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(1.0,0.0,1.0, v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb1(cloud1);
  viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud1, rgb1, "Cloud #1",v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud #1", v1);

  int v2 = 0;
  viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  viewer->setBackgroundColor(1.0,0.0,1.0, v2);
  viewer->addPolygonMesh(*mesh,"Mesh #2",v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Mesh #2", v2);
  viewer->addCoordinateSystem (0.1, "Mesh #2", v2);

  return (viewer);
}

int main(int argc, char *argv[])
{

  std::string filename = argv[1];
  std::string filenameOut;
  bool saveOutput = false;
  if (argc >= 3) {
     filenameOut = argv[2];
     saveOutput = true;
  }

  pcl::console::print_highlight("SOUPY");
  pcl::console::print_info("Input file: ");
  pcl::console::print_value("%s\n", filename.c_str());
  pcl::console::print_info("Output file: ");
  if (saveOutput) {
    pcl::console::print_value("%s\n", filenameOut.c_str());
  }
  else {
      pcl::console::print_value("%s\n","No output file specified!");
  }

  // Load cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcLoader loader(filename, true);
  loader.load();
  loader.getCloud(*cloud);

  // Convex Hull Volume
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConvHull (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PolygonMesh::Ptr meshConvHull (new pcl::PolygonMesh);
  float convexHullVolume = 0;
  convexHullVolume = extractConvexHullVolume(cloud, cloudConvHull, meshConvHull, false);
  pcl::console::print_info("Convex Hull Volume: "); pcl::console::print_value("%f\n", convexHullVolume);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConvHullStemPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PolygonMesh::Ptr meshConvHullStemPoint (new pcl::PolygonMesh);
  float convexHullVolumeStemPoint = 0;
  convexHullVolumeStemPoint = extractConvexHullVolume(cloud, cloudConvHullStemPoint, meshConvHullStemPoint, true);
  pcl::console::print_info("Convex Hull Volume (stem point): "); pcl::console::print_value("%f\n", convexHullVolumeStemPoint);

  /*
   * SURFACE AREA
   */

  // Convex hull surface area
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConvHullArea (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PolygonMesh::Ptr meshConvHullArea (new pcl::PolygonMesh);
    float convexHullArea = 0;
    convexHullArea = extractConvexHullArea(cloud, cloudConvHullArea, false);
    pcl::console::print_info("Convex Hull Area: "); pcl::console::print_value("%f\n", convexHullArea);

  // Convex hull (stem point) surface area
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConvHullAreaStemPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PolygonMesh::Ptr meshConvHullAreaStemPoint (new pcl::PolygonMesh);
    float convexHullAreaStemPoint = 0;
    convexHullAreaStemPoint = extractConvexHullArea(cloud, cloudConvHullAreaStemPoint, true);
    pcl::console::print_info("Convex Hull Area (stem point): "); pcl::console::print_value("%f\n", convexHullAreaStemPoint);

  // Voxel
  pcl::console::print_info("Voxelization\n");
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudVoxelized (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> Voxelizer;
  Voxelizer.setInputCloud(cloud);
  float voxelSize = 0.005; // 5 mm
  Voxelizer.setLeafSize (voxelSize, voxelSize, voxelSize);
  Voxelizer.filter(*cloudVoxelized);

  pcl::console::print_info("  Voxel size: "); pcl::console::print_value("% 14.3e\n", voxelSize);
  pcl::console::print_info("  Voxels    : "); pcl::console::print_value("% 6d\n", cloudVoxelized->points.size());
  pcl::console::print_info("  Area      : "); pcl::console::print_value("% 14.3e\n", cloudVoxelized->points.size()*voxelSize*voxelSize*100*100);
  pcl::console::print_info("  Volume    : "); pcl::console::print_value("% 14.3e\n", cloudVoxelized->points.size()*voxelSize*voxelSize*voxelSize*100*100*100);

  // Concave hull

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConcaveHull (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PolygonMesh::Ptr meshConcaveHull (new pcl::PolygonMesh);
  pcl::ConcaveHull<pcl::PointXYZRGBNormal> concHull;
  concHull.setInputCloud (cloud);
  concHull.setAlpha (0.02);
  concHull.setKeepInformation(true);
  concHull.reconstruct(*cloudConcaveHull);
  concHull.reconstruct(*meshConcaveHull);

  vtkSmartPointer< vtkPolyData > triangles_out_vtk;
  pcl::VTKUtils::convertToVTK(*meshConcaveHull,triangles_out_vtk);

  vtkSmartPointer< vtkMassProperties > massProp = vtkSmartPointer< vtkMassProperties >::New();
  massProp->SetInputData(triangles_out_vtk);
  double concaveHullArea = massProp->GetSurfaceArea();
  double concaveHullVolume = massProp->GetVolume();
  pcl::console::print_value("Concave area: %14.3e\n",concaveHullArea);
  pcl::console::print_value("Concave volume: %14.3e\n",concaveHullVolume);

  /*
   * Leaf Cover Area
   */

  // LCA
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudGround (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  project2ground(cloud, cloudGround);

  // Voxelization
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudGroundVoxelized (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> VoxelizerGround;
  VoxelizerGround.setInputCloud(cloudGround);
  float voxelSizeGround = 0.005; // 5 mm
  VoxelizerGround.setLeafSize (voxelSizeGround, voxelSizeGround, voxelSizeGround);
  VoxelizerGround.filter(*cloudGroundVoxelized);
  pcl::console::print_info("LCA, Voxelization: "); pcl::console::print_value("%f\n", cloudGroundVoxelized->points.size()*voxelSizeGround*voxelSizeGround);

  // Convex hull
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudGroundConvHull (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  float convexHullGroundArea = 0;
  convexHullGroundArea = extractConvexHullArea(cloudGround, cloudGroundConvHull, false);
  pcl::console::print_value("LCA, Convex hull area: %14.3e\n",convexHullGroundArea);

  /*
   * HEIGHT
   */

  // Height
  float height = getHeight(cloud);
  pcl::console::print_info("Height: "); pcl::console::print_value("%f\n", height);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudConcaveHull2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PolygonMesh::Ptr meshConcaveHull2 (new pcl::PolygonMesh);
  pcl::ConcaveHull<pcl::PointXYZRGBNormal> concHull2;
  concHull2.setInputCloud (cloudGround);
  concHull2.setAlpha (0.02);
  concHull2.setKeepInformation(true);
  pcl::console::print_info("Reconstruct\n");
  concHull2.reconstruct(*cloudConcaveHull2);
  concHull2.reconstruct(*meshConcaveHull2);

  vtkSmartPointer< vtkPolyData > triangles_out_vtk2;
  pcl::console::print_info("Convert to VTK\n");
  pcl::VTKUtils::convertToVTK(*meshConcaveHull2,triangles_out_vtk2);

  vtkSmartPointer< vtkMassProperties > massProp2 = vtkSmartPointer< vtkMassProperties >::New();
  pcl::console::print_info("Set input cloud\n");
  massProp2->SetInputData(triangles_out_vtk2);
  pcl::console::print_info("Get area\n");
  double concaveHullArea2 = massProp2->GetSurfaceArea();
  pcl::console::print_value("LCA, Concave area: %14.3e\n",concaveHullArea2);

  if (!saveOutput)
    {
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
      viewer2 = dualVis(cloud, meshConcaveHull2);

      while (!viewer2->wasStopped ())
      {
        viewer2->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    }
  if (saveOutput)
    {
      pcl::console::print_info("Writing features to CSV...\n");
      FILE * pFile;
      pFile = fopen(filenameOut.c_str(),"w");

      fprintf(pFile,"%s,%.8e,%s\n","Vol_ConvexHull",convexHullVolume,"m^3");
      fprintf(pFile,"%s,%.8e,%s\n","Vol_ConvexHullStemPoint",convexHullVolumeStemPoint,"m^3");
      fprintf(pFile,"%s,%.8e,%s\n","Vol_ConcaveHullVolume",concaveHullVolume,"m^3");

      fprintf(pFile,"%s,%.8e,%s\n","SA_ConvexHull",convexHullArea,"m^2");
      fprintf(pFile,"%s,%.8e,%s\n","SA_ConvexHullStemPoint",convexHullAreaStemPoint,"m^2");
      fprintf(pFile,"%s,%.8e,%s\n","SA_ConcaveHullArea",concaveHullArea,"m^2");
      fprintf(pFile,"%s,%d,%s\n"  ,"SA_VoxelCount",(int)cloudVoxelized->points.size(),"no unit");

      fprintf(pFile,"%s,%.8e,%s\n","LCA_ConvexHull",convexHullGroundArea,"m^2");
      fprintf(pFile,"%s,%.8e,%s\n","LCA_ConcaveHull",concaveHullArea2,"m^2");
      fprintf(pFile,"%s,%.8e,%s\n","LCA_Voxelization",cloudGroundVoxelized->points.size()*voxelSizeGround*voxelSizeGround,"m^2");

      fprintf(pFile,"%s,%.8e,%s\n","Height",height,"m");

      fclose(pFile);
    }
}

