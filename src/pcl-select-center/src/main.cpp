#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <fstream>

using namespace std;

float limX = 0.3f;
float limY = 0.15f;
float limZ = 0.1f;

float pickX;
float pickY;
float pickZ;
bool pointPicked = false;

static void pointPickingEvent (const pcl::visualization::PointPickingEvent &event,void*viewer_void) {
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  float x,y,z;
  event.getPoint(x,y,z);
  pcl::console::print_value("x: %f\t",x);
  pcl::console::print_value("y: %f\t",y);
  pcl::console::print_value("z: %f\n",z);

  // Remove existing rectangle
  viewer->removeShape("line");
  // Add new rectangle at selected point
  viewer->addCube(x-0.001, x+0.001, y-0.001, y+0.001, z-limZ, z+limZ, 1.0, 0.0, 1.0, "line");
  pickX = x;
  pickY = y;
  pickZ = z;
  pointPicked = true;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewCloud (
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud, bool registerPicks = false)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0.08627451,0.08627451,0.11372549); // Eigengrau
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);

  if (registerPicks) {
    viewer->registerPointPickingCallback(pointPickingEvent, (void*)viewer.get());
  }

  return (viewer);
}


int main(int argc, char *argv[])
{

  std::string filename = argv[1];
  std::string filenameOut = argv[2];

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::io::loadPCDFile(filename, *cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = viewCloud(cloud, true);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  viewer->close();

  if(pointPicked) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudExtracted (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // Find points within rectangle
    pcl::PointIndices::Ptr indices_in (new pcl::PointIndices);

    for (int i = 0; i < cloud->points.size(); i++) {
      if ((cloud->points[i].x > pickX-limX) && (cloud->points[i].x < pickX+limX) && (cloud->points[i].y > pickY-limY) && (cloud->points[i].y < pickY+limY)) {
        indices_in->indices.push_back(i);
      }
    }

    // Extract cloud based on indicies
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> eifilter (true);
    eifilter.setInputCloud (cloud);
    eifilter.setIndices (indices_in);
    eifilter.filter (*cloudExtracted);

    pcl::console::print_info("Writing center to csv\n");
    FILE * pFile;
    pFile = fopen(filenameOut.c_str(),"w");
    fprintf(pFile,"%.8e,%.8e,%.8e\n",pickX,pickY,pickZ);
    fclose(pFile);

  }

  return 0;
}
