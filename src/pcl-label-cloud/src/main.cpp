#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pcloader.h"

#include <string>
#include <algorithm>
#include <fstream>


bool pointPicked = false;
int pointPickIdx;
bool coloringChanged = false;
bool increaseRadius = false;
bool decreaseRadius = false;
bool increaseClass = false;
bool decreaseClass = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

  if (event.keyDown())
    {
      if (!event.getKeySym().compare("v")) // Change viewing
        {
          coloringChanged = true;
        }
      if (!event.getKeySym().compare("w"))
        {
          increaseRadius = true;
        }
      if (!event.getKeySym().compare("s"))
        {
          decreaseRadius = true;
        }
      if (!event.getKeySym().compare("a"))
        {
          decreaseClass = true;
        }
      if (!event.getKeySym().compare("d"))
        {
          increaseClass = true;
        }
    }

}

static void pointPickingEvent (const pcl::visualization::PointPickingEvent &event,void*viewer_void) {
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  float x,y,z;
  event.getPoint(x,y,z);
  pointPicked = true;
  pointPickIdx = event.getPointIndex();
}

inline bool fileExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

int main(int argc, char *argv[])
{

  float labelRadius = 0.16; // m
  bool colorByLabel = false;

  std::vector<std::string> classes;
  classes.push_back("Unknown"); // DO NOT INSERT; ONLY ADD AT THE END!!
  classes.push_back("Lettuce of interest");  // DO NOT INSERT; ONLY ADD AT THE END!!
  classes.push_back("Other lettuce");  // DO NOT INSERT; ONLY ADD AT THE END!!
  std::vector<std::string>::iterator classesIterator;
  classesIterator = classes.begin();

  std::string filename = argv[1];
  std::string filenameOut;
  bool saveOutput = false;
  if (argc >= 3) {
     filenameOut = argv[2];
     saveOutput = true;
  }

  pcl::console::print_info("Input file: ");
  pcl::console::print_value("%s\n", filename.c_str());
  if (saveOutput) {
      pcl::console::print_info("Output file: ");
      pcl::console::print_value("%s\n", filenameOut.c_str());
    }

  pcl::console::print_info("Output file: ");
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloudLabels (new pcl::PointCloud<pcl::PointXYZRGBL>);
  bool existingLabelFile = false;
  if (saveOutput) {
    pcl::console::print_value("%s\n", filenameOut.c_str());
    if (fileExists(filenameOut))
      {
        pcl::console::print_highlight("Output file already exists. Loading file to continue labeling on that...\n");
        existingLabelFile = true;
        pcl::io::loadPCDFile(filenameOut, *cloudLabels);
        labelRadius = 0.02; // Set to 2 cm to avoid accidentially messing too much with existing labels
        colorByLabel = true;
      }
  }
  else {
      pcl::console::print_value("%s\n","No output file specified!");
  }

  // Load point cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudFull (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcLoader loader(filename, true);
  loader.load();
  loader.getCloud(*cloudFull);

  if (!existingLabelFile)
    {
      // Create label cloud
      pcl::PointXYZRGBL pointLabel;
      for (int i = 0; i < cloudFull->points.size(); i++)
        {
          pointLabel.x = cloudFull->points[i].x;
          pointLabel.y = cloudFull->points[i].y;
          pointLabel.z = cloudFull->points[i].z;
          pointLabel.rgb = cloudFull->points[i].rgb;
          pointLabel.label = 0;
          cloudLabels->points.push_back(pointLabel);
        }
      cloudLabels->width = cloudLabels->points.size();
      cloudLabels->height = 1;
    }

  // Setup KD-tree for NN search
  pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
  kdtree.setInputCloud(cloudLabels);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Label point cloud"));
  viewer->setBackgroundColor(0.08627451,0.08627451,0.11372549); // Eigengrau
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(cloudLabels);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
  viewer->registerPointPickingCallback(pointPickingEvent, (void*)viewer.get());
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get());

  bool updateCloud = true; // Update in first loop
  bool updateText = true; // Add the text during first loop
  while (!viewer->wasStopped ())
    {
      // Sleep for 100 ms to allow the user to perform actions
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));

      // Check if a point has been picked
      if (pointPicked)
        {
          pcl::PointXYZRGBL pickedPoint;
          pickedPoint = cloudLabels->points[pointPickIdx];
          pcl::console::print_info("Point picked:\n");
          pcl::console::print_info("   (x,y,z) = ");
          pcl::console::print_value("(%f,", pickedPoint.x);
          pcl::console::print_value("%f,", pickedPoint.y);
          pcl::console::print_value("%f)\n", pickedPoint.z);

          pcl::console::print_info("   Old label: ");
          pcl::console::print_value("%s\n", classes[pickedPoint.label].c_str());
          pcl::console::print_info("   New label: ");
          pcl::console::print_value("%s\n", classes[classesIterator - classes.begin()].c_str());

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          if (kdtree.radiusSearchT(pickedPoint, labelRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
              pcl::console::print_value("   %d points",pointIdxRadiusSearch.size());

              for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
                {
                  cloudLabels->points[pointIdxRadiusSearch[i]].label = classesIterator - classes.begin();
                }
            }
          else
            {
              pcl::console::print_info("   0 points");
            }
          pcl::console::print_value(" within a radius of %f m.\n", labelRadius);

          updateCloud = true;
          pointPicked = false;
        }

      if (coloringChanged)
        {
          colorByLabel ^= 1;
          coloringChanged = false;
          updateCloud = true;
        }
      if (decreaseRadius)
        {
          labelRadius /= 2;
          decreaseRadius = false;
          updateText = true;
          pcl::console::print_info("Label radius: ");
          pcl::console::print_value("%f\n",labelRadius);
        }
      if (increaseRadius)
        {
          labelRadius *= 2;
          increaseRadius = false;
          updateText = true;
          pcl::console::print_info("Label radius: ");
          pcl::console::print_value("%f\n",labelRadius);
        }
      if (increaseClass)
        {
          if (classesIterator+1 < classes.end())
            {
              classesIterator++;
              updateText = true;
            }
          pcl::console::print_info("Label: ");
          pcl::console::print_value("%s\n", classes[classesIterator - classes.begin()].c_str());
          increaseClass = false;
        }
      if (decreaseClass)
        {
          if (classesIterator-1 >= classes.begin())
            {
              classesIterator--;
              updateText = true;
            }
          pcl::console::print_info("Label: ");
          pcl::console::print_value("%s\n", classes[classesIterator - classes.begin()].c_str());
          decreaseClass = false;
        }
      if (updateText)
        {
          viewer->removeText3D("classLabel");
          viewer->removeText3D("classLabelValue");
          viewer->removeText3D("radius");
          viewer->removeText3D("radiusValue");
          float r = 1.0;
          float g = 1.0;
          float b = 1.0;
          viewer->addText("Class: ", 20, 50, 20, 1.0, 1.0, 1.0, "classLabel");
          if (classesIterator - classes.begin() == 0)
            {
              r = 1.0;
              g = 0.0;
              b = 0.0;
            }
          else if (classesIterator - classes.begin() == 1)
            {
             r = 0.0;
             g = 1.0;
             b = 0.0;
            }
          else if (classesIterator - classes.begin() == 2)
            {
             r = 0.0;
             g = 0.0;
             b = 1.0;
            }
          else if (classesIterator - classes.begin() == 3)
            {
             r = 1.0;
             g = 1.0;
             b = 0.0;
            }
          else if (classesIterator - classes.begin() == 4)
            {
             r = 1.0;
             g = 0.0;
             b = 1.0;
            }
          else if (classesIterator - classes.begin() == 5)
            {
             r = 0.0;
             g = 1.0;
             b = 1.0;
            }
          else
            {
             r = 1.0;
             g = 1.0;
             b = 1.0;
            }
          viewer->addText(classes[classesIterator - classes.begin()],100,50,20,r, g, b,"classLabelValue");
          viewer->addText("Radius: ", 20, 20, 20, 1.0, 1.0, 1.0, "radius");
          viewer->addText(std::to_string(labelRadius) + " m", 100, 20, 20, 1.0, 1.0, 1.0, "radiusValue");
          updateText = false;
        }

      if (updateCloud)
        {
          viewer->removeAllPointClouds();
          if (colorByLabel)
            {
              for (int i = 0; i < cloudLabels->points.size(); i++)
                {
                  float intensity_f = ((float)cloudFull->points[i].r + (float)cloudFull->points[i].g + (float)cloudFull->points[i].b)/(255*3);
                  uint8_t intensity = (uint8_t)std::min(255.0,255.0*intensity_f);
                  if (cloudLabels->points[i].label == 0)
                    {
                      cloudLabels->points[i].r = intensity;
                      cloudLabels->points[i].g = 0;
                      cloudLabels->points[i].b = 0;
                    }
                  if (cloudLabels->points[i].label == 1)
                    {
                      cloudLabels->points[i].r = 0;
                      cloudLabels->points[i].g = intensity;
                      cloudLabels->points[i].b = 0;
                    }
                  if (cloudLabels->points[i].label == 2)
                    {
                      cloudLabels->points[i].r = 0;
                      cloudLabels->points[i].g = 0;
                      cloudLabels->points[i].b = intensity;
                    }
                  if (cloudLabels->points[i].label == 3)
                    {
                      cloudLabels->points[i].r = intensity;
                      cloudLabels->points[i].g = intensity;
                      cloudLabels->points[i].b = 0;
                    }
                  if (cloudLabels->points[i].label == 4)
                    {
                      cloudLabels->points[i].r = intensity;
                      cloudLabels->points[i].g = 0;
                      cloudLabels->points[i].b = intensity;
                    }
                  if (cloudLabels->points[i].label == 5)
                    {
                      cloudLabels->points[i].r = 0;
                      cloudLabels->points[i].g = intensity;
                      cloudLabels->points[i].b = intensity;
                    }
                  if (cloudLabels->points[i].label == 6)
                    {
                      cloudLabels->points[i].r = intensity;
                      cloudLabels->points[i].g = intensity;
                      cloudLabels->points[i].b = intensity;
                    }
                }
            }
          else
            {
              for (int i = 0; i < cloudLabels->points.size(); i++)
                {
                  cloudLabels->points[i].r = cloudFull->points[i].r;
                  cloudLabels->points[i].g = cloudFull->points[i].g;
                  cloudLabels->points[i].b = cloudFull->points[i].b;
                }
            }
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(cloudLabels);
          viewer->addPointCloud<pcl::PointXYZRGBL> (cloudLabels, rgb, "Cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");
          updateCloud = false;
        }
    }



  if (saveOutput)
    {
      pcl::io::savePCDFileBinary(filenameOut, *cloudLabels);
    }

  return 0;
}
