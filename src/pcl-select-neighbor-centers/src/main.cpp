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

bool addPointToNeighbors;

static void pointPickingEvent (const pcl::visualization::PointPickingEvent &event,void*viewer_void) {
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  float x,y,z;
  event.getPoint(x,y,z);
  pcl::console::print_value("x: %f\t",x);
  pcl::console::print_value("y: %f\t",y);
  pcl::console::print_value("z: %f\n",z);

  // Remove existing rectangle
  viewer->removeShape("pointPickedLine");
  // Add new rectangle at selected point
  viewer->addCube(x-0.001, x+0.001, y-0.001, y+0.001, z-limZ, z+limZ, 1.0, 0.0, 1.0, "pointPickedLine");
  pickX = x;
  pickY = y;
  pickZ = z;
  pointPicked = true;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

  /*
  if (event.keyDown())
    {
      pcl::console::print_highlight("Key pressed: "); pcl::console::print_value("Down\n");
    }
  if (event.keyUp())
    {
      pcl::console::print_highlight("Key released: "); pcl::console::print_value("Up\n");
    }

  pcl::console::print_info("  Key symbol: "); pcl::console::print_value("%s\n",event.getKeySym().c_str());
  pcl::console::print_info("  Key code: "); pcl::console::print_value("%d\n",event.getKeyCode());
  pcl::console::print_info("  Ctrl : "); pcl::console::print_value("%d\n", event.isCtrlPressed());
  pcl::console::print_info("  Alt  : "); pcl::console::print_value("%d\n", event.isAltPressed());
  pcl::console::print_info("  Shift: "); pcl::console::print_value("%d\n", event.isShiftPressed());
  */

  if (event.keyDown())
    {
      if (!event.getKeySym().compare("a")) // Change viewing
        {
          addPointToNeighbors = true;
        }
    }
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
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get());
  }

  return (viewer);
}


int main(int argc, char *argv[])
{

  std::string filename = argv[1];
  std::string centerCSV = argv[2];
  std::string filenameOut = argv[3];

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::io::loadPCDFile(filename, *cloud);

  float centerX;
  float centerY;
  float centerZ;
  FILE * centerFile;
  int coordinateStringMaxLength = 100;
  char mystring [coordinateStringMaxLength];
  centerFile = fopen (centerCSV.c_str() , "r");
  if (centerFile != NULL)
    {
      if ( fgets(mystring , coordinateStringMaxLength , centerFile) != NULL )
        {
          // Convert to string
          std::string coordinateString;
          for (int i = 0; i < coordinateStringMaxLength; i++)
            {
              coordinateString.push_back(mystring[i]);
            }
          // Only grab first line minus newline character
          coordinateString = coordinateString.substr(0,coordinateString.find("\n"));

          // Grab first coordinate
          int firstDelimiterIdx = coordinateString.find(",",0);
          std::string centerXstring = coordinateString.substr(0,firstDelimiterIdx);
          // Grab second coordinate
          int secondDelimiterIdx = coordinateString.find(",",firstDelimiterIdx+1);
          std::string centerYstring = coordinateString.substr(firstDelimiterIdx+1,secondDelimiterIdx-firstDelimiterIdx-1);
          // Grab third coordinate
          std::string centerZstring = coordinateString.substr(secondDelimiterIdx+1,-1);

          // Store center as floats
          centerX = strtof(centerXstring.c_str(),NULL);
          centerY = strtof(centerYstring.c_str(),NULL);
          centerZ = strtof(centerZstring.c_str(),NULL);
        }
      else
        {
          pcl::console::print_info("Could not read CSV-file!\n");
          return 0;
        }
    }
  else
    {
      pcl::console::print_info("Could not open CSV-file for reading!\n");
      return 0;
    }
  fclose (centerFile);
  pcl::console::print_info("Center point: ");
  pcl::console::print_value("(%f,",centerX);
  pcl::console::print_value("%f,",centerY);
  pcl::console::print_value("%f)\n",centerZ);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //viewer = normalsVis(cloud);
  //viewer = normalsVis2(cloudColoredByNormals);
  viewer = viewCloud(cloud, true);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNeighbors (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ pointNeighbor;
  pcl::PointXYZ plantNeighbor2Point1;
  plantNeighbor2Point1.x = centerX;
  plantNeighbor2Point1.y = centerY;
  plantNeighbor2Point1.z = centerZ-0.1;
  pcl::PointXYZ plantNeighbor2Point2;
  plantNeighbor2Point2.x = centerX;
  plantNeighbor2Point2.y = centerY;
  plantNeighbor2Point2.z = centerZ+0.2;
  viewer->addLine(plantNeighbor2Point1, plantNeighbor2Point2,0.0,0.0,1.0,"Plant line #2");

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    if (addPointToNeighbors)
      {
        pointNeighbor.x = pickX;
        pointNeighbor.y = pickY;
        pointNeighbor.z = pickZ;
        cloudNeighbors->points.push_back(pointNeighbor);
        pcl::console::print_info("Add point to neighbor list:\n");
        pcl::console::print_value("x: %f\t",pickX);
        pcl::console::print_value("y: %f\t",pickY);
        pcl::console::print_value("z: %f\n",pickZ);

        viewer->addCube(pickX-0.001, pickX+0.001, pickY-0.001, pickY+0.001, pickZ-limZ, pickZ+limZ, 0.0, 1.0, 1.0, "Neighbor" + std::to_string(cloudNeighbors->points.size()-1));
        pointNeighbor.z = pointNeighbor.z + limZ;
        viewer->addText3D("N" + std::to_string(cloudNeighbors->points.size()-1) ,pointNeighbor,0.01,0.0,0.0,1.0,"N" + std::to_string(cloudNeighbors->points.size()-1));
        addPointToNeighbors = false;
      }
  }
  viewer->close();
  cloudNeighbors->width = cloudNeighbors->points.size();
  cloudNeighbors->height = 1;
  pcl::console::print_value("Number of neighbor points: %d\n", cloudNeighbors->points.size());

  if (cloudNeighbors->points.size() > 0)
    {
      pcl::console::print_info("Saving neighbor centers to file: ");
      pcl::console::print_value("%s \n", filenameOut.c_str());
      pcl::io::savePCDFileBinary(filenameOut, *cloudNeighbors);
    }

  /*
  if(pointPicked) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudExtracted (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // Find points within rectangle
    //boost::shared_ptr< const std::vector< int > > indices_in;
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

    //pcl::io::savePCDFileBinary(filenameOut, *cloudExtracted);
    //pcl::io::savePLYFileBinary(filenameOut, *cloudExtracted);

  }
  */

  return 0;
}
