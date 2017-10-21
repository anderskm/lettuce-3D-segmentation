#include "pcviewer.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

/***************************
 *                         *
 *         PCViewer        *
 *                         *
 ***************************/

namespace PCViewer
{
  std::vector<PCViewerCloud>::iterator PCViewerCloudsIterator;
  bool cloudChangeFirst = false;
  bool cloudChangeLast = false;
  bool cloudChangeNext = false;
  bool cloudChangePrior = false;
  bool pointPicked = false;
  bool normalDisplayChange = false;
  int pointPickIdx;

  void singleViewer_keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
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
        if (!event.getKeySym().compare("Home"))
          {
            cloudChangeFirst = true;
          }
        if (!event.getKeySym().compare("End"))
          {
            cloudChangeLast = true;
          }
        if (!event.getKeySym().compare("Next"))
          {
            cloudChangeNext = true;
          }
        if (!event.getKeySym().compare("Prior"))
          {
            cloudChangePrior = true;
          }
        if (!event.getKeySym().compare("n"))
          {
            normalDisplayChange = true;
          }
      }

  }

  static void singleViewer_pointPickingEvent (const pcl::visualization::PointPickingEvent &event,void*viewer_void) {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    float x,y,z;
    event.getPoint(x,y,z);
    pointPicked = true;
    pointPickIdx = event.getPointIndex();
  }

  PCViewer::PCViewer()
  {
    this->showNormals = false;
    this->verbose = false;
  }

  PCViewer::PCViewer(bool _verbose)
  {
    this->showNormals = false;
    this->verbose = _verbose;

    if (this->verbose)
      {
        pcl::console::print_info("PCViewer: ");
        pcl::console::print_value("Verbose: %d\n", this->verbose);
     }
  }

  void PCViewer::singleViewer()
  {
    this->singleViewer(0);
  }

  void PCViewer::singleViewer(int cloudIdx)
  {
    if (this->verbose)
      {
        pcl::console::print_info("PCViewer: ");
        pcl::console::print_value("Viewer initilized.\n");
      }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer (new pcl::visualization::PCLVisualizer ("PCViewer"));
    _viewer->initCameraParameters ();
    _viewer->setCameraPosition(0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
    _viewer->setBackgroundColor(1.0,0.0,1.0);

    pcl::PointXYZRGB _pointPick;
    _pointPick.x = 0;
    _pointPick.y = 0;
    _pointPick.z = 0;
    _pointPick.r = 0;
    _pointPick.g = 0;
    _pointPick.b = 0;
    this->pointPick = _pointPick;

    PCViewerCloudsIterator = this->PCViewerClouds.begin();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = PCViewerCloudsIterator->cloud;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloudPtr);
    _viewer->addPointCloud<pcl::PointXYZRGB> (PCViewerCloudsIterator->cloud, rgb1, "Cloud #1");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud #1");
    /*
    if (cloudIdx < this->PCViewerClouds.size())
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = this->PCViewerClouds[cloudIdx].cloud;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloudPtr);
        _viewer->addPointCloud<pcl::PointXYZRGB> (this->PCViewerClouds[cloudIdx].cloud, rgb1, "Cloud #1");
        _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud #1");
      }
  */
    _viewer->registerKeyboardCallback (singleViewer_keyboardEventOccurred, (void*)_viewer.get());
    _viewer->registerPointPickingCallback(singleViewer_pointPickingEvent, (void*)_viewer.get());

    this->viewer = _viewer;
  }

  void PCViewer::spin()
  {
    bool updateCloud = false;
    if (this->verbose)
      {
        pcl::console::print_info("PCViewer: ");
        pcl::console::print_value("Spinning viewer...\n");
      }

    while (!this->viewer->wasStopped ())
      {
        // Sleep for 100 ms to allow the user to perform actions
        this->viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        // Check if a point has been picked
        if (pointPicked)
          {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = PCViewerCloudsIterator->cloud;
            pcl::PointXYZRGB _pointPick;
            _pointPick = cloudPtr->points[pointPickIdx];
            pcl::console::print_info("PCViewer: ");
            pcl::console::print_value("Point picked:\n");
            pcl::console::print_value("   (x,y,z) = ");
            pcl::console::print_value("(%f,", _pointPick.x);
            pcl::console::print_value("%f,", _pointPick.y);
            pcl::console::print_value("%f)\n", _pointPick.z);
            pcl::console::print_value("   (r,g,b) = ");
            pcl::console::print_value("(%d,", _pointPick.r);
            pcl::console::print_value("%d,", _pointPick.g);
            pcl::console::print_value("%d)\n", _pointPick.b);
            float dx = _pointPick.x - this->pointPick.x;
            float dy = _pointPick.y - this->pointPick.y;
            float dz = _pointPick.z - this->pointPick.z;
            pcl::console::print_value("   Dist. to prev. point = %f\n",sqrt(dx*dx + dy*dy + dz*dz));
            pcl::console::print_value("   Dist. to origin      = %f\n",sqrt(_pointPick.x*_pointPick.x + _pointPick.y*_pointPick.y + _pointPick.z*_pointPick.z));
            this->pointPick = _pointPick;
            pointPicked = false;
          }
        else
          {
            updateCloud = false;
            // Check if cloud needs to be updated
            if (cloudChangeNext)
              {
                if (PCViewerCloudsIterator+1 < this->PCViewerClouds.end())
                  {
                    PCViewerCloudsIterator++;
                    updateCloud = true;
                  }
                cloudChangeNext = false;
              }

            if (cloudChangePrior)
              {
                if (PCViewerCloudsIterator-1 >= this->PCViewerClouds.begin())
                  {
                    PCViewerCloudsIterator--;
                    updateCloud = true;
                  }
                cloudChangePrior = false;
              }
            if (cloudChangeFirst)
              {
                PCViewerCloudsIterator = this->PCViewerClouds.begin();
                cloudChangeFirst = false;
                updateCloud = true;
              }

            if (cloudChangeLast)
              {
                PCViewerCloudsIterator = this->PCViewerClouds.end()-1;
                cloudChangeLast = false;
                updateCloud = true;
              }
            if (normalDisplayChange)
              {
                this->showNormals ^= 1;
                updateCloud = true;
                normalDisplayChange = false;
              }


            if (updateCloud)
              {
                this->updateViewer();
              }
          }
        // End of spin loop
      }

    if (this->verbose)
      {
        pcl::console::print_info("PCViewer: ");
        pcl::console::print_value("Viewer stopped.\n");
      }
  }

  void PCViewer::updateViewer()
  {
    this->viewer->removeAllPointClouds();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = PCViewerCloudsIterator->cloud;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloudPtr);
    this->viewer->addPointCloud<pcl::PointXYZRGB> (PCViewerCloudsIterator->cloud, rgb1, "Cloud #1");
    if ((PCViewerCloudsIterator->hasNormal) && (this->showNormals))
      {
        this->viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (PCViewerCloudsIterator->cloud, PCViewerCloudsIterator->normals,1,0.01,"normals");
          //viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> (cloud, cloud, 100, 0.01, "normals");
      }
    this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud #1");
    this->viewer->setWindowName("PCViewer: " + PCViewerCloudsIterator->name);
  }

  void PCViewer::addCloud(PCViewerCloud viewerCloud)
  {
    if (this->verbose)
      {
        pcl::console::print_info("PCViewer: ");
        pcl::console::print_value("Added cloud \"%s\"\n",viewerCloud.name.c_str());
      }
    this->PCViewerClouds.push_back(viewerCloud);
  }

  void PCViewer::addCloud(std::string _name, pcl::PointCloud<pcl::PointXY>::Ptr _cloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = _cloud->width;
    cloud->height = _cloud->height;

    cloud->points.resize(cloud->width*cloud->height);
    for (int i = 0; i < _cloud->size(); i++)
      {
        cloud->points[i].x = _cloud->points[i].x;
        cloud->points[i].y = _cloud->points[i].y;
      }

    PCViewerCloud viewerCloud(_name, cloud);
    this->addCloud(viewerCloud);
    //this->PCViewerClouds.push_back(viewerCloud);
  }

  void PCViewer::addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud, float minIntensity, float maxIntensity)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = _cloud->width;
    cloud->height = _cloud->height;

    cloud->points.resize(cloud->width*cloud->height);

    float intensityRange = maxIntensity - minIntensity;
    if (intensityRange == 0)
      {
        intensityRange = 1;
      }

    float gray;
    uint32_t rgb;
    for (int i = 0; i < _cloud->size(); i++)
      {
        cloud->points[i].x = _cloud->points[i].x;
        cloud->points[i].y = _cloud->points[i].y;
        cloud->points[i].z = _cloud->points[i].z;

        gray = _cloud->points[i].intensity;
        if (gray < minIntensity)
          gray = minIntensity;
        if (gray > maxIntensity)
          gray = maxIntensity;
        gray = 255.0*(gray - minIntensity)/intensityRange;
        rgb = (static_cast<uint32_t>(gray) << 16 |
               static_cast<uint32_t>(gray) << 8 |
               static_cast<uint32_t>(gray));
        cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
      }

    PCViewerCloud viewerCloud(_name, cloud);
    this->addCloud(viewerCloud);
    //this->PCViewerClouds.push_back(viewerCloud);
  }
  void PCViewer::addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud)
  {
    float maxIntensity = _cloud->points[0].intensity;
    float minIntensity = _cloud->points[0].intensity;
    for (int i = 0; i < _cloud->size(); i++)
      {
        if (_cloud->points[i].intensity > maxIntensity)
          {
            maxIntensity = _cloud->points[i].intensity;
          }
        if (_cloud->points[i].intensity < minIntensity)
          {
            minIntensity = _cloud->points[i].intensity;
          }
      }

    pcl::console::print_value("PCViewer: Min intensity: %f\n",minIntensity);
    pcl::console::print_value("PCViewer: Max intensity: %f\n",maxIntensity);
    this->addCloud(_name, _cloud, minIntensity, maxIntensity);
  }
  void PCViewer::addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZL>::Ptr _cloud)
  {
    // Find labels
    std::vector<int> labels;
    for (int i = 0; i < _cloud->points.size(); i++)
      {
         labels.push_back(_cloud->points[i].label);
      }

    // Find unique labels
    std::sort (labels.begin(), labels.end());
    std::vector<int>::iterator it;
    it = std::unique(labels.begin(), labels.end());
    labels.resize( std::distance(labels.begin(),it) );

    // Generate random colors for the labels
    std::vector<PCViewerColor> labelColors;
    for (int i = 0; i < labels.size(); i++)
      {
        // Add a random color;
        PCViewerColor labelColor;
        labelColors.push_back(labelColor);
      }


    // Copy and color cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = _cloud->width;
    cloud->height = _cloud->height;
    cloud->points.resize(cloud->width*cloud->height);
    uint32_t rgb;
    for (int i = 0; i < _cloud->points.size(); i++)
      {
        cloud->points[i].x = _cloud->points[i].x;
        cloud->points[i].y = _cloud->points[i].y;
        cloud->points[i].z = _cloud->points[i].z;

        PCViewerColor labelColor;
        for (int l = 0; l < labels.size(); l++)
          {
            if (labels[l] == _cloud->points[i].label)
              {
                labelColor = labelColors[l];
              }
          }
        rgb = ((uint32_t)(labelColor.r) << 16 |
               (uint32_t)(labelColor.g) << 8 |
               (uint32_t)(labelColor.b));

        cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
      }


    PCViewerCloud viewerCloud(_name, cloud);
    this->addCloud(viewerCloud);
  }

  void PCViewer::addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
  {
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloudExG(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = _cloud->width;
    cloud->height = _cloud->height;

    cloud->points.resize(cloud->width*cloud->height);

    for (int i = 0; i < _cloud->size(); i++)
      {
        cloud->points[i].x = _cloud->points[i].x;
        cloud->points[i].y = _cloud->points[i].y;
        cloud->points[i].z = _cloud->points[i].z;
        cloud->points[i].r = _cloud->points[i].r;
        cloud->points[i].g = _cloud->points[i].g;
        cloud->points[i].b = _cloud->points[i].b;
      }

    PCViewerCloud viewerCloud(_name, cloud);
    this->addCloud(viewerCloud);
    //this->PCViewerClouds.push_back(viewerCloud);
  }
  void PCViewer::addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud)
  {
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloudExG(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    cloud->width = _cloud->width;
    cloud->height = _cloud->height;
    normals->width = _cloud->width;
    normals->height = _cloud->height;

    cloud->points.resize(cloud->width*cloud->height);
    normals->points.resize(normals->width*normals->height);

    for (int i = 0; i < _cloud->size(); i++)
      {
        cloud->points[i].x = _cloud->points[i].x;
        cloud->points[i].y = _cloud->points[i].y;
        cloud->points[i].z = _cloud->points[i].z;
        cloud->points[i].r = _cloud->points[i].r;
        cloud->points[i].g = _cloud->points[i].g;
        cloud->points[i].b = _cloud->points[i].b;
        normals->points[i].normal_x = _cloud->points[i].normal_x;
        normals->points[i].normal_y = _cloud->points[i].normal_y;
        normals->points[i].normal_z = _cloud->points[i].normal_z;
      }

    PCViewerCloud viewerCloud(_name, cloud, normals);
    this->addCloud(viewerCloud);
    //this->PCViewerClouds.push_back(viewerCloud);
  }

  int PCViewer::getNumberOfClouds()
  {
    return this->PCViewerClouds.size();
  }

  /***************************
   *                         *
   *      PCViewerCloud      *
   *                         *
   ***************************/

  PCViewerCloud::PCViewerCloud()
  {
    this->name = "";
    this->hasNormal = false;
    this->isMesh = false;
  }

  PCViewerCloud::PCViewerCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
  {
    this->name = _name;
    this->cloud = _cloud;
    this->hasNormal = false;
    this->isMesh = false;
  }
  PCViewerCloud::PCViewerCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, pcl::PointCloud<pcl::Normal>::Ptr _normals)
  {
    this->name = _name;
    this->cloud = _cloud;
    this->normals = _normals;
    this->hasNormal = true;
    this->isMesh = false;
  }
  PCViewerCloud::PCViewerCloud(std::string _name, pcl::PolygonMesh::Ptr _mesh)
  {
    this->name = _name;
    this->mesh = _mesh;
    this->hasNormal = false;
    this->isMesh = true;
  }
  PCViewerCloud::PCViewerCloud(std::string _name, pcl::PolygonMesh::Ptr _mesh, pcl::PointCloud<pcl::Normal>::Ptr _normals)
  {
    this->name = _name;
    this->mesh = _mesh;
    this->normals = _normals;
    this->hasNormal = true;
    this->isMesh = true;
  }

  /***************************
   *                         *
   *      PCViewerColor      *
   *                         *
   ***************************/

  PCViewerColor::PCViewerColor()
  {
    this->r = (uint8_t)(std::rand() % 255);
    this->g = (uint8_t)(std::rand() % 255);
    this->b = (uint8_t)(std::rand() % 255);
  }
  PCViewerColor::PCViewerColor(uint8_t _r, uint8_t _g, uint8_t _b)
  {
    this->r = _r;
    this->g = _g;
    this->b = _b;
  }
  PCViewerColor::PCViewerColor(float _r, float _g, float _b)
  {
    this->r = (uint8_t)(_r*255);
    this->g = (uint8_t)(_g*255);
    this->b = (uint8_t)(_b*255);
  }
}
