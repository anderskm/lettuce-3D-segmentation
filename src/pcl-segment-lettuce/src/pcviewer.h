#ifndef PCVIEWER_H
#define PCVIEWER_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

namespace PCViewer
{
  class PCViewerCloud
  {
  public:
    bool isMesh;
    bool hasNormal;
    std::string name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PolygonMesh::Ptr mesh;

    PCViewerCloud();
    PCViewerCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud); // sets hasNormal = false, sets isMesh = false
    PCViewerCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, pcl::PointCloud<pcl::Normal>::Ptr _normals); // sets hasNormal = true, sets isMesh = false
    PCViewerCloud(std::string _name, pcl::PolygonMesh::Ptr _mesh); // sets hasNormal = false, sets isMesh = true;
    PCViewerCloud(std::string _name, pcl::PolygonMesh::Ptr _mesh, pcl::PointCloud<pcl::Normal>::Ptr _normals); // sets hasNormal = true, sets isMesh = true
    //setNormals(pcl::PointCloud<pcl::Normal> _normals); // Sets hasNormal = true;
  };

  class PCViewerColor
  {
    public:
      uint8_t r;
      uint8_t g;
      uint8_t b;

      PCViewerColor();
      PCViewerColor(uint8_t _r, uint8_t _g, uint8_t _b);
      PCViewerColor(float _r, float _g, float _b);
  };

  class PCViewer
  {
  public:

    bool verbose;

    PCViewer();
    PCViewer(bool _verbose);

    void addCloud(PCViewerCloud viewerCloud);
    void addCloud(std::string _name, pcl::PointCloud<pcl::PointXY>::Ptr _cloud);
    void addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud);
    void addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud, float minIntensity, float maxIntensity);
    void addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZL>::Ptr _cloud);
    void addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
    void addCloud(std::string _name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud);

    void singleViewer();
    void singleViewer(int cloudIdx);
    void spin();
    //void singleViewer_keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    int getNumberOfClouds();
    //dualViewer();
    //dualViewer(int cloud1Idx, int cloud2Idx);

  private:
    std::vector<PCViewerCloud> PCViewerClouds;
    bool showNormals;
    void updateViewer();
    pcl::PointXYZRGB pointPick;

    /* Key presses
     * Page Up/Down - skip through added clouds
     * Arrow Up/Down - skip through added clouds
     * F1/F2 - select left or right cloud in viewer
     * Arrow Left/Right - skip through viewports
     * C - select coloring (RGB/Intensity, normals, height)
     * B - Toggle through background colors (black, red, green, blue, yellow, cyan, magenta, white)
     * N - Toggle normals. Show/hide normal vectors
     * H - Brig up help text
     */
  };

}
#endif // PCVIEWER_H
