#ifndef PCLOADER_H
#define PCLOADER_H

#include <string>
#include <pcl/io/ply_io.h>

class pcLoader
{
public:
  pcl::PCLPointCloud2 cloud;
  pcLoader(std::string _filename);
  pcLoader(std::string _filename, bool _verbose);
  std::string getExtension();
  int readHeader();

  int load(const std::string filename, pcl::PCLPointCloud2 &cloud);
  int load(const std::string filename);
  int load();

  std::string suggestPointT();

  int getCloud(pcl::PointCloud<pcl::PointXY> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZHSV> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZI> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZL> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZRGBL> &cloud);
  int getCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

private:
  // Flags
  bool verbose;

  bool isHeaderRead;
  bool isCloudLoaded;

  bool hasXY;
  bool hasZ; // Assumes, hasXY == true
  bool hasI; // Assumes, hasZ == true
  bool hasL; // Assumes, hasZ == true
  bool hasHSV; // Assumes, hasZ == true
  bool hasRGB; // Assumes, hasZ == true
  bool hasA; // Assumes, hasRGB == true
  bool hasNormal; // Assumes, hasI == true || hasRGB == true
  bool hasCurvature; // Assumes, hasNormal == true

  // Strings
  std::string filename;
  std::string ext;

  // Potential point clouds
  pcl::PointCloud<pcl::PointXY> cloudXY;
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
  pcl::PointCloud<pcl::PointXYZHSV> cloudXYZHSV;
  pcl::PointCloud<pcl::PointXYZI> cloudXYZI;
  pcl::PointCloud<pcl::PointXYZINormal> cloudXYZINormal;
  pcl::PointCloud<pcl::PointXYZL> cloudXYZL;
  pcl::PointCloud<pcl::PointXYZRGB> cloudXYZRGB;
  pcl::PointCloud<pcl::PointXYZRGBA> cloudXYZRGBA;
  pcl::PointCloud<pcl::PointXYZRGBL> cloudXYZRGBL;
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloudXYZRGBNormal;

  std::string getExtension(std::string filename);
  int parseFieldsList(std::string fieldsList);
  int readHeader(std::string filename);
  int loadPLY();
  int loadPCD();

};

#endif // PCLOADER_H
