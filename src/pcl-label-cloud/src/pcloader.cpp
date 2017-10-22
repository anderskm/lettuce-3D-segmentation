#include "pcloader.h"
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>

pcLoader::pcLoader(std::string _filename)
{
  this->filename = _filename;
  this->verbose = false;
  this->isHeaderRead = false;
  this->isCloudLoaded = false;
  this->hasXY = false;
  this->hasZ = false;
  this->hasHSV = false;
  this->hasI = false;
  this->hasNormal = false;
  this->hasL = false;
  this->hasRGB = false;
  this->hasA = false;
}

pcLoader::pcLoader(std::string _filename, bool _verbose)
{
  this->filename = _filename;
  this->verbose = _verbose;
  this->isHeaderRead = false;
  this->isCloudLoaded = false;
  this->hasXY = false;
  this->hasZ = false;
  this->hasHSV = false;
  this->hasI = false;
  this->hasNormal = false;
  this->hasL = false;
  this->hasRGB = false;
  this->hasA = false;
}

int pcLoader::readHeader(std::string filename) {
  if (this->verbose) {
    pcl::console::print_info("Reading header: ");
    pcl::console::print_value("%s\n", filename.c_str());
  }
  std::string ext = this->getExtension(filename);

  pcl::PCLPointCloud2 _cloud;
  int status;

  // Select correct loader based on extension
  if (!ext.compare("ply")) {
    pcl::PLYReader reader;

    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int ply_version;
    int data_type;
    uint data_idx;

    status = reader.readHeader(filename, _cloud, origin, orientation, ply_version, data_type, data_idx);
  }
  else if (!ext.compare("pcd")) {
      pcl::PCDReader reader;
      reader.readHeader(filename, _cloud);
      //return pcl::io::loadPCDFile(filename, cloud);
  }
  else {
    if (this->verbose) {
      pcl::console::print_error("Handling of extension unknown.\n");
    }
    return -1;
  }

  if (status < 0) {
      if (this->verbose) {
        pcl::console::print_error("An error occured while reading the header...\n");
      }
      return status;
    }

  std::string fieldsList = pcl::getFieldsList(_cloud);

  if (this->verbose) {
    pcl::console::print_info("Fields: ");
    pcl::console::print_value("%s\n", fieldsList.c_str());
  }

  status = this->parseFieldsList(fieldsList);

  if (status >= 0)
    this->isHeaderRead = true;

  return status;
}
int pcLoader::readHeader() {
  // Call explicit reader
  return this->readHeader(this->filename);
}

std::string pcLoader::getExtension(std::string filename) {
  // Find extension
  std::string ext = filename.substr(filename.length()-3,3);
  if (this->verbose) {
    pcl::console::print_info("Extension: ");
    pcl::console::print_value("%s\n",ext.c_str());
  }
  return ext;
}

std::string pcLoader::suggestPointT()
{
  int status;
  std::string cloudPointT;

  if (!this->isHeaderRead)
    status = this->readHeader();
  if (status < 0)
    return cloudPointT;

  if ((this->hasXY) && (!this->hasZ)) // Assume PointXY
    cloudPointT = "PointXY";
  else if ((this->hasXY) && (this->hasZ) && (!this->hasRGB) && (!this->hasHSV) && (!this->hasI) && (!this->hasL) && (!this->hasNormal))
    cloudPointT = "PointXYZ";
  else if ((this->hasXY) && (this->hasZ) && (this->hasHSV))
    cloudPointT = "PointXYZHSV";
  else if ((this->hasXY) && (this->hasZ) && (this->hasI) && (!this->hasNormal))
    cloudPointT = "PointXYZI";
  else if ((this->hasXY) && (this->hasZ) && (this->hasI) && (this->hasNormal))
    cloudPointT = "PointXYZINormal";
  else if ((this->hasXY) && (this->hasZ) && (this->hasL))
    cloudPointT = "PointXYZL";
  else if ((this->hasXY) && (this->hasZ) && (this->hasRGB) && (!this->hasA) && (!this->hasL) && (!this->hasNormal))
    cloudPointT = "PointXYZRGB";
  else if ((this->hasXY) && (this->hasZ) && (this->hasRGB) && (this->hasA) && (!this->hasL) && (!this->hasNormal))
    cloudPointT = "PointXYZRGBA";
  else if ((this->hasXY) && (this->hasZ) && (this->hasRGB) && (!this->hasA) && (this->hasL) && (!this->hasNormal))
    cloudPointT = "PointXYZRGBL";
  else if ((this->hasXY) && (this->hasZ) && (this->hasRGB) && (!this->hasA) && (!this->hasL) && (this->hasNormal))
    cloudPointT = "PointXYZRGBNormal";
  else
    if (this->verbose) {
      pcl::console::print_highlight("Could not determined pointT type. Unknown combination of fields.");
    }

  if (this->verbose) {
    pcl::console::print_info("Suggested PointT: "); pcl::console::print_value("%s\n",cloudPointT.c_str());
  }
  return cloudPointT;
}

std::string pcLoader::getExtension() {
  return this->getExtension(this->filename);
}

int pcLoader::load(std::string filename) {
  int status;

  // Find extension
  std::string ext = filename.substr(filename.length()-3,3);
  if (this->verbose) {
    pcl::console::print_value("Extension: %s\n",ext.c_str());
  }

  if (!this->isHeaderRead)
    status = this->readHeader();
  if (status < 0)
    return status;

  // Select correct loader based on extension
  if (!ext.compare("ply")) {
    status = pcLoader::loadPLY();
  }
  else if (!ext.compare("pcd")) {
      status = pcLoader::loadPCD();
  }
  else {
    if (this->verbose) {
      pcl::console::print_highlight("Unknown extension: "); pcl::console::print_value("%s\n",ext.c_str());
    }
    status = -1;
  }
  return status;
}
int pcLoader::load() {
  // call explicit loader
  return this->load(this->filename);
}

int pcLoader::loadPLY() {
  int status;
  std::string filename = this->filename;

  std::string cloudPointT = this->suggestPointT();
  if (cloudPointT.empty()) {
    status = -1;
    return status;
  }

  if (!cloudPointT.compare("PointXY"))
    status = pcl::io::loadPLYFile(filename, this->cloudXY);
  else if (!cloudPointT.compare("PointXYZ"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZ);
  else if (!cloudPointT.compare("PointXYZHSV"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZHSV);
  else if (!cloudPointT.compare("PointXYZI"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZI);
  else if (!cloudPointT.compare("PointXYZINormal"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZINormal);
  else if (!cloudPointT.compare("PointXYZL"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZL);
  else if (!cloudPointT.compare("PointXYZRGB"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZRGB);
  else if (!cloudPointT.compare("PointXYZRGBA"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZRGBA);
  else if (!cloudPointT.compare("PointXYZRGBL"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZRGBL);
  else if (!cloudPointT.compare("PointXYZRGBNormal"))
    status = pcl::io::loadPLYFile(filename, this->cloudXYZRGBNormal);
  else {
    status = -1;
    if (this->verbose) {
      pcl::console::print_warn("Unknown PointT format.");
      pcl::console::print_value("%s\n",cloudPointT);
    }
    return status;
  }

  return status;
}

int pcLoader::loadPCD() {
  int status;
  std::string filename = this->filename;

  std::string cloudPointT = this->suggestPointT();
  if (cloudPointT.empty()) {
    status = -1;
    return status;
  }

  if (!cloudPointT.compare("PointXY"))
    status = pcl::io::loadPCDFile(filename, this->cloudXY);
  else if (!cloudPointT.compare("PointXYZ"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZ);
  else if (!cloudPointT.compare("PointXYZHSV"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZHSV);
  else if (!cloudPointT.compare("PointXYZI"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZI);
  else if (!cloudPointT.compare("PointXYZINormal"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZINormal);
  else if (!cloudPointT.compare("PointXYZL"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZL);
  else if (!cloudPointT.compare("PointXYZRGB"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZRGB);
  else if (!cloudPointT.compare("PointXYZRGBA"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZRGBA);
  else if (!cloudPointT.compare("PointXYZRGBL"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZRGBL);
  else if (!cloudPointT.compare("PointXYZRGBNormal"))
    status = pcl::io::loadPCDFile(filename, this->cloudXYZRGBNormal);
  else {
    status = -1;
    if (this->verbose) {
      pcl::console::print_warn("Unknown PointT format.");
    }
    return status;
  }

  return status;
}

int pcLoader::load(const std::string filename, pcl::PCLPointCloud2 &cloud) {

  // Find extension
  std::string ext = filename.substr(filename.length()-3,3);
  if (this->verbose) {
    pcl::console::print_value("Extension: %s\n",ext.c_str());
  }

  // Select correct loader based on extension
  if (!ext.compare("ply")) {
    return pcl::io::loadPLYFile(filename, cloud);
  }
  else if (!ext.compare("pcd")) {
      return pcl::io::loadPCDFile(filename, cloud);
  }
  else {
    if (this->verbose) {
      pcl::console::print_highlight("Unknown extension: "); pcl::console::print_value("%s\n",ext.c_str());
    }
    return -1;
  }
}

int pcLoader::parseFieldsList(std::string fieldsList) {
  std::string delimiter = " "; // delimiter between fields

  bool _hasX = false;
  bool _hasY = false;
  bool _hasZ = false;
  bool _hasR = false;
  bool _hasG = false;
  bool _hasB = false;
  bool _hasNx = false;
  bool _hasNy = false;
  bool _hasNz = false;
  bool _hasCurve = false;

  // Store local copy of field list - just in case
  std::string _fieldsList = fieldsList;

  std::size_t delimiterIdx;
  std::string field;
  bool stopLoop = false;
  // Loop through fields until there are no more characters in the string, or the loop is terminated
  while ((_fieldsList.length() > 0) && (!stopLoop)) {
    // Find first delimiter and extract field; pass remaining list to next loop-pass
    delimiterIdx = _fieldsList.find_first_of(delimiter);
    if (delimiterIdx == std::string::npos) {
      field = _fieldsList;
      stopLoop = true;
    }
    else {
      field = _fieldsList.substr(0,delimiterIdx);
      _fieldsList = _fieldsList.substr(delimiterIdx+1,_fieldsList.length()-delimiterIdx-1);
    }

    // Compare field to known names; set appropriate flags
    // NOTE: string.compare("s") returns 0 if string is equal to "s"; hence ! before all string.compare()
    if (!field.compare("x") || !field.compare("X")) // Check for X
      _hasX = true;
    else if (!field.compare("y") || !field.compare("Y"))
      _hasY = true;
    else if (!field.compare("z") || !field.compare("Z"))
      _hasZ = true;
    else if (!field.compare("rgb") || !field.compare("RGB")) {
      _hasR = true;
      _hasG = true;
      _hasB = true;
    }
    else if (!field.compare("r") || !field.compare("R"))
      _hasR = true;
    else if (!field.compare("g") || !field.compare("G"))
      _hasG = true;
    else if (!field.compare("b") || !field.compare("B"))
      _hasB = true;
    else if (!field.compare("nx") || !field.compare("normal_x"))
      _hasNx = true;
    else if (!field.compare("ny") || !field.compare("normal_y"))
      _hasNy = true;
    else if (!field.compare("nz") || !field.compare("normal_z"))
      _hasNz = true;
    else if (!field.compare("curvature"))
      _hasCurve = true;
    else {
      if(this->verbose) {
        pcl::console::print_highlight("Unknown field: ");
        pcl::console::print_value("%s\n",field.c_str());
      }
    }

  }

  // Store variables, if conditions are met
  if ((_hasX) && (_hasY)) {
    this->hasXY = true;
  }
  if (_hasZ) {
    this->hasZ = true;
  }
  if ((_hasR) && (_hasG) && (_hasB)) {
    this->hasRGB = true;
  }
  if ((_hasNx) && (_hasNx) && (_hasNx)) {
    this->hasNormal = true;
  }
  if (_hasCurve) {
      this->hasCurvature = true;
  }

  if (this->verbose) {
    pcl::console::print_info("Has XY       : "); pcl::console::print_value("%d\n", this->hasXY);
    pcl::console::print_info("Has Z        : "); pcl::console::print_value("%d\n", this->hasZ);
    pcl::console::print_info("Has RGB      : "); pcl::console::print_value("%d\n", this->hasRGB);
    pcl::console::print_info("Has Normal   : "); pcl::console::print_value("%d\n", this->hasNormal);
    pcl::console::print_info("Has Curvature: "); pcl::console::print_value("%d\n", this->hasCurvature);
  }

  return 0;
}

int pcLoader::getCloud(pcl::PointCloud<pcl::PointXY> &cloud) {
  cloud = this->cloudXY;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  cloud = this->cloudXYZ;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZHSV> &cloud) {
  cloud = this->cloudXYZHSV;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZI> &cloud) {
  cloud = this->cloudXYZI;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloud) {
  cloud = this->cloudXYZINormal;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZL> &cloud) {
  cloud = this->cloudXYZL;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  cloud = this->cloudXYZRGB;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZRGBA> &cloud) {
  cloud = this->cloudXYZRGBA;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZRGBL> &cloud) {
  cloud = this->cloudXYZRGBL;
}
int pcLoader::getCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud) {
  cloud = this->cloudXYZRGBNormal;
}
