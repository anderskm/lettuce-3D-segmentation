#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/common/pca.h>

#include <boost/program_options.hpp>

#include "pcloader.h"
#include "pcviewer.h"

#include <string>
#include <algorithm>
#include <fstream>
#include <time.h>

namespace po = boost::program_options;


uint32_t pointIdx2label(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, std::vector<int> pointIdx, std::vector<float> pointSquaredDistance, float distanceThreshold)
{
  // Find all labels
  std::vector<int> labels;
  for (int i = 0; i < pointIdx.size(); i++)
    {
       labels.push_back(cloud->points[pointIdx[i]].label);
    }

  // Find unique labels
  std::sort (labels.begin(), labels.end()); // Sort labels first, otherwise unique might return multiple instances of same label
  std::vector<int>::iterator it;
  it = std::unique(labels.begin(), labels.end());
  labels.resize( std::distance(labels.begin(),it) );

  // Calculate label scores; higher score is better
  std::vector<float> labelScores;
  for (int l = 0; l < labels.size(); l++)
    {
      int label = labels[l];
      float score = 0;
      for (int p = 0; p < pointIdx.size(); p++)
        {
          if (cloud->points[pointIdx[p]].label == label)
            {
              if (distanceThreshold*distanceThreshold > pointSquaredDistance[p])
                {
                  score += 1/sqrt(pointSquaredDistance[p]); // Inverse distance
                }
            }
        }
      labelScores.push_back(score);
    }


  // Find max score
  int maxLabelScoreIdx = 0;
  float maxLabelScore = labelScores[maxLabelScoreIdx];
  for (int i = 0; i < labelScores.size(); i++)
    {
      if (labelScores[maxLabelScoreIdx] < labelScores[i])
        {
          maxLabelScore = labelScores[i];
          maxLabelScoreIdx = i;
        }

    }

  if (maxLabelScore > 0)
    return labels[maxLabelScoreIdx];
  else
    return -1;
}

void reconstructFromClusters(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloudIn, std::vector <pcl::PointIndices> _clustersIn, std::vector <int> clustersIndices, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloudOut)
{
  pcl::PointXYZRGBNormal clusterPoint;
  int clusterPointIdx = 0;
  int clusterIdx = 0;
  for (int c = 0; c < clustersIndices.size(); c++)
    {
      clusterIdx = clustersIndices[c];
      for (int i = 0; i < _clustersIn[clusterIdx].indices.size(); i++)
        {
          clusterPointIdx = _clustersIn[clusterIdx].indices[i];
          clusterPoint = _cloudIn->points[clusterPointIdx];
          _cloudOut->points.push_back(clusterPoint);
        }
    }
  _cloudOut->width = _cloudOut->points.size();
  _cloudOut->height = 1;
}

void extractNeighborhoodCircle(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud, pcl::PointIndices::Ptr processingIndices, float centerX, float centerY, float neighborhoodRadius, pcl::PointIndices::Ptr &indicesWithinNeighborhood, pcl::PointIndices::Ptr &indicesOutsideNeighborhood)
{

  float R2 = 0;
  float Y = 0;
  float X = 0;
  int idx;
  float neighbodhoodRadiusSquared = (neighborhoodRadius*neighborhoodRadius);
  for (int i = 0; i < processingIndices->indices.size(); i++)
    {
      idx = processingIndices->indices[i];
      X = cloud->points[idx].x - centerX;
      Y = cloud->points[idx].y - centerY;
      R2 = X*X + Y*Y;
      if (R2 < neighbodhoodRadiusSquared)
        {
          indicesWithinNeighborhood->indices.push_back(idx);
        }
      else
        {
          indicesOutsideNeighborhood->indices.push_back(idx);
        }
    }
}

float estimateGroundPlane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointIndices::Ptr processingIndices, float distanceThreshold, pcl::ModelCoefficients::Ptr &coefficientsOut, float refX, float refY)
{
  // Fit plane to ground
  pcl::PointIndices::Ptr groundPlane_inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Required
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distanceThreshold);
  seg.setIndices(processingIndices);

  seg.setInputCloud (cloudIn);
  seg.segment (*groundPlane_inliers, *coefficientsOut);

  float z0;

  if (groundPlane_inliers->indices.size () == 0)
  {
    pcl::console::print_error("Could not estimate a planar model for the given dataset.");
    return 0;
  }
  else
  {
    z0 = (-coefficientsOut->values[3] - coefficientsOut->values[0]*refX - coefficientsOut->values[1]*refY) / (coefficientsOut->values[2]);
    pcl::console::print_info("Plane fitted!\n");

    pcl::console::print_info("   Number of inliners: ");
    pcl::console::print_value("%d\n",groundPlane_inliers->indices.size ());

    pcl::console::print_info("   Model coefficients (ax + by + cz + d = 0):\n");
    pcl::console::print_value("      a = %f\n",coefficientsOut->values[0]);
    pcl::console::print_value("      b = %f\n",coefficientsOut->values[1]);
    pcl::console::print_value("      c = %f\n",coefficientsOut->values[2]);
    pcl::console::print_value("      d = %f\n",coefficientsOut->values[3]);

    pcl::console::print_info("   Given x0, y0, then z0 = -(d + a*x0 + b*y0)/c:\n");
    pcl::console::print_value("      z0 = %f\n",z0);
  }
  return z0;
}

void copyPointIndices(pcl::PointIndices::Ptr &indices_in, pcl::PointIndices::Ptr &indices_out)
{
  indices_out->indices.clear();
  for (int i = 0; i < indices_in->indices.size(); i++)
    {
      indices_out->indices.push_back(indices_in->indices[i]);
    }
}

void pointCloud2pointIndices(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointIndices::Ptr &indices)
{
  indices->indices.clear();
  indices->indices.reserve(cloudIn->points.size());
  for (int i = 0; i < cloudIn->points.size(); i++)
    {
      indices->indices.push_back(i);
    }
}

void removeDarkPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointIndices::Ptr processingIndices,  float grayTheshold, pcl::PointIndices::Ptr &lightPointIndices, pcl::PointIndices::Ptr &darkPointIndices)
{
  // Remove dark points, e.g shadow edges around the leaves
  for (int i = 0; i < processingIndices->indices.size(); i++)
    {
      int idx = processingIndices->indices[i];
      float gray = (0.2989 * (float)cloudIn->points[idx].r + 0.5870 * (float)cloudIn->points[idx].g + 0.1140 * (float)cloudIn->points[idx].b)/255.0;
      if (gray >= grayTheshold)
        {
          lightPointIndices->indices.push_back(idx);
        }
      else
        {
          darkPointIndices->indices.push_back(idx);
        }
    }
}

void ExGRfilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointIndices::Ptr processingIndices, float ExGRThreshold, pcl::PointIndices::Ptr &pointIndicesAboveThreshold, pcl::PointIndices::Ptr &pointIndicesBelowThreshold)
{
  for (int i = 0; i < processingIndices->indices.size(); i++)
    {
      int idx = processingIndices->indices[i];
      float ExGR = (3.0*(float)cloudIn->points[idx].g - 2.4*(float)cloudIn->points[idx].r - 1.0*(float)cloudIn->points[idx].b)/((float)cloudIn->points[idx].g + (float)cloudIn->points[idx].r + (float)cloudIn->points[idx].b);
      if (ExGR > ExGRThreshold)
        {
          pointIndicesAboveThreshold->indices.push_back(idx);
        }
      else
        {
          pointIndicesBelowThreshold->indices.push_back(idx);
        }
    }
}

void heightFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIn, pcl::PointIndices::Ptr processingIndices, pcl::PointXYZ referencePoint, float heightScale, float heightOffset, pcl::PointIndices::Ptr &pointIndicesAboveThreshold, pcl::PointIndices::Ptr &pointIndicesBelowThreshold)
{
  float radius;
  float x;
  float y;
  float z;
  int idx;
  for (int i = 0; i < processingIndices->indices.size(); i++)
    {
      idx = processingIndices->indices[i];
      x = (cloudIn->points[idx].x - referencePoint.x);
      y = (cloudIn->points[idx].y - referencePoint.y);
      z = (cloudIn->points[idx].z - referencePoint.z);
      z = z * 100; // m to cm
      radius = sqrt(x*x + y*y);
      radius = radius*100; // m to cm
      //float heightThreshold = std::max(heightScale*log10(radius-heightOffset), 0.0);
      float heightThreshold = std::max(heightScale*log10(radius),0.0);
      if (z > heightThreshold)
        {
          pointIndicesAboveThreshold->indices.push_back(idx);
        }
      else
        {
          pointIndicesBelowThreshold->indices.push_back(idx);
        }
    }
}

int main(int argc, char *argv[])
{
  std::string filename;
  std::string centerCSV;
  std::string filenameOut;
  std::string filenameZ0;
  bool showClouds;

  // Bed variables
  float betweenRowSpacing;
  float inRowSpacing;
  std::string filenameNeighborPlants;
  float darkPointsThreshold_bed;
  float ExGRThreshold_bed;
  float soilDenoiseStdDev_bed;
  float planeDistanceThreshold_bed;

  // Neighborhood variables
  float neighborhoodRadius;
  float darkPointsThreshold_neighborhood;
  float ExGRThreshold_neighborhood;
  int normalsNumNeighbors;

  // Plant variables
  float plantHeightThreshold;
  float plantHeightOffset;

  // Leaf segmentation variables
  int leafClusteringNumNeighbors;
  float leafClusteringSmoothessThreshold;
  float leafClusteringCurvatureThreshold;
  float leafAssignmentInnerDist;
  float leafAssignmentOuterDist;
  int leafClusteringMinSize;
  int leafClusteringKNN;

  // Skip processing steps
  bool skipPreprocess;
  bool skipHeightFilter;
  bool skipSegLeaves;
  bool skipSubLeaves;

  // Performance variables
  std::string filenameLabels;
  std::string filenameOutConfusionMatrix;
  std::string filenameOutLabels;
  std::string filenameOutLoiLabels;
  float performanceNeighborhoodRadius;

    // Declare the supported options.
    po::options_description genericOptions("Generic options");
    genericOptions.add_options()
        ("help", "produce help message")
        ("input-file", po::value<std::string>(&filename)->required(), "Path to point cloud with lettuce bed")
        ("plant-csv", po::value<std::string>(&centerCSV)->required(), "Path to csv-file with x,y,z-coordinates of lettuce of interest")
        ("output-file", po::value<std::string>(&filenameOut)->default_value("NotSet"), "Path to output file")
        ("output-z0-csv", po::value<std::string>(&filenameZ0)->default_value("NotSet"), "Path to output csv-file where the height of the stem point will be saved.")
        ("show-clouds", po::value<bool>(&showClouds)->default_value(false), "True/false if clouds should be shown before closing program. If both output-file and labels are not set, show-clouds are automatically set to true.")
    ;
    po::options_description bedOptions("Bed options");
    bedOptions.add_options()
        ("plant-spacing-between-rows", po::value<float>(&betweenRowSpacing)->default_value(0.6), "The sowing distance between two rows of lettuce in the same bed.")
        ("plant-spacing-in-row", po::value<float>(&inRowSpacing)->default_value(0.3), "The sowing distance between two neighboring lettuce in the same row.")
        ("bed-neighbor-locations", po::value<std::string>(&filenameNeighborPlants)->default_value("NotSet"), "Path to point cloud with location of neighboring plants. Used for ground plane estimation. If not specified, the positions with be estimated using PCA.")
        ("bed-dark-th", po::value<float>(&darkPointsThreshold_bed)->default_value(0.1), "Threshold value for excluding dark points [0;1]. Any gray (0.2989*R + 0.5870*G + 0.114*B) value lower than the threshold is removed.")
        ("bed-exgr-th", po::value<float>(&ExGRThreshold_bed)->default_value(0.0), "Threshold value for separating soil and green plant material using ExGR.")
        ("bed-soil-denoise-th", po::value<float>(&soilDenoiseStdDev_bed)->default_value(2.0), "Threshold for removing .")
        ("bed-plant-dist-th", po::value<float>(&planeDistanceThreshold_bed)->default_value(0.02), "Distance threshold ")
    ;
    po::options_description neighborhoodOptions("Neighborhood options");
    neighborhoodOptions.add_options()
        ("nb-radius", po::value<float>(&neighborhoodRadius)->default_value(0.2), "Radius of circular neighborhood around around plant center, which is extracted for segmentation.")
        ("nb-dark-th", po::value<float>(&darkPointsThreshold_neighborhood)->default_value(0.1), "Threshold value for excluding dark points [0;1]. Any gray (0.2989*R + 0.5870*G + 0.114*B) value lower than the threshold is removed.")
        ("nb-exgr-th", po::value<float>(&ExGRThreshold_neighborhood)->default_value(0.0), "Threshold value for separating soil and green plant material using ExGR.")
        ("nb-normals-k", po::value<int>(&normalsNumNeighbors)->default_value(50), "Number of neighbors used for normal estimation.")
    ;
    po::options_description plantOptions("Plant options");
    plantOptions.add_options()
        ("plant-height-th", po::value<float>(&plantHeightThreshold)->default_value(9.0), "Plant height threshold in cm.")
        ("plant-height-offset", po::value<float>(&plantHeightOffset)->default_value(0.0), "Plant height distance off set in cm.")
    ;
    po::options_description leafOptions("Leaf segmentation options");
    leafOptions.add_options()
        ("leaf-num-neighbors", po::value<int>(&leafClusteringNumNeighbors)->default_value(30), "Number of neighbors to look at during leaf clustering")
        ("leaf-smooth-th", po::value<float>(&leafClusteringSmoothessThreshold)->default_value(4.0), "Max difference in angle between points in leaf cluster")
        ("leaf-curvature-th", po::value<float>(&leafClusteringCurvatureThreshold)->default_value(1.0), "Max curvature of leafs")
        ("leaf-inner-dist-th", po::value<float>(&leafAssignmentInnerDist)->default_value(0.15), "Inner distance of centroid for upper leaf cluster")
        ("leaf-outer-dist-th", po::value<float>(&leafAssignmentOuterDist)->default_value(0.20), "Outer distance of centroid for upper leaf cluster")
        ("leaf-min-size", po::value<int>(&leafClusteringMinSize)->default_value(500), "Minimum number of points in leaf clusters")
        ("leaf-inner-clustering-K", po::value<int>(&leafClusteringKNN)->default_value(30), "Number of neighbors to look at when assigning small clusters (< leaf-min-size) to larger leaf clusters (> leaf-min-size).")
    ;
    po::options_description skipOptions("Skip options");
    skipOptions.add_options()
        ("skip-pre-process", po::value<bool>(&skipPreprocess)->default_value(false), "True/false if pre-processing and all subsequent steps should be skipped.")
        ("skip-height-filter", po::value<bool>(&skipHeightFilter)->default_value(false), "True/false if height filter and all subsequent steps should be skipped.")
        ("skip-seg-leaves", po::value<bool>(&skipSegLeaves)->default_value(false), "True/false if leaf segmentation and all subsequent steps should be skipped.")
        ("skip-sub-leaves", po::value<bool>(&skipSubLeaves)->default_value(false), "True/false if sub leaf inclusion and all subsequent steps should be skipped.")
    ;
    po::options_description performanceOptions("Performance options");
    performanceOptions.add_options()
        ("labels", po::value<std::string>(&filenameLabels)->default_value("NotSet"), "Path to labeled point cloud")
        ("output-conf-mat-csv", po::value<std::string>(&filenameOutConfusionMatrix)->default_value("NotSet"), "Path to output csv-file where the confusion matrix will be saved.")
        ("output-labels-gt", po::value<std::string>(&filenameOutLabels)->default_value("NotSet"), "Path to output pcd-file where the ground truth labels with in perf-radius will be saved.")
        ("output-labels-loi", po::value<std::string>(&filenameOutLoiLabels)->default_value("NotSet"), "Path to output pcd-file where the LOI labels with in perf-radius will be saved.")
        ("perf-radius", po::value<float>(&performanceNeighborhoodRadius)->default_value(0.3), "Radius of circular neighborhood extracted for calculating performance.")
    ;

    po::options_description cmdline_options;
    cmdline_options.add(genericOptions).add(bedOptions).add(neighborhoodOptions).add(plantOptions).add(leafOptions).add(skipOptions).add(performanceOptions);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(cmdline_options).run(), vm);

    if ((vm.count("help")) || (argc < 2)) {
        cout << "Usage:" << std::endl;
        cout << "\n";
        cout << genericOptions << "\n";
        cout << bedOptions << "\n";
        cout << neighborhoodOptions << "\n";
        cout << plantOptions << "\n";
        cout << leafOptions << "\n";
        cout << skipOptions << "\n";
        cout << performanceOptions << "\n";
        return 1;
    }

    po::notify(vm);

    if (skipPreprocess)
      skipHeightFilter = true;
    if (skipHeightFilter)
      skipSegLeaves = true;
    if (skipSegLeaves)
      skipSubLeaves = true;


  std::clock_t processTimeTotal = std::clock();


  bool saveOutput = false;
  bool compareToLabels = false;
  bool saveConfusionMatrix = false;
  bool saveLabels = false;
  bool saveLoiLabels = false;
  if (filenameOut.compare("NotSet")) // Returns 0 if equal to "NotSet"
    {
      saveOutput = true;
    }
  if (filenameLabels.compare("NotSet")) // Returns 0 if equal to "NotSet"
    {
      compareToLabels = true;
    }
  if (filenameOutConfusionMatrix.compare("NotSet")) // Returns 0 if equal to "NotSet"
    {
      saveConfusionMatrix = true;
    }
  if (filenameOutLabels.compare("NotSet")) // Returns 0 if equal to "NotSet"
    {
      saveLabels = true;
    }
  if (filenameOutLoiLabels.compare("NotSet")) // Returns 0 if equal to "NotSet"
    {
      saveLoiLabels = true;
    }

  if ((!filenameLabels.compare("NotSet")) && (!filenameOut.compare("NotSet")))
    {
      showClouds = true;
    }

  pcl::console::print_info("Input file: ");
  pcl::console::print_value("%s\n", filename.c_str());
  pcl::console::print_info("Center CSV file: ");
  pcl::console::print_value("%s\n", centerCSV.c_str());

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

  pcl::console::print_info("Output file: ");
  if (saveOutput) {
    pcl::console::print_value("%s\n", filenameOut.c_str());
  }
  else {
      pcl::console::print_value("%s\n","No output file specified!");
  }

  pcl::console::print_info("Show clouds: "); pcl::console::print_value("%d\n", showClouds);

  std::clock_t processTime;

  processTime = std::clock();
  // Load point cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudFull (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcLoader loader(filename, false);
  loader.load();
  loader.getCloud(*cloudFull);
  processTime = std::clock() - processTime;
  pcl::console::print_value("Cloud loaded in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  // Load labels
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloudFullGroundTruthLabels (new pcl::PointCloud<pcl::PointXYZL>);
  if (compareToLabels)
    {
      processTime = std::clock();
      pcLoader labelLoader(filenameLabels, false);
      labelLoader.load();
      labelLoader.getCloud(*cloudFullGroundTruthLabels);
      processTime = std::clock() - processTime;
      pcl::console::print_value("Labels loaded in %f s\n",((float)processTime)/CLOCKS_PER_SEC);
    }

  /*
   * FULL CLOUD BEGIN
   *
   * */

  // Create debug viewer and add original cloud if output is not saved
  PCViewer::PCViewer pcViewer(true);
  if (showClouds)
    {
      pcViewer.addCloud("Full cloud",cloudFull);
      if (compareToLabels)
        {
          pcViewer.addCloud("Ground truth", cloudFullGroundTruthLabels);
        }
    }

  processTime = std::clock();
  pcl::PointIndices::Ptr allPointsIndices (new pcl::PointIndices);
  pointCloud2pointIndices(cloudFull, allPointsIndices);

  pcl::PointIndices::Ptr lightPointsIndices (new pcl::PointIndices);
  pcl::PointIndices::Ptr darkPointsIndices (new pcl::PointIndices);
  removeDarkPoints(cloudFull, allPointsIndices, darkPointsThreshold_bed, lightPointsIndices, darkPointsIndices);
  processTime = std::clock() - processTime;
  pcl::console::print_value("Shadows removed (indices) in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  if (showClouds)
    {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudFull_NoDark(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::copyPointCloud(*cloudFull, lightPointsIndices->indices, *cloudFull_NoDark);
      pcViewer.addCloud("Full; No dark",cloudFull_NoDark);
    }

  // Separate soil and green plant material by using ExGR
  processTime = std::clock();
  pcl::PointIndices::Ptr pointsIndicesAboveExGR (new pcl::PointIndices);
  pcl::PointIndices::Ptr pointsIndicesBelowExGR (new pcl::PointIndices);
  ExGRfilter(cloudFull, lightPointsIndices, ExGRThreshold_bed, pointsIndicesAboveExGR, pointsIndicesBelowExGR);
  processTime = std::clock() - processTime;
  pcl::console::print_value("ExGR calculated (indices) in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantNeighborCenter (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::console::print_highlight("Neighbor plant positions: ");
  if (filenameNeighborPlants.compare("NotSet")) // Returns 0 if equal to "NotSet"
    {
      pcl::console::print_value("File\n");
      pcl::io::loadPCDFile(filenameNeighborPlants, *cloudPlantNeighborCenter);
      if (showClouds)
        {
          pcViewer.addCloud("Ground neighborhood points, file",cloudPlantNeighborCenter);
        }
    }
  else
    {
      pcl::console::print_value("PCA\n");
      // PCA
      processTime = std::clock();
      pcl::PCA<pcl::PointXYZRGBNormal> pcaPlant;
      pcaPlant.setInputCloud(cloudFull);
      pcaPlant.setIndices(pointsIndicesAboveExGR);
      Eigen::Vector3f eigenValues = pcaPlant.getEigenValues();
      pcl::console::print_info("Eigen values:\n");
      pcl::console::print_value("e1: %f\n", eigenValues[0]);
      pcl::console::print_value("e2: %f\n", eigenValues[1]);
      pcl::console::print_value("e3: %f\n", eigenValues[2]);
      Eigen::Matrix3f eigenVectors = pcaPlant.getEigenVectors();
      pcl::console::print_info("     \t"); pcl::console::print_value("%f\t",eigenVectors(0,0)); pcl::console::print_info("     \t"); pcl::console::print_value("%f\t",eigenVectors(0,1)); pcl::console::print_info("     \t"); pcl::console::print_value("%f\n",eigenVectors(0,2));
      pcl::console::print_info("v1 = \t"); pcl::console::print_value("%f\t",eigenVectors(1,0)); pcl::console::print_info("v2 = \t"); pcl::console::print_value("%f\t",eigenVectors(1,1)); pcl::console::print_info("v3 = \t"); pcl::console::print_value("%f\n",eigenVectors(1,2));
      pcl::console::print_info("     \t"); pcl::console::print_value("%f\t",eigenVectors(2,0)); pcl::console::print_info("     \t"); pcl::console::print_value("%f\t",eigenVectors(2,1)); pcl::console::print_info("     \t"); pcl::console::print_value("%f\n",eigenVectors(2,2));

      int zDim = 0;
      bool zDimPos = true;
      if (fabs(eigenVectors(2,1)) > fabs(eigenVectors(2,2)))
        {
          zDim = 1;
          if (eigenVectors(2,1) < 0)
            {
              zDimPos = false;
            }
        }
      else
        {
          zDim = 2;
          if (eigenVectors(2,2) < 0)
            {
              zDimPos = false;
            }
        }
      pcl::console::print_value("Z-dim in PCA space: %d\n",zDim);
      pcl::console::print_value("Z-dim in PCA space positve direction?: %d\n",zDimPos);

      processTime = std::clock() - processTime;
      pcl::console::print_value("PCA calculated in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

      if (showClouds)
        {
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudFull_PlantPCA (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudFull_Plant (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          pcl::copyPointCloud(*cloudFull,pointsIndicesAboveExGR->indices,*cloudFull_Plant);
          pcaPlant.project(*cloudFull_Plant, *cloudFull_PlantPCA);
          pcViewer.addCloud("Full PCA Plant",cloudFull_PlantPCA);
        }

      processTime = std::clock();
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantCenterPoint (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::PointXYZRGBNormal pointPlantCenter;
      pointPlantCenter.x = centerX;
      pointPlantCenter.y = centerY;
      pointPlantCenter.z = centerZ;
      cloudPlantCenterPoint->points.push_back(pointPlantCenter);
      cloudPlantCenterPoint->width = cloudPlantCenterPoint->points.size();
      cloudPlantCenterPoint->height = 1;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantCenterPointPCA (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcaPlant.project(*cloudPlantCenterPoint,*cloudPlantCenterPointPCA);
      // Add neighbor points to cloud
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantNeighborCenterPointPCA (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::PointXYZRGBNormal pointPlantCenterPCA;
      pointPlantCenterPCA = cloudPlantCenterPointPCA->points[0];
      pointPlantCenterPCA.x  = pointPlantCenterPCA.x - inRowSpacing;
      cloudPlantNeighborCenterPointPCA->points.push_back(pointPlantCenterPCA);
      pointPlantCenterPCA.x  = pointPlantCenterPCA.x + 2*inRowSpacing;
      cloudPlantNeighborCenterPointPCA->points.push_back(pointPlantCenterPCA);
      if (zDim == 1)
        {
          if (pointPlantCenterPCA.z < 0)
            {
              pointPlantCenterPCA.z = pointPlantCenterPCA.z + betweenRowSpacing;
            }
          else
            {
              pointPlantCenterPCA.z = pointPlantCenterPCA.z - betweenRowSpacing;
            }
        }
      if (zDim == 2)
        {
          if (pointPlantCenterPCA.y < 0)
            {
              pointPlantCenterPCA.y = pointPlantCenterPCA.y + betweenRowSpacing;
            }
          else
            {
              pointPlantCenterPCA.y = pointPlantCenterPCA.y - betweenRowSpacing;
            }
        }
      cloudPlantNeighborCenterPointPCA->points.push_back(pointPlantCenterPCA);
      pointPlantCenterPCA.x  = pointPlantCenterPCA.x - 2*inRowSpacing;
      cloudPlantNeighborCenterPointPCA->points.push_back(pointPlantCenterPCA);
      cloudPlantNeighborCenterPointPCA->width = cloudPlantNeighborCenterPointPCA->points.size();
      cloudPlantNeighborCenterPointPCA->height = 1;

      // Project back into normal space
      pcaPlant.reconstruct(*cloudPlantNeighborCenterPointPCA,*cloudPlantNeighborCenter);

      if (showClouds)
        {
          pcViewer.addCloud("Ground neighborhood points, PCA",cloudPlantNeighborCenterPointPCA);
          pcViewer.addCloud("Ground neighborhood points",cloudPlantNeighborCenter);
        }

      processTime = std::clock() - processTime;
      pcl::console::print_value("Center and neighbor points projected back and forth in %f s\n",((float)processTime)/CLOCKS_PER_SEC);
  }


  float inRowGroundWidth = 0.5; // Relative. 1 = full width
  float betweenRowGroundWidth = 1.0; // Relative. 1 = full width
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantGroundPlaneNeighborhoodPoints (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointXYZRGBNormal pointInPlane;
  // Same row
  pointInPlane.x = cloudPlantNeighborCenter->points[0].x + (1-inRowGroundWidth)/2.0*(cloudPlantNeighborCenter->points[1].x -cloudPlantNeighborCenter->points[0].x);
  pointInPlane.y = cloudPlantNeighborCenter->points[0].y + (1-inRowGroundWidth)/2.0*(cloudPlantNeighborCenter->points[1].y -cloudPlantNeighborCenter->points[0].y);
  pointInPlane.z = centerZ;
  cloudPlantGroundPlaneNeighborhoodPoints->points.push_back(pointInPlane);
  pointInPlane.x = cloudPlantNeighborCenter->points[0].x + ((1-inRowGroundWidth)/2.0 + inRowGroundWidth)*(cloudPlantNeighborCenter->points[1].x - cloudPlantNeighborCenter->points[0].x);
  pointInPlane.y = cloudPlantNeighborCenter->points[0].y + ((1-inRowGroundWidth)/2.0 + inRowGroundWidth)*(cloudPlantNeighborCenter->points[1].y - cloudPlantNeighborCenter->points[0].y);
  cloudPlantGroundPlaneNeighborhoodPoints->points.push_back(pointInPlane);
  // Opposite row
  pointInPlane.x = cloudPlantNeighborCenter->points[3].x + (1-inRowGroundWidth)/2.0*(cloudPlantNeighborCenter->points[2].x -cloudPlantNeighborCenter->points[3].x);
  pointInPlane.y = cloudPlantNeighborCenter->points[3].y + (1-inRowGroundWidth)/2.0*(cloudPlantNeighborCenter->points[2].y -cloudPlantNeighborCenter->points[3].y);
  cloudPlantGroundPlaneNeighborhoodPoints->points.push_back(pointInPlane);
  pointInPlane.x = cloudPlantNeighborCenter->points[3].x + ((1-inRowGroundWidth)/2.0 + inRowGroundWidth)*(cloudPlantNeighborCenter->points[2].x - cloudPlantNeighborCenter->points[3].x);
  pointInPlane.y = cloudPlantNeighborCenter->points[3].y + ((1-inRowGroundWidth)/2.0 + inRowGroundWidth)*(cloudPlantNeighborCenter->points[2].y - cloudPlantNeighborCenter->points[3].y);
  cloudPlantGroundPlaneNeighborhoodPoints->points.push_back(pointInPlane);
  cloudPlantGroundPlaneNeighborhoodPoints->width = cloudPlantGroundPlaneNeighborhoodPoints->points.size();
  cloudPlantGroundPlaneNeighborhoodPoints->height = 1;

  if (showClouds)
    {
      pcViewer.addCloud("Ground plane neighborhood points, new",cloudPlantGroundPlaneNeighborhoodPoints);
    }


  // Ground plane estimation

  // ** Extract bounding box neighborhood for ground plane estimation **
  // Find min and max in x- and y-directions
  float xMax = cloudPlantGroundPlaneNeighborhoodPoints->points[0].x;
  float xMin = cloudPlantGroundPlaneNeighborhoodPoints->points[0].x;
  float yMax = cloudPlantGroundPlaneNeighborhoodPoints->points[0].y;
  float yMin = cloudPlantGroundPlaneNeighborhoodPoints->points[0].y;
  for (int i = 1; i < cloudPlantGroundPlaneNeighborhoodPoints->points.size(); i++)
    {
      if (xMax < cloudPlantGroundPlaneNeighborhoodPoints->points[i].x)
        {
          xMax = cloudPlantGroundPlaneNeighborhoodPoints->points[i].x;
        }
      if (xMin > cloudPlantGroundPlaneNeighborhoodPoints->points[i].x)
        {
          xMin = cloudPlantGroundPlaneNeighborhoodPoints->points[i].x;
        }
      if (yMax < cloudPlantGroundPlaneNeighborhoodPoints->points[i].y)
        {
          yMax = cloudPlantGroundPlaneNeighborhoodPoints->points[i].y;
        }
      if (yMin > cloudPlantGroundPlaneNeighborhoodPoints->points[i].y)
        {
          yMin = cloudPlantGroundPlaneNeighborhoodPoints->points[i].y;
        }
    }
  processTime = std::clock() - processTime;
  pcl::console::print_value("Ground patch extrema points estimated in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  processTime = std::clock();
  // Find indices of points within bounding box
  pcl::PointIndices::Ptr indicesInsideGroundPlanePointBoundingBox (new pcl::PointIndices);
  for (int i = 0; i < pointsIndicesBelowExGR->indices.size(); i++)
    {
      int idx = pointsIndicesBelowExGR->indices[i];
      if ((cloudFull->points[idx].x > xMin) && (cloudFull->points[idx].x < xMax) && // Check x-coordinate
          (cloudFull->points[idx].y > yMin) && (cloudFull->points[idx].y < yMax)) // Check y-coordinate
        {
          indicesInsideGroundPlanePointBoundingBox->indices.push_back(idx);
        }
    }
  processTime = std::clock() - processTime;
  pcl::console::print_value("Ground patch points (loop over indices) within extrema points bounding box in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  // Fit ground plane to neighborhood soil pixels
  pcl::ModelCoefficients::Ptr modelCoefficientsGroundPlane (new pcl::ModelCoefficients);
  float z0;
  processTime = std::clock();
  z0 = estimateGroundPlane(cloudFull, indicesInsideGroundPlanePointBoundingBox, planeDistanceThreshold_bed, modelCoefficientsGroundPlane, centerX, centerY);
  pcl::console::print_value("      z02 = %f\n", z0);
  processTime = std::clock() - processTime;
  pcl::console::print_value("Plane fitting (indices) in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  /*
   * FULL CLOUD END
   *
   * */



  /*
   * NEIGHBORHOOD BEGIN
   *
   * */


  // Extract neighborhood

  // Extract neighborhood for performance
  processTime = std::clock();
  pcl::PointIndices::Ptr indicesWithinNeighborhoodPerformance(new pcl::PointIndices);
  pcl::PointIndices::Ptr indicesOutsideNeighborhoodPerformance(new pcl::PointIndices);
  extractNeighborhoodCircle(cloudFull, allPointsIndices, centerX, centerY, performanceNeighborhoodRadius, indicesWithinNeighborhoodPerformance, indicesOutsideNeighborhoodPerformance);
  processTime = std::clock() - processTime;
  pcl::console::print_value("Extracted neigborhood (indices) in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

  if (showClouds)
    {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::copyPointCloud(*cloudFull, indicesWithinNeighborhoodPerformance->indices, *cloud_tmp);
      pcViewer.addCloud("Plant neigborhood \"full\" (indices)",cloud_tmp); // Verified visually same
    }


  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudNeighborhood (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud(*cloudFull, indicesWithinNeighborhoodPerformance->indices, *cloudNeighborhood);
  pcl::PointIndices::Ptr allPointsIndicesInNeighborhood (new pcl::PointIndices);
  pointCloud2pointIndices(cloudNeighborhood, allPointsIndicesInNeighborhood);

  pcl::PointCloud<pcl::PointXYZL>::Ptr cloudNeighborhoodGroundTruthLabels (new pcl::PointCloud<pcl::PointXYZL>);
  if (compareToLabels)
    {
      pcl::copyPointCloud(*cloudFullGroundTruthLabels, indicesWithinNeighborhoodPerformance->indices, *cloudNeighborhoodGroundTruthLabels);
      if (showClouds)
        {
          pcViewer.addCloud("Ground truth, neighborhood",cloudNeighborhoodGroundTruthLabels);
        }
    }

  // Extract neighborhood around LOI for segmentation/further processing
  processTime = std::clock();
  pcl::PointIndices::Ptr indicesWithinNeighborhood(new pcl::PointIndices);
  copyPointIndices(allPointsIndicesInNeighborhood, indicesWithinNeighborhood);

  pcl::PointIndices::Ptr LOIindices  (new pcl::PointIndices);
  copyPointIndices(indicesWithinNeighborhood, LOIindices);

  pcl::console::print_highlight("Step 1: Remove dark points");
  pcl::PointIndices::Ptr lightPointsIndicesInPlantNeighborhood (new pcl::PointIndices);
  if (!skipPreprocess)
    {
    // Remove dark points, e.g shadow edges around the leafs
    processTime = std::clock();
    pcl::PointIndices::Ptr darkPointsIndicesInPlantNeighborhood (new pcl::PointIndices);
    removeDarkPoints(cloudNeighborhood, indicesWithinNeighborhood, darkPointsThreshold_neighborhood, lightPointsIndicesInPlantNeighborhood, darkPointsIndicesInPlantNeighborhood);
    processTime = std::clock() - processTime;
    pcl::console::print_value("Remove dark points (indices) in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

    if (showClouds)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        pcl::copyPointCloud(*cloudNeighborhood, lightPointsIndicesInPlantNeighborhood->indices, *cloud_tmp);
        pcViewer.addCloud("No dark (indices)",cloud_tmp); // Verified visually same
      }
      copyPointIndices(lightPointsIndicesInPlantNeighborhood, LOIindices);
    }
  else
    {
      pcl::console::print_info("Skipped!");
    }

  pcl::console::print_highlight("Step 3: Height filtering");
  pcl::PointIndices::Ptr heightIndicesAboveThreshold (new pcl::PointIndices);
  pcl::PointIndices::Ptr heightIndicesBelowThreshold (new pcl::PointIndices);
  if (!skipHeightFilter)
    {
      processTime = std::clock();
      pcl::PointIndices::Ptr pointsIndicesAboveExGRInPlantNeighborhood (new pcl::PointIndices);
      pcl::PointIndices::Ptr pointsIndicesBelowExGRInPlantNeighborhood (new pcl::PointIndices);
      ExGRfilter(cloudNeighborhood, lightPointsIndicesInPlantNeighborhood, ExGRThreshold_neighborhood, pointsIndicesAboveExGRInPlantNeighborhood, pointsIndicesBelowExGRInPlantNeighborhood);
      processTime = std::clock() - processTime;
      pcl::console::print_value("Extract ExGR (indices) in %f s\n",((float)processTime)/CLOCKS_PER_SEC);

      if (showClouds)
        {
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

          pcl::copyPointCloud(*cloudNeighborhood, pointsIndicesBelowExGRInPlantNeighborhood->indices, *cloud_tmp);
          pcViewer.addCloud("Soil and dead plants (indices)",cloud_tmp); // Verified visually same

          pcl::copyPointCloud(*cloudNeighborhood, pointsIndicesAboveExGRInPlantNeighborhood->indices, *cloud_tmp);
          pcViewer.addCloud("Plant material (indices)",cloud_tmp); // Verified visually same
        }

      pcl::PointIndices::Ptr heightIndices_tmp2 (new pcl::PointIndices);

      pcl::PointXYZ stemPoint;
      stemPoint.x = centerX;
      stemPoint.y = centerY;
      stemPoint.z = z0;
      heightFilter(cloudNeighborhood, lightPointsIndicesInPlantNeighborhood, stemPoint, plantHeightThreshold, plantHeightOffset, heightIndicesAboveThreshold, heightIndices_tmp2);
      heightFilter(cloudNeighborhood, pointsIndicesAboveExGRInPlantNeighborhood, stemPoint, plantHeightThreshold, plantHeightOffset, heightIndices_tmp2, heightIndicesBelowThreshold);

      if (showClouds)
        {
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

          pcl::copyPointCloud(*cloudNeighborhood, heightIndicesBelowThreshold->indices, *cloud_tmp);
          pcViewer.addCloud("Plant material below threshold (indices)",cloud_tmp); // Verified visually same

          pcl::copyPointCloud(*cloudNeighborhood, heightIndicesAboveThreshold->indices, *cloud_tmp);
          pcViewer.addCloud("Plant material above threshold (indices)",cloud_tmp); // Verified visually same
        }

      copyPointIndices(heightIndicesAboveThreshold, LOIindices);
    }
  else
    {
      pcl::console::print_info("Skipped!");
    }



  // Beginning of TOP LEAF SEGMENTATION AND CLASSIFICATION
  pcl::console::print_highlight("Step 4: Leaf segmentation and classification");
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr treeNorm2 = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> > (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  pcl::PointCloud <pcl::Normal>::Ptr normals2 (new pcl::PointCloud <pcl::Normal>);
  std::vector <pcl::PointIndices> clusters2;
  std::vector <int> clustersPlantIdx;
  pcl::PointIndices::Ptr clusterInlier  (new pcl::PointIndices);
  if (!skipSegLeaves)
    {
      pcl::console::print_info("Estimating normals of top leafs...\n");
      pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normal_estimator2;
      normal_estimator2.setSearchMethod (treeNorm2);
      normal_estimator2.setInputCloud (cloudNeighborhood);
      normal_estimator2.setKSearch (normalsNumNeighbors);
      normal_estimator2.compute (*normals2);

      pcl::console::print_info("Grow regions based on normals...\n");
      pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg2;
      reg2.setSearchMethod (treeNorm2);
      reg2.setNumberOfNeighbours (leafClusteringNumNeighbors);
      reg2.setInputCloud (cloudNeighborhood);
      reg2.setIndices (heightIndicesAboveThreshold);
      reg2.setInputNormals (normals2);
      reg2.setSmoothnessThreshold (leafClusteringSmoothessThreshold / 180.0 * M_PI);
      reg2.setCurvatureThreshold (leafClusteringCurvatureThreshold);

      pcl::console::print_value("leafClusteringNumNeighbors: %d\n",leafClusteringNumNeighbors);
      pcl::console::print_value("leafClusteringSmoothessThreshold: %f\n",leafClusteringSmoothessThreshold);
      pcl::console::print_value("leafClusteringCurvatureThreshold: %f\n",leafClusteringCurvatureThreshold);

      pcl::console::print_info("Extract clusters...\n");
      reg2.extract (clusters2);


      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusterCentroids (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusterCentroidsPlant (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      std::vector <int> largeClusterIdx;
      std::vector <int> smallClusterIdx;
      pcl::PointXYZRGBNormal centroidPoint;
      float plantDistMinThreshold = leafAssignmentInnerDist;
      float plantDistMaxThreshold = leafAssignmentOuterDist;
      int minLeafClusterSize = leafClusteringMinSize;

      pcl::console::print_value("plantDistMinThreshold: %f\n",plantDistMinThreshold);
      pcl::console::print_value("plantDistMaxThreshold: %f\n",plantDistMaxThreshold);
      pcl::console::print_value("minLeafClusterSize: %d\n",minLeafClusterSize);
      for (int c = 0; c < clusters2.size(); c++)
        {
          pcl::CentroidPoint<pcl::PointXYZRGBNormal> centroid;
          for (int i = 0; i < clusters2[c].indices.size(); i++) // Move this for loop into the if below! Centroid is only used, if the cluster is large enough
            {
              int idx = clusters2[c].indices[i];
              centroid.add(cloudNeighborhood->points[idx]);
            }
          if (clusters2[c].indices.size() > minLeafClusterSize)
            {
              largeClusterIdx.push_back(c);
              centroid.get(centroidPoint);
              float centroidDistSquared = (centroidPoint.x - centerX)*(centroidPoint.x - centerX) + (centroidPoint.y - centerY)*(centroidPoint.y - centerY);
              if ((centroidDistSquared) > (plantDistMinThreshold*plantDistMinThreshold)) // If centroid is outside circle, check orientation of normal before add
                {
                  if ((centroidDistSquared) < (plantDistMaxThreshold*plantDistMaxThreshold))
                    {
                      clusterCentroids->points.push_back(centroidPoint);

                      Eigen::Vector2f N; // Normal vector of cluster centroid
                      N << centroidPoint.normal_x, centroidPoint.normal_y;

                      Eigen::Vector2f P; // Vector from cluster to plant of interest
                      float px = centerX - centroidPoint.x;
                      float py = centerY - centroidPoint.y;
                      P << px, py;

                      Eigen::Vector2f D1; // Vector from cluster Centroid to neigbor plant point 1
                      float dx = cloudPlantNeighborCenter->points[0].x - centroidPoint.x;
                      float dy = cloudPlantNeighborCenter->points[0].y - centroidPoint.y;
                      D1 << dx, dy;

                      dx = cloudPlantNeighborCenter->points[1].x - centroidPoint.x;
                      dy = cloudPlantNeighborCenter->points[1].y - centroidPoint.y;
                      Eigen::Vector2f D2; // Vector from cluster centroid to neighbor plant point 2
                      D2 << dx, dy;

                      float cosThetaD;
                      if (D1.norm() < D2.norm())
                        {
                          cosThetaD = D1.dot(N)/(D1.norm()*N.norm());
                        }
                      else
                        {
                          cosThetaD = D2.dot(N)/(D2.norm()*N.norm());
                        }

                      float cosThetaP = P.dot(N)/(P.norm()*N.norm());

                      if (cosThetaP > cosThetaD) // Larger cos(theta) --> smaller (absolute) angle
                        {
                          clusterCentroidsPlant->points.push_back(centroidPoint);
                          clustersPlantIdx.push_back(c);
                        }
                      else
                        {
                          // Do nothing...
                        }
                    }
                }
              else // If centroid is within radius, add it no matter what
                {
                  clusterCentroidsPlant->points.push_back(centroidPoint);
                  clustersPlantIdx.push_back(c);
                }
            }
          else // Cluster too small; add to list of small clusters
            {
              smallClusterIdx.push_back(c);
            }
        }
      clusterCentroids->width = clusterCentroids->points.size();
      clusterCentroids->height = 1;
      clusterCentroidsPlant->width = clusterCentroidsPlant->points.size();
      clusterCentroidsPlant->height = 1;

      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud2 = reg2.getColoredCloud ();
      if (showClouds)
        {
          pcViewer.addCloud("Top leaf clusters2",colored_cloud2);
          pcViewer.addCloud("Top leaf centroids all",clusterCentroids);
          pcViewer.addCloud("Top leaf centroids filtered",clusterCentroidsPlant);
        }

      if (showClouds)
        {
          // Reconstruct cloud from clusters
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantClusterReconstructed (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          reconstructFromClusters(cloudNeighborhood, clusters2, clustersPlantIdx, cloudPlantClusterReconstructed);
          pcViewer.addCloud("Clusters reconstructed",cloudPlantClusterReconstructed);
        }


      processTime = std::clock();
      // KNN
      // 1) Add clusters to labels
      pcl::PointCloud<pcl::PointXYZL>::Ptr cloudLabels (new pcl::PointCloud<pcl::PointXYZL>);
      pcl::PointIndices cloudLabelsIndices;
      pcl::PointXYZL pointLabel;
      for (int c = 0; c < largeClusterIdx.size(); c++)
        {
          int clusterIdx = largeClusterIdx[c];
          std::vector<int> _indices = clusters2[clusterIdx].indices;
          for (int i = 0; i < _indices.size(); i++)
            {
              int idx = _indices[i];
              pointLabel.x = cloudNeighborhood->points[idx].x;
              pointLabel.y = cloudNeighborhood->points[idx].y;
              pointLabel.z = cloudNeighborhood->points[idx].z;
              pointLabel.label = clusterIdx;
              cloudLabels->points.push_back(pointLabel);
              cloudLabelsIndices.indices.push_back(idx);
            }
        }
      cloudLabels->width = cloudLabels->points.size();
      cloudLabels->height = 1;


      pcl::PointCloud<pcl::PointXYZL>::Ptr cloudAllLabels (new pcl::PointCloud<pcl::PointXYZL>);
      pcl::PointIndices cloudAllLabelsIndices = cloudLabelsIndices;
      pcl::copyPointCloud(*cloudLabels,*cloudAllLabels);


      // 2) kD-tree
      pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
      kdtree.setInputCloud(cloudLabels);

      // 3) Loop through all small clusters and assign each pixel to a large cluster based on KNN and inverse distance measure
      int K = leafClusteringKNN;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      pcl::PointXYZL searchPoint;
      for (int c = 0; c < smallClusterIdx.size(); c++)
        {
          int clusterIdx = smallClusterIdx[c];
          std::vector<int> _indices = clusters2[clusterIdx].indices;
          for (int i = 0; i < _indices.size(); i++)
            {
              int idx = _indices[i];
              searchPoint.x = cloudNeighborhood->points[idx].x;
              searchPoint.y = cloudNeighborhood->points[idx].y;
              searchPoint.z = cloudNeighborhood->points[idx].z;
              if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                {
                  int label = pointIdx2label(cloudLabels, pointIdxNKNSearch, pointNKNSquaredDistance, 0.05);
                  if (label < 0)
                    {
                      searchPoint.label = clusters2.size()+1;
                    }
                    else
                    {
                       searchPoint.label = label;
                    }
                }
              else
                {
                  searchPoint.label = clusters2.size()+1;
                }
              cloudAllLabels->points.push_back(searchPoint);
              cloudAllLabelsIndices.indices.push_back(idx);
            }
        }
      cloudAllLabels->width = cloudAllLabels->points.size();
      cloudAllLabels->height = 1;

      if (showClouds)
        {
          pcViewer.addCloud("Cluster labels",cloudLabels);
          pcViewer.addCloud("Cluster labels KNN",cloudAllLabels);
        }

      // Reconstruct
      pcl::console::print_value("cloudAllLabels->points.size() = %d\n", cloudAllLabels->points.size());
      for (int c = 0; c < clustersPlantIdx.size(); c++)
        {
          for (int i = 0; i < cloudAllLabels->points.size(); i++)
            {
              if (cloudAllLabels->points[i].label == clustersPlantIdx[c])
                {
                  int idx = cloudAllLabelsIndices.indices[i];
                  clusterInlier->indices.push_back(idx);
                }
            }
        }
      pcl::console::print_info("Done adding points.\n");

      if (showClouds)
        {
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantClusterFullyReconstructed (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          pcl::copyPointCloud(*cloudNeighborhood,clusterInlier->indices, *cloudPlantClusterFullyReconstructed);
          pcViewer.addCloud("Clusters reconstructed fully",cloudPlantClusterFullyReconstructed);
        }
      copyPointIndices(clusterInlier, LOIindices);
    }
  else
    {
      pcl::console::print_info("Skipped!");
    }
  // End of TOP LEAF SEGMENTATION AND CLASSIFICATION

  pcl::console::print_highlight("Step 5: Including lower leaves");
  if (!skipSubLeaves)
    {
      float rMaxSquared = 0;
      for (int i = 0; i < clusterInlier->indices.size(); i++)
        {
          int idx = clusterInlier->indices[i];
          float x = cloudNeighborhood->points[idx].x - centerX;
          float y = cloudNeighborhood->points[idx].y - centerY;
          float rThisSquared = x*x + y*y;
          if (rThisSquared > rMaxSquared)
            {
              rMaxSquared = rThisSquared;
            }
        }
      float rMax = sqrt(rMaxSquared);
      pcl::console::print_value("Top max leaf radius: %f\n", rMax);

      int avgClusterSize = 0;
      std::vector<int> clusterSizes;
      for (int i = 0; i < clustersPlantIdx.size(); i++)
        {
          avgClusterSize += clusters2[clustersPlantIdx[i]].indices.size();
          clusterSizes.push_back(clusters2[clustersPlantIdx[i]].indices.size());
        }
      double median;
      size_t numClusters = clusterSizes.size();

      sort(clusterSizes.begin(), clusterSizes.end());

      if (numClusters > 0)
        {
          if (numClusters % 2 == 0)
          {
              median = (clusterSizes[numClusters / 2 - 1] + clusterSizes[numClusters / 2]) / 2;
          }
          else
          {
              median = clusterSizes[numClusters / 2];
          }
            avgClusterSize /= numClusters;
        }


      pcl::console::print_info("Average cluster: "); pcl::console::print_value("%d points\n", avgClusterSize);
      pcl::console::print_info("Median cluster: "); pcl::console::print_value("%d points\n", (int)median);


    /*
     * Sub leaf clustering
     * */

      pcl::console::print_info("Grow regions based on normals...\n");
      pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> regSubLeafs;
      regSubLeafs.setMinClusterSize (avgClusterSize/2);
      regSubLeafs.setSearchMethod (treeNorm2);
      regSubLeafs.setNumberOfNeighbours (leafClusteringNumNeighbors);
      regSubLeafs.setInputCloud (cloudNeighborhood);
      regSubLeafs.setIndices (heightIndicesBelowThreshold);
      regSubLeafs.setInputNormals (normals2);
      regSubLeafs.setSmoothnessThreshold (leafClusteringSmoothessThreshold / 180.0 * M_PI);
      regSubLeafs.setCurvatureThreshold (leafClusteringCurvatureThreshold);

      pcl::console::print_info("Extract clusters...\n");
      std::vector <pcl::PointIndices> clustersSubLeafs;
      regSubLeafs.extract (clustersSubLeafs);
      if (clustersSubLeafs.size() > 0)
        {
          if (showClouds)
            {
              pcViewer.addCloud("Sub leaf clusters",regSubLeafs.getColoredCloud());
            }
        }

      float xCent;
      float yCent;
      pcl::console::print_info("Number of clusters: "); pcl::console::print_value("%d\n",clustersSubLeafs.size () );
      for (int i = 0; i < clustersSubLeafs.size(); i++)
        {
          xCent = 0;
          yCent = 0;
          for (int j = 0; j < clustersSubLeafs[i].indices.size(); j++)
            {
              xCent += cloudNeighborhood->points[clustersSubLeafs[i].indices[j]].x;
              yCent += cloudNeighborhood->points[clustersSubLeafs[i].indices[j]].y;
            }
          xCent /= clustersSubLeafs[i].indices.size();
          yCent /= clustersSubLeafs[i].indices.size();
          xCent = xCent - centerX;
          yCent = yCent - centerY;
          float rThisSquared = xCent*xCent + yCent*yCent;

          if (rThisSquared < rMaxSquared)
            {
              for (int j = 0; j < clustersSubLeafs[i].indices.size(); j++)
                {
                  clusterInlier->indices.push_back(clustersSubLeafs[i].indices[j]);
                }

            }
        }

      copyPointIndices(clusterInlier, LOIindices);
      pcl::console::print_info("Subleafs added!\n");
    }
  else
    {
      pcl::console::print_info("Skipped!");
    }

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPlantClusterFullyReconstructedFinal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud(*cloudNeighborhood, LOIindices->indices, *cloudPlantClusterFullyReconstructedFinal);

  pcl::console::print_info("Cloud copied!\n");

  if (showClouds)
    {
      pcViewer.addCloud("Plant with subleafs (indices)",cloudPlantClusterFullyReconstructedFinal);
    }


  pcl::PointCloud<pcl::PointXYZL>::Ptr cloudLabelsOfLettuce (new pcl::PointCloud<pcl::PointXYZL>);
  int TP = 0;
  int FP = 0;
  int FN = 0;
  int TN = 0;
  int P = 0;
  int N = 0;
  float precision = 0.0;
  float recall = 0.0;
  float f1score = 0.0;
  Eigen::MatrixXi ConfusionMatrix(3,2);
  ConfusionMatrix(0,0) = 0;
  ConfusionMatrix(1,0) = 0;
  ConfusionMatrix(2,0) = 0;
  ConfusionMatrix(0,1) = 0;
  ConfusionMatrix(1,1) = 0;
  ConfusionMatrix(2,1) = 0;

  if (compareToLabels)
    {
      pcl::copyPointCloud(*cloudNeighborhoodGroundTruthLabels, LOIindices->indices, *cloudLabelsOfLettuce);

      for (int i = 0; i < cloudLabelsOfLettuce->points.size(); i++)
        {
          if (cloudLabelsOfLettuce->points[i].label == 0)
            {
              ConfusionMatrix(0,0) += 1;
            }
          else if (cloudLabelsOfLettuce->points[i].label == 1)
            {
              ConfusionMatrix(1,0) += 1;
            }
          else if (cloudLabelsOfLettuce->points[i].label == 2)
            {
              ConfusionMatrix(2,0) += 1;
            }
        }

      for (int i = 0; i < cloudNeighborhoodGroundTruthLabels->points.size(); i++)
        {
          if (cloudNeighborhoodGroundTruthLabels->points[i].label == 0)
            {
              ConfusionMatrix(0,1) += 1;
            }
          else if (cloudNeighborhoodGroundTruthLabels->points[i].label == 1)
            {
              ConfusionMatrix(1,1) += 1;
            }
          else if (cloudNeighborhoodGroundTruthLabels->points[i].label == 2)
            {
              ConfusionMatrix(2,1) += 1;
            }
        }

      ConfusionMatrix(0,1) = ConfusionMatrix(0,1) - ConfusionMatrix(0,0);
      ConfusionMatrix(1,1) = ConfusionMatrix(1,1) - ConfusionMatrix(1,0);
      ConfusionMatrix(2,1) = ConfusionMatrix(2,1) - ConfusionMatrix(2,0);

      pcl::console::print_info("\n");
      pcl::console::print_info("Confusion matrix:\n");
      pcl::console::print_info("         :  "); pcl::console::print_info("  Lettuce  ");                   pcl::console::print_info("    Other  ");                    pcl::console::print_info(" : "); pcl::console::print_info("    Total \n");
      pcl::console::print_info(".........:. "); pcl::console::print_info(".......... ");                   pcl::console::print_info(".......... ");                    pcl::console::print_info(".:."); pcl::console::print_info("...........\n");
      pcl::console::print_info("Other    :  "); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(0,0))); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(0,1))); pcl::console::print_info(" : "); pcl::console::print_value(" %8d \n",(int)(ConfusionMatrix(0,0) + ConfusionMatrix(0,1)));
      pcl::console::print_info("Lettuce  :  "); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(1,0))); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(1,1))); pcl::console::print_info(" : "); pcl::console::print_value(" %8d \n",(int)(ConfusionMatrix(1,0) + ConfusionMatrix(1,1)));
      pcl::console::print_info("Neighbor :  "); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(2,0))); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(2,1))); pcl::console::print_info(" : "); pcl::console::print_value(" %8d \n",(int)(ConfusionMatrix(2,0) + ConfusionMatrix(2,1)));
      pcl::console::print_info(".........:. "); pcl::console::print_info(".......... ");                   pcl::console::print_info(".......... ");                    pcl::console::print_info(".:."); pcl::console::print_info("...........\n");
      pcl::console::print_info("Total    :  "); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(0,0)+ConfusionMatrix(1,0)+ConfusionMatrix(2,0))); pcl::console::print_value(" %8d  ",(int)(ConfusionMatrix(0,1)+ConfusionMatrix(1,1)+ConfusionMatrix(2,1))); pcl::console::print_info(" : "); pcl::console::print_value(" %8d \n",(int)(ConfusionMatrix(0,0) + ConfusionMatrix(0,1) + ConfusionMatrix(1,0) + ConfusionMatrix(1,1) + ConfusionMatrix(2,0) + ConfusionMatrix(2,1)));
      pcl::console::print_info("\n");

      TP = ConfusionMatrix(1,0);
      FP = ConfusionMatrix(0,0) + ConfusionMatrix(2,0);
      FN = ConfusionMatrix(1,1);
      TN = ConfusionMatrix(0,1) + ConfusionMatrix(2,1);

      precision = (float)TP/((float)TP + (float)FP);
      recall = (float)TP/((float)TP + (float)FN);
      f1score = (2.0 * (float)TP) / (2.0 * (float)TP + (float)FP + (float)FN);

      pcl::console::print_info("Precision: "); pcl::console::print_value("%f\n",precision);
      pcl::console::print_info("Recall   : "); pcl::console::print_value("%f\n",recall);
      pcl::console::print_info("F1-score : "); pcl::console::print_value("%f\n",f1score);
      pcl::console::print_info("\n");


      if (saveConfusionMatrix)
        {
          pcl::console::print_info("Writing confusion matrix to csv file: ");
          pcl::console::print_value("%s\n",filenameOutConfusionMatrix.c_str());
          FILE * pFile;
          pFile = fopen(filenameOutConfusionMatrix.c_str(),"w");
          fprintf(pFile,"%s,%s,%s\n","Observed\\predicted","Lettuce","Other (incl. neighbors)");
          fprintf(pFile,"%s,%d,%d\n","Other"   ,ConfusionMatrix(0,0),ConfusionMatrix(0,1));
          fprintf(pFile,"%s,%d,%d\n","Lettuce" ,ConfusionMatrix(1,0),ConfusionMatrix(1,1));
          fprintf(pFile,"%s,%d,%d\n","Neighbors",ConfusionMatrix(2,0),ConfusionMatrix(2,1));

          fclose(pFile);
        }

      if (showClouds)
        {
          pcViewer.addCloud("Ground truth, final cloud",cloudLabelsOfLettuce);
        }
    }


  processTimeTotal = std::clock() - processTimeTotal;
  pcl::console::print_highlight("Total processing time: ");
  pcl::console::print_value("%f s\n",((float)processTimeTotal)/CLOCKS_PER_SEC);


  if (showClouds)
    {
      // Find max z-coordinate of LOI
      float maxZ = cloudPlantClusterFullyReconstructedFinal->points[0].z;
      for (int i = 0; i < cloudPlantClusterFullyReconstructedFinal->points.size(); i++)
        {
          if (maxZ < cloudPlantClusterFullyReconstructedFinal->points[i].z)
            {
              maxZ = cloudPlantClusterFullyReconstructedFinal->points[i].z;
            }
        }
      // Start viewer
      pcViewer.singleViewer();

      // Add ground plane
      pcViewer.viewer->addPlane(*modelCoefficientsGroundPlane, centerX, centerY, z0);
      pcl::ModelCoefficients::Ptr circle (new pcl::ModelCoefficients);
      circle->values.push_back(0.0);
      circle->values.push_back(0.0);
      circle->values.push_back(0.15);

      // Add cylinder showing approximate area around LOI
      pcl::ModelCoefficients::Ptr cylinder (new pcl::ModelCoefficients);
      cylinder->values.push_back(centerX);
      cylinder->values.push_back(centerY);
      cylinder->values.push_back(z0);
      cylinder->values.push_back(0.0);
      cylinder->values.push_back(0.0);
      cylinder->values.push_back(maxZ-z0);
      cylinder->values.push_back(0.15);
      pcViewer.viewer->addCylinder(*cylinder);
      // Add coordinate systems
      pcViewer.viewer->addCoordinateSystem(0.1,centerX,centerY,z0);
      pcViewer.viewer->addCoordinateSystem(0.1,0.0,0.0,0);

      // Add line and text for adjacent lettuce #1
      pcl::PointXYZ plantNeighbor1Point1;
      plantNeighbor1Point1.x = cloudPlantNeighborCenter->points[0].x;
      plantNeighbor1Point1.y = cloudPlantNeighborCenter->points[0].y;
      plantNeighbor1Point1.z = z0;
      pcl::PointXYZ plantNeighbor1Point2;
      plantNeighbor1Point2.x = cloudPlantNeighborCenter->points[0].x;
      plantNeighbor1Point2.y = cloudPlantNeighborCenter->points[0].y;
      plantNeighbor1Point2.z = maxZ;
      pcViewer.viewer->addLine(plantNeighbor1Point1, plantNeighbor1Point2,0.0,0.0,1.0,"Plant line #1");
      pcViewer.viewer->addText3D("D1",plantNeighbor1Point2,0.01,0.0,0.0,1.0,"D1");

      // Add line and text for adjacent lettuce #2
      pcl::PointXYZ plantNeighbor2Point1;
      plantNeighbor2Point1.x = cloudPlantNeighborCenter->points[1].x;
      plantNeighbor2Point1.y = cloudPlantNeighborCenter->points[1].y;
      plantNeighbor2Point1.z = z0;
      pcl::PointXYZ plantNeighbor2Point2;
      plantNeighbor2Point2.x = cloudPlantNeighborCenter->points[1].x;
      plantNeighbor2Point2.y = cloudPlantNeighborCenter->points[1].y;
      plantNeighbor2Point2.z = maxZ;
      pcViewer.viewer->addLine(plantNeighbor2Point1, plantNeighbor2Point2,0.0,0.0,1.0,"Plant line #2");
      pcViewer.viewer->addText3D("D2",plantNeighbor2Point2,0.01,0.0,0.0,1.0,"D2");

      // Spin viewer until user exits it
      pcViewer.spin();
    }

  if (saveOutput)
    {
      // Shift plant to z=0
      for (int i = 0; i < cloudPlantClusterFullyReconstructedFinal->points.size(); i++)
        {
          cloudPlantClusterFullyReconstructedFinal->points[i].x = cloudPlantClusterFullyReconstructedFinal->points[i].x - centerX;
          cloudPlantClusterFullyReconstructedFinal->points[i].y = cloudPlantClusterFullyReconstructedFinal->points[i].y - centerY;
          cloudPlantClusterFullyReconstructedFinal->points[i].z = cloudPlantClusterFullyReconstructedFinal->points[i].z - z0;
        }
      pcl::io::savePCDFileBinary(filenameOut, *cloudPlantClusterFullyReconstructedFinal);

      if (filenameZ0.compare("NotSet")) // Returns 0 if equal to "NotSet"
        {
          pcl::console::print_info("Writing Z0 to csv file: ");
          pcl::console::print_value("%s\n",filenameZ0.c_str());
          FILE * pFile;
          pFile = fopen(filenameZ0.c_str(),"w");
          fprintf(pFile,"%s,%.8e,%s\n","z0",z0,"m");

          fclose(pFile);

        }
    }

  return 0;
}
