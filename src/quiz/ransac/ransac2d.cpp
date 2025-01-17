/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++)
  {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--)
  {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);

  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(const std::string& file_name)
{
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd(file_name);
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

template <typename Point3>
pcl::PointXYZI get_plane(Point3& p1, Point3& p2, Point3& p3)
{
  float a1 = p2.x - p1.x;
  float b1 = p2.y - p1.y;
  float c1 = p2.z - p1.z;
  float a2 = p3.x - p1.x;
  float b2 = p3.y - p1.y;
  float c2 = p3.z - p1.z;

  float a = b1 * c2 - b2 * c1;
  float b = a2 * c1 - a1 * c2;
  float c = a1 * b2 - b1 * a2;
  float d = (-a * p1.x - b * p1.y - c * p1.z);

  pcl::PointXYZI plane;
  plane.x = a;
  plane.y = b;
  plane.z = c;
  plane.intensity = d;
  return plane;
}

template <typename Point3, typename Point4>
float distance_to_plane(Point3& pt, Point4& plane) {

  return std::fabs(plane.x * pt.x + plane.y * pt.y + plane.z * pt.z + plane.intensity) / std::sqrtf(plane.x * plane.x + plane.y * plane.y + plane.z * plane.z);

}

template<typename Point3>
std::unordered_set<int> Ransac(typename pcl::PointCloud<Point3>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // For max iterations 
  while (maxIterations--) {

    // Randomly sample subset and fit line
    std::unordered_set<int> inliers;
    while (inliers.size() < 3) {
      inliers.insert(rand() % (cloud->points.size()));
    }
    auto it = inliers.begin();
    auto pt1 = cloud->points[*it++];
    auto pt2 = cloud->points[*it++];
    auto pt3 = cloud->points[*it];

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    auto plane = get_plane<pcl::PointXYZ>(pt1, pt2, pt3);
    for (int idx = 0; idx < cloud->points.size(); idx++) {
      if (inliers.count(idx) > 0) {
        continue;
      }

      auto pt = cloud->points[idx];
      if (distance_to_plane(pt, plane) < distanceTol) {
        inliers.insert(idx);
      }
    }

    // Return indicies of inliers from fitted line with most inliers
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  return inliersResult;
}

int main()
{

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  // Define file location in CMakeLists.txt
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(POINT_CLOUD_FILE);


  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }


  // Render 2D point cloud with inliers and outliers
  if (inliers.size())
  {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  }
  else
  {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

}
