/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include <cmath>


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

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
    auto plane = get_plane<PointT>(pt1, pt2, pt3);
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

