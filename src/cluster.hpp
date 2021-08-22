#pragma once

#include "render/render.h"
#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

template <typename PointT>
using eigen_vector = std::vector<PointT, Eigen::aligned_allocator<PointT> >;

template <typename PointT, int Depth>
void clusterHelper(int idx, const eigen_vector<PointT>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT, Depth>* tree, float distanceTol, int maxCluster) {

  if (cluster.size() >= maxCluster) {
    return;
  }

  processed[idx] = true;
  cluster.push_back(idx);

  std::vector<int> nearest = tree->search(points[idx], distanceTol);
  for (int id : nearest) {
    if (!processed[id]) {
      clusterHelper(id, points, cluster, processed, tree, distanceTol, maxCluster);
    }
  }
}

template <typename PointT, int Depth>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>& pointCloud, KdTree<PointT, Depth>* tree, float distanceTol, int minCluster, int maxCluster)
{

  auto& points = pointCloud.points;

  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(points.size(), false);

  for (int i = 0; i < points.size(); i++) {
    if (processed[i]) {
      continue;
    }

    std::vector<int> cluster;
    clusterHelper(i, points, cluster, processed, tree, distanceTol, maxCluster);
    if (cluster.size() >= minCluster) {
      clusters.push_back(cluster);
    }
  }
  return clusters;
}
