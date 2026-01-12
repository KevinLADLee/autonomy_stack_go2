#ifndef LOCAL_PLANNER__PATH_FILE_IO_HPP_
#define LOCAL_PLANNER__PATH_FILE_IO_HPP_

#include <fstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace local_planner::io
{

/// Reads the PLY file header and returns the number of points.
/// Returns -1 on error.
int readPlyHeader(std::ifstream& file);

/// Reads start paths from startPaths.ply into the provided vector.
/// Returns true on success, false on error.
bool readStartPaths(
  const std::string& pathFolder,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& startPaths,
  int groupNum);

/// Reads paths from paths.ply into the provided vector, with optional point skipping.
/// Returns true on success, false on error.
bool readPaths(
  const std::string& pathFolder,
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& paths,
  int pathNum,
  int pointSkipNum = 30);

/// Reads path list from pathList.ply, populating path-to-group mapping and end directions.
/// Returns true on success, false on error.
bool readPathList(
  const std::string& pathFolder,
  std::vector<int>& pathList,
  std::vector<float>& endDirPathList,
  int pathNum,
  int groupNum);

/// Reads correspondences from correspondences.txt, mapping grid voxels to paths.
/// Returns true on success, false on error.
bool readCorrespondences(
  const std::string& pathFolder,
  std::vector<std::vector<int>>& correspondences,
  int gridVoxelNum,
  int pathNum);

}  // namespace local_planner::io

#endif  // LOCAL_PLANNER__PATH_FILE_IO_HPP_
