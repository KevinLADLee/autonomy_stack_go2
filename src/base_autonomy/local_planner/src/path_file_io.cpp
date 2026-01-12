#include "local_planner/path_file_io.hpp"

#include <cmath>

namespace local_planner::io
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

int readPlyHeader(std::ifstream& file)
{
  std::string current;
  std::string previous;
  int pointNum = 0;

  while (current != "end_header") {
    if (!(file >> current)) {
      return -1;
    }

    if (current == "vertex" && previous == "element") {
      if (!(file >> pointNum)) {
        return -1;
      }
    }
    previous = current;
  }

  return pointNum;
}

bool readStartPaths(
  const std::string& pathFolder,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& startPaths,
  int groupNum)
{
  std::ifstream file(pathFolder + "/startPaths.ply");
  if (!file.is_open()) {
    return false;
  }

  int pointNum = readPlyHeader(file);
  if (pointNum < 0) {
    return false;
  }

  pcl::PointXYZ point;
  int groupID;

  for (int i = 0; i < pointNum; i++) {
    if (!(file >> point.x >> point.y >> point.z >> groupID)) {
      return false;
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  return true;
}

bool readPaths(
  const std::string& pathFolder,
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& paths,
  int pathNum,
  int pointSkipNum)
{
  std::ifstream file(pathFolder + "/paths.ply");
  if (!file.is_open()) {
    return false;
  }

  int totalPoints = readPlyHeader(file);
  if (totalPoints < 0) {
    return false;
  }

  pcl::PointXYZI point;
  int skipCount = 0;
  int pathID;

  for (int i = 0; i < totalPoints; i++) {
    if (!(file >> point.x >> point.y >> point.z >> pathID >> point.intensity)) {
      return false;
    }

    if (pathID >= 0 && pathID < pathNum) {
      skipCount++;
      if (skipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        skipCount = 0;
      }
    }
  }

  return true;
}

bool readPathList(
  const std::string& pathFolder,
  std::vector<int>& pathList,
  std::vector<float>& endDirPathList,
  int pathNum,
  int groupNum)
{
  std::ifstream file(pathFolder + "/pathList.ply");
  if (!file.is_open()) {
    return false;
  }

  if (pathNum != readPlyHeader(file)) {
    return false;
  }

  int pathID;
  int groupID;
  float endX;
  float endY;
  float endZ;

  for (int i = 0; i < pathNum; i++) {
    if (!(file >> endX >> endY >> endZ >> pathID >> groupID)) {
      return false;
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = static_cast<float>(2.0 * std::atan2(endY, endX) * 180.0 / kPi);
    }
  }

  return true;
}

bool readCorrespondences(
  const std::string& pathFolder,
  std::vector<std::vector<int>>& correspondences,
  int gridVoxelNum,
  int pathNum)
{
  std::ifstream file(pathFolder + "/correspondences.txt");
  if (!file.is_open()) {
    return false;
  }

  int gridVoxelID;
  int pathID;

  for (int i = 0; i < gridVoxelNum; i++) {
    if (!(file >> gridVoxelID)) {
      return false;
    }

    while (true) {
      if (!(file >> pathID)) {
        return false;
      }

      if (pathID == -1) {
        break;
      }

      if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
        correspondences[gridVoxelID].push_back(pathID);
      }
    }
  }

  return true;
}

}  // namespace local_planner::io
