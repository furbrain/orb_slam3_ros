#ifndef UTILS_H
#define UTILS_H
#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"

struct RansacResult {
    Sophus::SE3f T;
    std::vector<int> inlierIndices;
    int numInliers = 0;
};

ORB_SLAM3::Atlas* load_atlas_from_file(const std::string &url);
ORB_SLAM3::Atlas* prepare_atlas(std::string url, std::string strVocFile = "");
void save_atlas_to_file(ORB_SLAM3::Atlas* atlas, const std::string &url, std::string strVocFile = "");
vector<ORB_SLAM3::Map*> merge_atlas(ORB_SLAM3::Atlas* atlas, std::string url);
ORB_SLAM3::Map* get_biggest_map(ORB_SLAM3::Atlas* atlas);
void alignAtlas(ORB_SLAM3::Atlas* atlas);
void alignMap(ORB_SLAM3::Map* map);
RansacResult RansacHornAlignmentPy(const Eigen::MatrixX3f& src,  // 3xN
                                    const Eigen::MatrixX3f& dst,  // 3xN
                                    float inlierThreshold = 0.1, // metres — tune to your map's scale/noise
                                    int maxIterations = 2000,
                                    int minInliersToAccept = 10);
void OfflineMergeMaps(ORB_SLAM3::Map* pMapA, ORB_SLAM3::Map* pMapB,
                      const Sophus::SE3f& TransformBtoA,
                      const std::vector<std::pair<ORB_SLAM3::MapPoint*, ORB_SLAM3::MapPoint*>>& vMatchedPairs);
void bundle_adjustment(ORB_SLAM3::Map* pMap, int num_iterations);                      
#endif