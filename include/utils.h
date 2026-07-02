#ifndef UTILS_H
#define UTILS_H
#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"

ORB_SLAM3::Atlas* load_atlas_from_file(const std::string &url);
ORB_SLAM3::Atlas* prepare_atlas(std::string url, std::string strVocFile = "");
vector<ORB_SLAM3::Map*> merge_atlas(ORB_SLAM3::Atlas* atlas, std::string url);
ORB_SLAM3::Map* get_biggest_map(ORB_SLAM3::Atlas* atlas);
void alignAtlas(ORB_SLAM3::Atlas* atlas);
void alignMap(ORB_SLAM3::Map* map);
#endif
