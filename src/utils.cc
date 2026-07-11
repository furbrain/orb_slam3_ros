#include "utils.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sophus/se3.hpp>
#include <vector>

ORB_SLAM3::ORBVocabulary *vocab = nullptr;

ORB_SLAM3::Atlas* load_atlas_from_file(const std::string &url) {
    ORB_SLAM3::Atlas *atlas = new ORB_SLAM3::Atlas();
    std::ifstream ifs(url, std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    std::string strFileVoc, strVocChecksum;
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> atlas;
    return atlas;
}

void alignMap(ORB_SLAM3::Map* map) {
    // compute the best fit rotation between the camera poses and the IMU poses in the map. 
    // This is a complex problem that typically involves solving a Procrustes problem or using 
    // SVD to find the optimal rotation that minimizes the distance between the two sets of orientations. 
    auto kfs = map->GetAllKeyFrames();
    if (kfs.size() < 5) {
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Not enough keyframes to align map (need at least 5)";
        return;
    }
    Eigen::Matrix3Xf cam_orientations(3,kfs.size()*3);
    Eigen::Matrix3Xf imu_orientations(3, kfs.size()*3);
    for (size_t i = 0; i < kfs.size(); i++) {
        auto kf = kfs[i];
        //if (kf->isBad()) continue;
        auto cam_q = kf->GetPose().so3().inverse();
        auto imu_q = kf->GetPoseFromEstimate().so3().inverse();
        cam_orientations.block<3,3>(0,i*3) = cam_q.matrix();
        imu_orientations.block<3,3>(0,i*3) = imu_q.matrix();
    }
    Eigen::Matrix4f T = Eigen::umeyama(cam_orientations, imu_orientations, false);
    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Sophus::SE3f R_se3(R, Eigen::Vector3f::Zero());
    for (ORB_SLAM3::KeyFrame* pKF : map->GetAllKeyFrames()) {
        if (pKF->isBad()) continue;
        auto pose = R_se3 * pKF->GetPose().inverse();
        pKF->SetPose(pose.inverse());
    }
    for (ORB_SLAM3::MapPoint* pMP : map->GetAllMapPoints()) {
        if (pMP->isBad()) continue;
        auto pos = R_se3 * pMP->GetWorldPos();
        pMP->SetWorldPos(pos);
    }
}

void alignAtlas(ORB_SLAM3::Atlas* atlas) {
    for(ORB_SLAM3::Map* map : atlas->GetAllMaps()) {
        alignMap(map);
    }
}


ORB_SLAM3::Atlas* prepare_atlas(std::string url, std::string strVocFile) {
    auto vocab = new ORB_SLAM3::ORBVocabulary();
    if (strVocFile.empty()) {
            strVocFile = ament_index_cpp::get_package_share_directory("orb_slam3") + "/vocab/ORBvoc.txt.bin";
    }
    bool bVocLoad = vocab->loadFromBinFile(strVocFile);
    if (!bVocLoad) {
        return nullptr;
    }
    auto mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*vocab);
    ORB_SLAM3::Atlas *atlas = load_atlas_from_file(url);
    atlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    atlas->SetORBVocabulary(vocab);
    atlas->PostLoad();
    atlas->PreSave();
    return atlas;
}

vector<ORB_SLAM3::Map*> merge_atlas(ORB_SLAM3::Atlas* atlas, std::string url) {
    // load the second atlas from the url, and merge it with the first atlas.
    // need to fix the keyframeIDs and map point IDs in the second atlas to avoid conflicts with the first atlas.
    //return all the new maps in the merged atlas.
    ORB_SLAM3::Atlas *new_atlas = load_atlas_from_file(url);
    auto vocab = atlas->GetORBVocabulary();
    auto mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*vocab);
    new_atlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    new_atlas->SetORBVocabulary(vocab);
    new_atlas->PostLoad();
    new_atlas->PreSave();
    auto new_maps = new_atlas->GetAllMaps();
    for(ORB_SLAM3::Map* map : new_maps) {
        alignMap(map);
    }
    atlas->ImportAtlas(new_atlas);
    return new_maps;
}


ORB_SLAM3::Map* get_biggest_map(ORB_SLAM3::Atlas* atlas) {
    ORB_SLAM3::Map *best_map;
    int map_size = 0;
    for (ORB_SLAM3::Map* map : atlas->GetAllMaps()) {
        if (map->GetAllKeyFrames().size() >= map_size) {
            best_map = map;
            map_size = map->GetAllKeyFrames().size();
        }
    }
    return best_map;
}


// Returns T such that q_i ≈ T * p_i  (T maps B-frame points into A-frame)
Sophus::SE3f HornAlignment(const std::vector<Eigen::Vector3f>& src, // map B points
                            const std::vector<Eigen::Vector3f>& dst) // map A points
{
    assert(src.size() == dst.size() && src.size() >= 3);
    const int N = src.size();

    // 1. Centroids
    Eigen::Vector3f centroidSrc = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroidDst = Eigen::Vector3f::Zero();
    for (int i = 0; i < N; ++i) { centroidSrc += src[i]; centroidDst += dst[i]; }
    centroidSrc /= N;
    centroidDst /= N;

    // 2. Center the point sets
    std::vector<Eigen::Vector3f> srcC(N), dstC(N);
    for (int i = 0; i < N; ++i) {
        srcC[i] = src[i] - centroidSrc;
        dstC[i] = dst[i] - centroidDst;
    }

    // 3. Cross-covariance matrix H = sum(srcC_i * dstC_i^T)
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (int i = 0; i < N; ++i)
        H += srcC[i] * dstC[i].transpose();

    // 4. SVD of H, rotation R = V * U^T
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    Eigen::Matrix3f R = V * U.transpose();

    // 5. Reflection check — if det(R) < 0, flip sign of smallest singular vector
    if (R.determinant() < 0) {
        Eigen::Matrix3f V_fixed = V;
        V_fixed.col(2) *= -1;
        R = V_fixed * U.transpose();
    }

    // 6. Translation
    Eigen::Vector3f t = centroidDst - R * centroidSrc;

    return Sophus::SE3f(R, t);
}

#include <random>


RansacResult RansacHornAlignment(
    const std::vector<Eigen::Vector3f>& src,   // map B points, index-aligned with dst
    const std::vector<Eigen::Vector3f>& dst,   // map A points
    float inlierThreshold,             // metres — tune to your map's scale/noise
    int maxIterations,
    int minInliersToAccept)
{
    const int N = src.size();
    assert(N == (int)dst.size());

    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> dist(0, N - 1);

    RansacResult best;

    for (int iter = 0; iter < maxIterations; ++iter) {
        // 1. Pick 3 distinct random correspondences (minimal set for rigid 3D)
        int i0, i1, i2;
        i0 = dist(rng);
        do { i1 = dist(rng); } while (i1 == i0);
        do { i2 = dist(rng); } while (i2 == i0 || i2 == i1);

        std::vector<Eigen::Vector3f> sampleSrc = {src[i0], src[i1], src[i2]};
        std::vector<Eigen::Vector3f> sampleDst = {dst[i0], dst[i1], dst[i2]};

        // Skip degenerate (near-collinear) samples
        Eigen::Vector3f v1 = sampleSrc[1] - sampleSrc[0];
        Eigen::Vector3f v2 = sampleSrc[2] - sampleSrc[0];
        if (v1.cross(v2).norm() < 1e-6f) continue;

        Sophus::SE3f T = HornAlignment(sampleSrc, sampleDst);

        // 2. Count inliers over the FULL correspondence set
        std::vector<int> inliers;
        inliers.reserve(N);
        for (int i = 0; i < N; ++i) {
            Eigen::Vector3f predicted = T * src[i];
            float err = (predicted - dst[i]).norm();
            if (err < inlierThreshold)
                inliers.push_back(i);
        }

        if ((int)inliers.size() > best.numInliers) {
            best.numInliers = inliers.size();
            best.inlierIndices = inliers;
            best.T = T; // will be refined below
        }
    }

    // 3. Refine: re-run Horn's method using ALL inliers from the best model
    if (best.numInliers >= minInliersToAccept) {
        std::vector<Eigen::Vector3f> inlierSrc, inlierDst;
        for (int idx : best.inlierIndices) {
            inlierSrc.push_back(src[idx]);
            inlierDst.push_back(dst[idx]);
        }
        best.T = HornAlignment(inlierSrc, inlierDst);
    }

    return best;
}

// Accepts numpy 3xN (or Nx3, see note below) directly via automatic Eigen conversion
RansacResult RansacHornAlignmentPy(const Eigen::MatrixX3f& src,  // 3xN
                                    const Eigen::MatrixX3f& dst,  // 3xN
                                    float inlierThreshold,
                                    int maxIterations,
                                    int minInliersToAccept)
{
    assert(src.rows() == dst.rows());
    const int N = src.rows();

    // Convert to your existing vector<Vector3f> interface
    std::vector<Eigen::Vector3f> srcVec(N), dstVec(N);
    for (int i = 0; i < N; ++i) {
        srcVec[i] = src.row(i).transpose();
        dstVec[i] = dst.row(i).transpose();
    }

    return RansacHornAlignment(srcVec, dstVec, inlierThreshold, maxIterations, minInliersToAccept);
}
