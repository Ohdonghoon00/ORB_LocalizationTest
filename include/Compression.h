#pragma once


#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"

class ORB_SLAM2::Map;
// class ORB_SLAM2::MapPoint;
// class ORB_SLAM2::KeyFrame;
class ORB_SLAM2::KeyFrameDatabase;

class Compression
{
public:    
    
    Compression()
    {}

    

    ORB_SLAM2::Map* Map;
    // ORB_SLAM2::MapPoint* MapPoint;
    // ORB_SLAM2::KeyFrame* Keyframe;
    ORB_SLAM2::KeyFrameDatabase* dbKeyframe;

    long unsigned int totalKeyframeNum;
    long unsigned int totalMapPointNum;    
    
    // Keyframe Removal Member
    Eigen::MatrixXd similarityMatrix;
    Eigen::VectorXd invScoreVec;
    inline static Eigen::VectorXd KeyframeInvWeight;
    
    std::vector<int> kfObsNums;
    std::vector<int> kfMpNums;

    // Score Parameter
    double obsRatio = 0.5;
    double mpRatio = 1.0 - obsRatio;

    
    // Compresssion
    void LandmarkSparsification(double CompressionRatio);
    int removalKeyframe1();
    int removalKeyframe2(double CompressionKfRatio);
    
    // preparing
    void initializing();
    void preparing();

    // Keyframe Score and Mp Num
    void getKeyframeScoreVector();
    int getTotalObservation();
    int getTotalMapPoints();
    int getObservation(ORB_SLAM2::KeyFrame* kf);
    int getObservation(ORB_SLAM2::MapPoint* mp);

    // Keyframe Similarity
    void getKeyframeSimilarityMatrix();
    int getCovisibilityMpNum(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);
    
    
    
///////////////////////////////////////////////////////////////////////   

    int getMemory(ORB_SLAM2::MapPoint* mp);
    
    int cvMatSize(cv::Mat a);

    static bool scoreComp(int i, int j){return KeyframeInvWeight[i] > KeyframeInvWeight[j];}
};