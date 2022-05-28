#pragma once


#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"

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

    double originalKeyframeNum;
    long unsigned int totalKeyframeNum;
    long unsigned int totalMapPointNum;    
    
    // Keyframe Removal Member
    Eigen::MatrixXd similarityMatrix;
    Eigen::VectorXd invScoreVec;
    inline static Eigen::VectorXd KeyframeInvWeight;
    
    std::vector<double> kfObsNums;
    std::vector<int> kfMpNums;
    inline static std::vector<double> kfObsNumsRatio;
    inline static std::vector<double> kfMpNumsRatio;
    std::vector<int> kfObsRank;
    std::vector<int> kfMpNumRank;
    double TotalObservation;
    int TotalMapPoints;

    // Score Parameter
    double obsRatio = 0.5;
    double mpRatio = 1.0 - obsRatio;
    double relPoseErr[2];
    int neighborKeyframeNumThres = 3; 
    double neighborKeyframeTranslationThres = 0.5;
    double neighborKeyframeRotationThres = 30;
    
    // Compresssion
    void LandmarkSparsification(double CompressionRatio);
    int removalKeyframe1();
    int removalKeyframe2(double CompressionKfRatio);
    
    // preparing
    void initializing();
    void preparing();

    // Keyframe Score and Mp Num
    void getKeyframeScoreVector();
    double getTotalObservation();
    int getTotalMapPoints();
    double getObservation(ORB_SLAM2::KeyFrame* kf);
    int getObservation(ORB_SLAM2::MapPoint* mp);

    // Keyframe Similarity
    void getKeyframeSimilarityMatrix();
    int getCovisibilityMpNum(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);
    
    void getRelativePose(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);
    int getNeighborKeyframeNum(ORB_SLAM2::KeyFrame* kf);
    
    // print
    void printKeyframeInfo(const std::string &file);

    void iterateKeyframeRemoval();

///////////////////////////////////////////////////////////////////////   


    int getMemory(ORB_SLAM2::MapPoint* mp);
    
    int cvMatSize(cv::Mat a);
    void minMaxNormalize(Eigen::VectorXd *vec);
    void minMaxNormalize(std::vector<double> *vec);
    void RMSError(Vector6d EsPose, Vector6d gtPose, double *err);

    static bool weightComp(int i, int j){return KeyframeInvWeight[i] > KeyframeInvWeight[j];}
    static bool obsScoreComp(int i, int j){return kfObsNumsRatio[i] > kfObsNumsRatio[j];}
    static bool mpNumScoreComp(int i, int j){return kfMpNumsRatio[i] > kfMpNumsRatio[j];}
};