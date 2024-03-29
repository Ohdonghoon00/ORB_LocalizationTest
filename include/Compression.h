#pragma once


#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include <random>

class ORB_SLAM2::Map;
// class ORB_SLAM2::MapPoint;
// class ORB_SLAM2::KeyFrame;
class ORB_SLAM2::KeyFrameDatabase;

class Compression
{
public:    
    
    Compression()
    {}

    double kfCompressedRatio;
    int remainLandmark;
    double removedMemory;

    ORB_SLAM2::Map* Map;
    // ORB_SLAM2::MapPoint* MapPoint;
    // ORB_SLAM2::KeyFrame* Keyframe;
    ORB_SLAM2::KeyFrameDatabase* dbKeyframe;

    int originalKeyframeNum, originalMapPointNum;
    long unsigned int totalKeyframeNum;
    long unsigned int totalMapPointNum;    
    
    // Keyframe Removal Member
    Eigen::MatrixXd similarityMatrix;
    Eigen::VectorXd invScoreVec;
    Eigen::VectorXd obs1NumRatio;
    Eigen::VectorXf reprojectionErrAvg;
    Eigen::VectorXf reprojectionErrInlier;
    inline static Eigen::VectorXd KeyframeInvWeight;
    
    std::vector<double> kfObsNums;
    std::vector<int> kfMpNums;
    inline static std::vector<double> kfObsNumsRatio;
    inline static std::vector<double> kfMpNumsRatio;
    std::vector<int> kfObsRank;
    std::vector<int> kfMpNumRank;
    std::vector<int> kfNewIds;
    int maxNewid;
    double TotalObservation;
    int TotalMapPoints;
    

    // Score Parameter
    double obsRatio = 1.0;
    double mpRatio = 1.0 - obsRatio;
    double relPoseErr[2];
    int neighborKeyframeNumThres = 1; 
    int neighborKeyframeIdThres; 
    double neighborKeyframeTranslationThres = 0.6;
    double neighborKeyframeRotationThres = 20;
    double obs1RatioThres = 0.5;
    double keypointDistThres = 40.0;
    
    // reprojection Err
    float inlierRatioInKeyframe;

    // Cube info
    Eigen::Vector3i minPoint;
    Eigen::Vector3i maxPoint;
    int smallCubeXNum, smallCubeYNum, smallCubeZNum, totalSmallCubeNum;
    std::vector<int> cubeIds;
    std::set<int> cubeIdsSet;
    Eigen::MatrixXi visibilityMatrix;
    std::vector<int> originalCubeVector;

    // Landmark Score
    std::vector<int> obsNum;
    std::vector<float> maxTrackDist;
    std::vector<float> maxAngle;
    std::vector<float> avgReprojectionErr;
    std::vector<float> landmarkScore;
    Eigen::MatrixXd keypointDistanceMatrix;
    std::map<std::tuple<int, int>, double> keypointDistanceQ;
    
    // Keyframe Score
    std::vector<float> keyframeScore;

    // remove randomly
    std::vector<std::vector<int>> storage;
    std::vector<double> compressionRatios = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.15, 0.10, 0.08, 0.06, 0.05, 0.04, 0.03};

    // main Compresssion
    void LandmarkSparsification();
    void LandmarkSparsification2(const double a, const double b,const double c,const double d);
    void LandmarkSparsificationIQP(const double a, const double b,const double c,const double d);
    int removalKeyframe1();
    int removalKeyframe2();
    int removalKeyframe3();
    int removalKeyframe4();
    int removalKeyframeSimilarity(const double a, const double b,const double c,const double d);
    int removalKeyframeILP(const double a, const double b,const double c,const double d);
    int removalKeyframeIQP(const double a, const double b,const double c,const double d);
    
    // random
    void selectRandomNum();
    int removeRandomKeyframe(int storageIdx, std::vector<ORB_SLAM2::KeyFrame *> kfdb);
    int randomNum(int low, int high, unsigned int seed);

    

    
    // preparing
    void setInitial(double kfCompressedRatio_);
    void initializing();
    void preparing();

    // Keyframe Score and Mp Num
    void getKeyframeScoreVector();
    double getTotalObservation();
    int getTotalMapPoints();
    double getObservation(ORB_SLAM2::KeyFrame* kf);
    int getObservation(ORB_SLAM2::MapPoint* mp);

    // Reprojection Err
    void getAllKeyframeReprojErr();
    float getreprojectionErrAvg(ORB_SLAM2::KeyFrame* kf);

    // obs 1
    void getAllObs1Ratio();

    // Keyframe Similarity
    void getKeyframeSimilarityMatrix();
    static int getCovisibilityMpNum(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);
    
    // Constraint
    void getRelativePose(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);
    float getRelativeTranslation(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2);
    float getRelativeRotation(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2, Eigen::Vector3f landmarkPos);

    int getNeighborKeyframeNum(ORB_SLAM2::KeyFrame* kf);
    int getNeighborKeyframeIdDist(int idx);
    
    // print
    void printKeyframeInfo(const std::string &file);


    static std::vector<ORB_SLAM2::MapPoint*> getKeyframeMap(ORB_SLAM2::KeyFrame* kf);
    double getKeyframeObs1Num(ORB_SLAM2::KeyFrame* kf);

    void iterateKeyframeRemoval();

    // calculate map shape
    void estimateCube();
    void getBigCube(    std::vector<ORB_SLAM2::MapPoint*> mpDb,
                        Eigen::Vector3d* minPoint,
                        Eigen::Vector3d* maxPoint);
    void getSmallCube();
    int getCubeId(ORB_SLAM2::MapPoint* mp);
    void getCubeIds();
    void getVisibilityMatrix();
    int getOriginalCubeVector();

    // Calculate Landmark Score
    void getLandmarkScore(const double a, const double b,const double c,const double d);
    void calculateVariousScore();
    float getMaxTrackDistance(ORB_SLAM2::MapPoint* mp);
    float getMaxAngle(ORB_SLAM2::MapPoint* mp);
    float getreprojectionErrAvg(ORB_SLAM2::MapPoint* mp);
        
        // IQP for Landmark  
    std::map<std::tuple<int, int>, double> getKeypointDistanceMatrix();
    std::vector<int> getKeyframeObsId( std::vector<ORB_SLAM2::KeyFrame *> &kfdb,
                                                    ORB_SLAM2::MapPoint* mp);
    std::vector<int> getKeyframeCovisibleMp(std::vector<ORB_SLAM2::KeyFrame *> &kfdb,
                                            std::vector<int> KfIdx1,
                                            ORB_SLAM2::MapPoint* mp2 );

    // Calculate Keyframe Score
    void getKeyframeScore();


///////////////////////////////////////////////////////////////////////   


    int getMemory(ORB_SLAM2::MapPoint* mp);
    int getMemory(ORB_SLAM2::KeyFrame* kf);
    
    int cvMatSize(cv::Mat a);
    void minMaxNormalize(Eigen::VectorXd *vec);
    void minMaxNormalize(Eigen::VectorXf *vec);
    void minMaxNormalizeInv(std::vector<float> *vec);
    void minMaxNormalize(std::vector<double> *vec);
    void minMaxNormalize(std::vector<float> *vec);
    std::vector<float> minMaxNormalize(std::vector<int> *vec);
    void zScoreNormalize(std::vector<int> *vec);
    void RMSError(Vector6d EsPose, Vector6d gtPose, double *err);
    static int getSetIndex(std::set<int> S, int K);

    static bool weightComp(int i, int j){return KeyframeInvWeight[i] > KeyframeInvWeight[j];}
    static bool obsScoreComp(int i, int j){return kfObsNumsRatio[i] > kfObsNumsRatio[j];}
    static bool mpNumScoreComp(int i, int j){return kfMpNumsRatio[i] > kfMpNumsRatio[j];}
};