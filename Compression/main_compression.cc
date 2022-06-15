
#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "Compression.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include <string>





    

int main(int argc, char** argv)
{
    // ORB_SLAM2::Map* dbMap;
    // ORB_SLAM2::KeyFrameDatabase* dbKeyframeDatabase;

    // dbMap = new ORB_SLAM2::Map();
    // dbKeyframeDatabase = new ORB_SLAM2::KeyFrameDatabase();

    Compression compression;


    std::ofstream f;
    f.open("abc.txt");
    
    // std::ofstream obsfile;
    // obsfile.open("KeyframeInfo.txt");

    // Load Map data
    std::string dataPath = argv[1];
    std::ifstream in(dataPath, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapdata for Compression " << std::endl;
        return EXIT_FAILURE;
    }
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> compression.Map;
    ia >> compression.dbKeyframe;    
    in.close();
    
    std::cout << " Keyframe Num : " << compression.Map->KeyFramesInMap() << " Landmark Num : " << compression.Map->MapPointsInMap() << std::endl;
    std::cout << " Compression Preparing ... " << std::endl;

    double compRatio = std::stod(argv[2]);
    compression.setInitial(compRatio);
    std::cout << "id threshold : " << compression.neighborKeyframeIdThres << std::endl;
    
    std::cout << " remove Keyframe ... " << std::endl;
    // compression.removalKeyframe1();
    int totalRemovedMemory = compression.removalKeyframe2();
    // compression.LandmarkSparsification();
    std::cout << " Finish Compression !! " << std::endl;
    
    std::cout << " print Compressed Keyframe Info ... " << std::endl;
    compression.iterateKeyframeRemoval();
    compression.printKeyframeInfo("CompressionKeyframeInfo.txt");
    
    // memory for removing keyframe
    int totalRemovedLandmark = 36829 - compression.Map->MapPointsInMap();
    totalRemovedMemory += totalRemovedLandmark * 736;
    std::cout <<  " Total memory of removed mapPoints : " << totalRemovedMemory << std::endl;
    


    std::cout << " Keyframe Num : " << compression.Map->KeyFramesInMap() << " Landmark Num : " << compression.Map->MapPointsInMap() << std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////
    // compression.getAllKeyframeReprojErr();
    // compression.printKeyframeInfo("CompressionKeyframeInfo.txt");
    // std::vector<ORB_SLAM2::KeyFrame*> kfdb_ = compression.Map->GetAllKeyFrames();
    // std::sort(kfdb_.begin(),kfdb_.end(),ORB_SLAM2::KeyFrame::lId);
    // int totalmp = 0;
    // int totalObsdd = 0;
    // std::vector<ORB_SLAM2::MapPoint*> mpdb_ = compression.Map->GetAllMapPoints();
    // for(size_t i = 0; i < kfdb_.size(); i++){
    //     totalmp += compression.kfMpNums[i];
    //     // std::cout << compression.kfMpNums[i] << " ";
    //     // std::set<ORB_SLAM2::MapPoint*> kfMpts = kfdb_[i]->GetMapPoints();
    //     // std::vector<ORB_SLAM2::MapPoint*> kfMpts_vec = kfdb_[i]->GetMapPointMatches();
    //     // std::cout << kfdb_.size() << " " << i << " " << kfdb_[i]->mnId << " " << kfMpts.size() << "  " << kfMpts_vec.size() << std::endl;
    //     // f << kfdb_[i]->mnId << " " << kfMpts.size() << std::endl;
    //     // std::cout << sizeof(kfMpts[i]) << std::endl;
    //     compression.getreprojectionErr(kfdb_[i]);
    //     std::cout << std::endl;
    // }
    //     std::cout << compression.getObservation(kfdb_[336]) << " ";
    // std::cout << std::endl;
    // cv::imshow("335left", kfdb_[335]->LeftImg);
    // cv::imshow("336left", kfdb_[336]->LeftImg);
    // cv::waitKey();
    // totalmp += compression.getKeyframeMap(kfdb_[0]).size();
    // for(size_t i = 0; i < mpdb_.size(); i++){
    //     totalObsdd += mpdb_[i]->mObservations.size();
    //     // std::cout << mpdb_[i]->mObservations.size() << " ";
    // }
    // std::cout << totalmp << " " << totalObsdd << std::endl;
////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // std::cout << sizeof(ORB_SLAM2::MapPoint) << std::endl;
    // std::cout << sizeof(ORB_SLAM2::Map) << std::endl;
    // std::cout << sizeof(ORB_SLAM2::KeyFrame) << std::endl;
    // std::cout << sizeof(std::vector<ORB_SLAM2::MapPoint*>) << std::endl;
    // std::cout << sizeof(std::set<ORB_SLAM2::KeyFrame*>) << std::endl;
    // std::cout << sizeof(std::set<ORB_SLAM2::MapPoint*>) << std::endl;
    // std::cout << kfdb_[10]->mDescriptors.elemSize() << std::endl;
    // std::cout << kfdb_[10]->mDescriptors.total() << std::endl;
    // // std::cout << kfdb_[115]->mvKeys.size() << std::endl;
    // // std::cout << kfdb_[115]->mvKeys.capacity() << std::endl;
    // cv::Mat ab(500, 200, CV_8UC1);
    // std::cout << sizeof(ab) << std::endl;
    // std::cout << sizeof(long int) << std::endl;
    // std::cout << sizeof(bool) << std::endl;
    // std::cout << sizeof(char) << std::endl;
    // std::cout << sizeof(size_t) << std::endl;
    // std::cout << sizeof(cv::Point2f) << std::endl;
    // std::cout << sizeof(cv::KeyPoint) << std::endl;
    // std::vector<ORB_SLAM2::MapPoint*> kfMptdkjnfa= kfdb_[10]->GetMapPointMatches();
    // std::cout << kfMptdkjnfa.size() << std::endl;
    // std::cout << kfMptdkjnfa[500]->mObservations.size() << std::endl;

    // std::cout << "agdg : " << sizeof(kfMptdkjnfa[500]->mWorldPos) << std::endl;
    // std::cout << cvMatSize(kfMptdkjnfa[500]->mPosGBA) << std::endl;
    // std::cout << cvMatSize(kfMptdkjnfa[500]->mWorldPos) << std::endl;
    // std::cout << cvMatSize(kfMptdkjnfa[500]->mNormalVector) << std::endl;
    // std::cout << cvMatSize(kfMptdkjnfa[500]->mDescriptor) << std::endl;
    // std::cout << sizeof(kfMptdkjnfa[500]->mMutexPos) << std::endl;
    // std::cout << sizeof(kfMptdkjnfa[500]->mMutexFeatures) << std::endl;

    // std::cout << kfdb_[10]->mvKeys.size() * sizeof(cv::KeyPoint) << std::endl;
    // std::cout << kfdb_[10]->mvKeysUn.size() * sizeof(cv::KeyPoint) << std::endl;
    // std::cout << kfdb_[10]->mvuRight.size() * sizeof(float) << std::endl;
    // std::cout << kfdb_[10]->mvDepth.size() * sizeof(float) << std::endl;
    // std::cout << cvMatSize(kfdb_[10]->mDescriptors) << std::endl;

    // std::cout << sizeof(kfdb_[10]->mBowVec) << std::endl;
    // std::cout << sizeof(kfdb_[10]->mBowVec) << std::endl;

    // std::cout << cvMatSize(kfdb_[10]->mTcp) << std::endl;

    // std::cout << kfdb_[10]->mvScaleFactors.size() * sizeof(float) << std::endl;
    // std::cout << kfdb_[10]->mvLevelSigma2.size() * sizeof(float) << std::endl;
    // std::cout << kfdb_[10]->mvInvLevelSigma2.size() * sizeof(float) << std::endl;
    // std::cout << cvMatSize(kfdb_[10]->mK) << std::endl;
    // std::cout << cvMatSize(kfdb_[10]->Tcw) << std::endl;
    // std::cout << cvMatSize(kfdb_[10]->Twc) << std::endl;
    // std::cout << cvMatSize(kfdb_[10]->Ow) << std::endl;
    // std::cout << cvMatSize(kfdb_[10]->Cw) << std::endl;


    // std::cout << kfdb_[10]->mvpMapPoints.size() * sizeof(ORB_SLAM2::MapPoint*) << std::endl;

    // std::cout << kfdb_[10]->mGrid.size() * sizeof(float) << std::endl;
    // int totalsize = 0;
    // for(size_t i = 0; i < kfdb_[10]->mGrid.size(); i++){
    //     int totalsize1 = 0;
    //     for(size_t j = 0; j < kfdb_[10]->mGrid[i].size(); j++){
    //         int cnt = kfdb_[10]->mGrid[i][j].size() * sizeof(size_t);
    //         totalsize1 += cnt;
    //     }
    //     totalsize += totalsize1;
    // }
    // std::cout << "ddd  : " << totalsize << std::endl;
    // std::cout << kfdb_[10]->mConnectedKeyFrameWeights.size() * (sizeof(ORB_SLAM2::KeyFrame*) + sizeof(int)) << std::endl;
    // std::cout << kfdb_[10]->mvpOrderedConnectedKeyFrames.size() * sizeof(ORB_SLAM2::KeyFrame*) << std::endl;
    // std::cout << kfdb_[10]->mvOrderedWeights.size() * sizeof(int) << std::endl;
    // std::cout << sizeof(kfdb_[10]->mMutexPose) << std::endl;
    // std::cout << sizeof(kfdb_[10]->mMutexConnections) << std::endl;
    // std::cout << sizeof(kfdb_[10]->mMutexFeatures) << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////

    // Save Map data
    std::cout << "Save Map ... " << std::endl;
    std::string outpath = "MH01_KFcompression_test.bin";
    std::ofstream out(outpath, std::ios_base::binary);
    if (!out)
    {
        std::cout << "Cannot Write to Database File: " << std::endl;
        exit(-1);
    }
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    std::cout << " ...done" << std::endl;
    oa << compression.Map;
    oa << compression.dbKeyframe;
    std::cout << " ...done" << std::endl;
    std::cout << " ...done" << std::endl;
    out.close();
    f.close();
    return 0;
}
