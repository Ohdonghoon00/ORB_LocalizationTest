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
    Compression DB1, DB2, oriDB;

    // save file
    std::ofstream adf;
    adf.open("dajklfsd.txt");
    // Load Map1 data
    std::string dataPath = argv[1];
    std::ifstream in(dataPath, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapdata1 for Compression " << std::endl;
        return EXIT_FAILURE;
    }
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> DB1.Map;
    ia >> DB1.dbKeyframe;    
    in.close();
    
    // Load Map2 data
    std::string dataPath2 = argv[2];
    std::ifstream in_(dataPath2, std::ios_base::binary);
    if (!in_)
    {
        cerr << "Cannot Open Mapdata2 for Compression " << std::endl;
        return EXIT_FAILURE;
    }
    boost::archive::binary_iarchive ia_(in_, boost::archive::no_header);
    ia_ >> DB2.Map;
    ia_ >> DB2.dbKeyframe;    
    in_.close();
    
    // Load Map1 data
    std::string dataPath3 = argv[3];
    std::ifstream in__(dataPath3, std::ios_base::binary);
    if (!in__)
    {
        cerr << "Cannot Open Mapdata1 for Compression " << std::endl;
        return EXIT_FAILURE;
    }
    boost::archive::binary_iarchive ia__(in__, boost::archive::no_header);
    ia__ >> oriDB.Map;
    ia__ >> oriDB.dbKeyframe;    
    in__.close();

    ///////////////
    std::vector<ORB_SLAM2::KeyFrame *> kfdb1 = DB1.Map->GetAllKeyFrames();
    std::sort(kfdb1.begin(), kfdb1.end(), ORB_SLAM2::KeyFrame::lId);

    std::vector<ORB_SLAM2::KeyFrame *> kfdb2 = DB2.Map->GetAllKeyFrames();
    std::sort(kfdb2.begin(), kfdb2.end(), ORB_SLAM2::KeyFrame::lId);   

    std::vector<ORB_SLAM2::KeyFrame *> kfdb3 = oriDB.Map->GetAllKeyFrames();
    std::sort(kfdb3.begin(), kfdb3.end(), ORB_SLAM2::KeyFrame::lId);

    std::cout << " Keyframe1 Num : " << DB1.Map->KeyFramesInMap() << " Landmark1 Num : " << DB1.Map->MapPointsInMap() << std::endl;
    std::cout << " Keyframe2 Num : " << DB2.Map->KeyFramesInMap() << " Landmark2 Num : " << DB2.Map->MapPointsInMap() << std::endl;
    std::cout << " Original Keyframe Num : " << oriDB.Map->KeyFramesInMap() << " Original Landmark Num : " << oriDB.Map->MapPointsInMap() << std::endl;
    ////////////
    
    ///////////////
    std::vector<ORB_SLAM2::MapPoint *> mpdb1 = DB1.Map->GetAllMapPoints();

    std::vector<ORB_SLAM2::MapPoint *> mpdb2 = DB2.Map->GetAllMapPoints();

    std::vector<ORB_SLAM2::MapPoint *> mpdb3 = oriDB.Map->GetAllMapPoints();
    ////////////////
    std::cout << mpdb1.size() << " " << mpdb2.size() << " " << mpdb3.size() << std::endl;

    for(size_t i = 0; i < mpdb3.size(); i++){
      adf << oriDB.getObservation(mpdb3[i]) << std::endl;
    }

    std::vector<int> kfId1, kfId2;
    // for(size_t i = 0; i < kfdb2.size(); i++){
    //     kfId2.emplace_back(kfdb2[i]->mnId);
    //     // std::cout << "i : " << i << "  Id : " << kfdb2[i]->mnId << "  size : " << DB1.getKeyframeMap(kfdb2[i]).size() << std::endl;
    //     std::cout << DB1.getKeyframeMap(kfdb1[i]).size() << " " << DB2.getKeyframeMap(kfdb2[i]).size() << std::endl;
    // }
    // int kfN = std::stoi(argv[3]);
    int kfN = 0;
    // for(size_t i = 0; i < kfdb1.size(); i++){

    //   kfN = i;
      
    //   cv::Mat img1 = kfdb1[kfN]->LeftImg.clone();
    //   cv::Mat img2 = kfdb2[kfN]->LeftImg.clone();
    //   cv::Mat oriImg = kfdb3[kfN]->LeftImg.clone();


    //   std::vector<ORB_SLAM2::MapPoint *> kfMapPoints1 = Compression::getKeyframeMap(kfdb1[kfN]);
    //   std::vector<ORB_SLAM2::MapPoint *> kfMapPoints2 = Compression::getKeyframeMap(kfdb2[kfN]);
    //   std::vector<ORB_SLAM2::MapPoint *> kfMapPoints3 = Compression::getKeyframeMap(kfdb3[kfN]);
      
    //   std::vector<cv::KeyPoint> adf1, adf2, oriK;
    //   // std::cout << kfMapPoints1.size() << "    " << kfMapPoints2.size() <<  "    " << kfMapPoints3.size() << std::endl;
      
      
    //   for(size_t j = 0; j < kfMapPoints1.size(); j++){
    //       int idx = (kfMapPoints1[j])->GetIndexInKeyFrame(kfdb1[kfN]);
    //       adf1.push_back(kfdb1[kfN]->mvKeys[idx]);
    //   }

    //   for(size_t j = 0; j < kfMapPoints2.size(); j++){
    //       int idx = (kfMapPoints2[j])->GetIndexInKeyFrame(kfdb2[kfN]);
    //       adf2.push_back(kfdb2[kfN]->mvKeys[idx]);
    //   }

    //   for(size_t j = 0; j < kfMapPoints3.size(); j++){
    //       int idx = (kfMapPoints3[j])->GetIndexInKeyFrame(kfdb3[kfN]);
    //       oriK.push_back(kfdb3[kfN]->mvKeys[idx]);
    //   }

    //   cv::Mat outImg1, outImg2, outOriImg;
    //   cv::drawKeypoints(img1, adf1, outImg1);
    //   cv::drawKeypoints(img2, adf2, outImg2);
    //   cv::drawKeypoints(oriImg, oriK, outOriImg);
    //   cv::imshow("ILP", outImg1);
    //   cv::imshow("IQP", outImg2);
    //   cv::imshow("original", outOriImg);
    //   cv::waitKey();
    // }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // for (int i = 0; i < 20; i++){
      
    //   std::vector<cv::KeyPoint> asdf;
    //   std::vector<ORB_SLAM2::MapPoint *> kfMapPoints3 = Compression::getKeyframeMap(kfdb3[0]);
    //   for (size_t j = 0; j < kfMapPoints3.size(); j++){
        
    //     int cnt = oriDB.getObservation(kfMapPoints3[j]);
    //     if (cnt > i){
    //       int idx = (kfMapPoints3[j])->GetIndexInKeyFrame(kfdb3[0]);
    //       asdf.push_back(kfdb3[0]->mvKeys[idx]);
    //     }
    //   }
    //   cv::Mat inpptImg = kfdb3[0]->LeftImg.clone();
    //   cv::Mat outimmg;
    //   cv::drawKeypoints(inpptImg, asdf, outimmg);
    //   cv::imshow("obsNum", outimmg);
    //   cv::waitKey();
    // }
    
    // for(size_t i = 0; i < kfdb2.size(); i++){
    //     kfId2.emplace_back(kfdb2[i]->mnId);
    //     // std::cout << kfdb2[i]->mnId << "  " << DB.getKeyframeMap(kfdb2[i]).size() << std::endl;
    // }
    // std::cout << "db 1 Keyframe Num is  : " << kfId1.size() << "   " << "db 2  Keyframe Num is  : " << kfId2.size() << std::endl;
    // int kfNum = kfId2.size();
    // int eraseIdx = 0;
    // for(int i = 0; i < kfNum; i++){
    //     int idd = kfId2[i];
    //     auto iter = std::find(kfId1.begin(), kfId2.end(), idd);
    //     if(iter != kfId1.end()){
    //         kfId2.erase(kfId2.begin() + i - eraseIdx);
    //         eraseIdx++;
    //     }
    // }
    // std::cout << "db2 remained Keyframe size : " << kfId2.size() << " erase index  : " << eraseIdx  << std::endl;
    return 0;
}