
#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include <string>

void LandmarkSparsification(ORB_SLAM2::Map* mpMap, double CompressionRatio)
{
    // Map Compression
    std::cout << "Map Compression ... " << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    std::vector<ORB_SLAM2::MapPoint*> mpDB = mpMap->GetAllMapPoints();
    long unsigned int PointCloudNum = mpMap->MapPointsInMap();

    std::cout << "before Compression Point Num : " << PointCloudNum << std::endl;
    std::cout << " Create Variables ... " << std::endl;
    // Create Variables
    std::vector<GRBVar> x = CreateVariablesBinaryVector(PointCloudNum, model);

    std::cout << " Set Objective ... " << std::endl;
    // Set Objective
    Eigen::Matrix<double, Eigen::Dynamic, 1> q = CalculateObservationCountWeight(mpMap);
    SetObjectiveILP(x, q, model);

    std::cout << " Add Constraint ... " << std::endl;
    // Add Constraint
    Eigen::MatrixXd A = CalculateVisibilityMatrix(mpMap);
    AddConstraint(mpMap, model, A, x, CompressionRatio);

    std::cout << std::endl;

    std::cout << " Optimize model ... " << std::endl;
    // Optimize model
    model.optimize();

    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    std::cout << std::endl;

    // Erase Map Point
    size_t index = 0;

    for (size_t i = 0; i < x.size(); i++){

        if (x[i].get(GRB_DoubleAttr_X) == 0){
            mpDB[i]->SetBadFlag();
            // mpMap->EraseMapPoint((mpMap->GetAllMapPoints())[i - index]);
            // mpMap->EraseMapPoint(i - index);
            index++;
        }
    }
    // mpMap->clear();

    std::cout << " Finish Map Compression" << std::endl;
    long unsigned int PointCloudNum_ = mpMap->MapPointsInMap();
    std::cout << "After Compression Point Num : " << PointCloudNum_ << std::endl;
}

void removalKeyframe(ORB_SLAM2::Map* mpMap)
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = mpMap->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    int removeKeyframeCnt = 0;
    int removeIdx = 0;
    for(size_t i = 0; i < kfdb.size(); i++){
        
        if(i == 0) continue;
        if(kfdb[i - 1]->isBad()) removeIdx++;
        else removeIdx = 0;
        
        std::vector<ORB_SLAM2::MapPoint*> kfMapts = kfdb[i]->GetMapPointMatches();
        int covisibilityCnt = 0;
        
        for(size_t j = 0; j < kfMapts.size(); j++){
            
            if(!kfMapts[j]) continue;
            if(!kfMapts[j]->isBad()){
                
                bool covisibilityLandmark = kfMapts[j]->IsInKeyFrame(kfdb[i - 1 - removeIdx]);
                if(covisibilityLandmark) covisibilityCnt++;
            }
        }
        std::cout << covisibilityCnt << std::endl;
        if(covisibilityCnt > 100){
            kfdb[i]->SetBadFlag();
            removeKeyframeCnt++;        
        } 
    }    
}

int main(int argc, char** argv)
{
    ORB_SLAM2::Map* dbMap;
    ORB_SLAM2::KeyFrameDatabase* dbKeyframeDatabase;

    dbMap = new ORB_SLAM2::Map();
    dbKeyframeDatabase = new ORB_SLAM2::KeyFrameDatabase();

    std::ofstream f;
    f.open("abc.txt");
    
    // Load Map data
    std::string dataPath = argv[1];
    std::ifstream in(dataPath, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapdata for Compression " << std::endl;
        return EXIT_FAILURE;
    }
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> dbMap;
    ia >> dbKeyframeDatabase;    
    in.close();

    std::vector<ORB_SLAM2::KeyFrame*> kfdb = dbMap->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);
    for(size_t i = 0; i < kfdb.size(); i++){
        std::set<ORB_SLAM2::MapPoint*> kfMpts = kfdb[i]->GetMapPoints();
        std::vector<ORB_SLAM2::MapPoint*> kfMpts_vec = kfdb[i]->GetMapPointMatches();
        std::cout << kfdb.size() << " " << i << " " << kfdb[i]->mnId << " " << kfMpts.size() << "  " << kfMpts_vec.size() << std::endl;
    }
    
    // Compression
    removalKeyframe(dbMap);
    // LandmarkSparsification(dbMap, 0.5);

    std::vector<ORB_SLAM2::KeyFrame*> kfdb_ = dbMap->GetAllKeyFrames();
    std::sort(kfdb_.begin(),kfdb_.end(),ORB_SLAM2::KeyFrame::lId);
    for(size_t i = 0; i < kfdb_.size(); i++){
        std::set<ORB_SLAM2::MapPoint*> kfMpts = kfdb_[i]->GetMapPoints();
        std::vector<ORB_SLAM2::MapPoint*> kfMpts_vec = kfdb_[i]->GetMapPointMatches();
        std::cout << kfdb_.size() << " " << i << " " << kfdb_[i]->mnId << " " << kfMpts.size() << "  " << kfMpts_vec.size() << std::endl;
        f << kfdb_[i]->mnId << " " << kfMpts.size() << std::endl;
        // std::cout << sizeof(kfMpts[i]) << std::endl;
    }

    std::cout << sizeof(ORB_SLAM2::MapPoint) << std::endl;
    std::cout << sizeof(ORB_SLAM2::Map) << std::endl;
    std::cout << sizeof(ORB_SLAM2::KeyFrame) << std::endl;
    // std::cout << kfdb_[10]->mDescriptors.elemSize() << std::endl;
    // std::cout << kfdb_[10]->mDescriptors.total() << std::endl;
    // std::cout << kfdb_[115]->mvKeys.size() << std::endl;
    // std::cout << kfdb_[115]->mvKeys.capacity() << std::endl;
    std::cout << sizeof(long unsigned int) << std::endl;
    std::cout << sizeof(unsigned int) << std::endl;
    std::cout << sizeof(cv::Point2f) << std::endl;
    std::cout << sizeof(cv::KeyPoint) << std::endl;
    std::cout << sizeof(ORB_SLAM2::KeyFrameDatabase) << std::endl;

    // Save Map data
    std::cout << "Save Map ... " << std::endl;
    std::string outpath = "MH01_kf_Compression_10_test.bin";
    std::ofstream out(outpath, std::ios_base::binary);
    if (!out)
    {
        std::cout << "Cannot Write to Database File: " << std::endl;
        exit(-1);
    }
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    std::cout << " ...done" << std::endl;
    oa << dbMap;
    oa << dbKeyframeDatabase;
    std::cout << " ...done" << std::endl;
    std::cout << " ...done" << std::endl;
    out.close();
    f.close();
    return 0;
}
