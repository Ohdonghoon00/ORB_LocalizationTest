
#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include <string>

void MapCompression(ORB_SLAM2::Map* mpMap, double CompressionRatio)
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
            index++;
        }
    }
    
    std::cout << " Finish Map Compression" << std::endl;
    long unsigned int PointCloudNum_ = mpMap->MapPointsInMap();
    std::cout << "After Compression Point Num : " << PointCloudNum_ << std::endl;
}

int main(int argc, char** argv)
{
    ORB_SLAM2::Map* dbMap;
    ORB_SLAM2::KeyFrameDatabase* dbKeyframeDatabase;

    dbMap = new ORB_SLAM2::Map();
    dbKeyframeDatabase = new ORB_SLAM2::KeyFrameDatabase();

    
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
    for(size_t i = 0; i < kfdb.size(); i++){
        std::set<ORB_SLAM2::MapPoint*> kfMpts = kfdb[i]->GetMapPoints();
        std::vector<ORB_SLAM2::MapPoint*> kfMpts_vec = kfdb[i]->GetMapPointMatches();
        std::cout << i << " " << kfMpts.size() << "  " << kfMpts_vec.size() << std::endl;
    }
    
    // Compression
    MapCompression(dbMap, 0.1);

    for(size_t i = 0; i < kfdb.size(); i++){
        std::set<ORB_SLAM2::MapPoint*> kfMpts = kfdb[i]->GetMapPoints();
        std::vector<ORB_SLAM2::MapPoint*> kfMpts_vec = kfdb[i]->GetMapPointMatches();
        std::cout << i << " " << kfMpts.size() << "  " << kfMpts_vec.size() << std::endl;
    }

    // Save Map data
    std::string outpath = "MH01_Compression_10.bin";
    std::ofstream out(outpath, std::ios_base::binary);
    if (!out)
    {
        std::cout << "Cannot Write to Database File: " << std::endl;
        exit(-1);
    }
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << dbMap;
    oa << dbKeyframeDatabase;
    out.close();

    return 0;
}
