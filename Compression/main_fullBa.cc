#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "Compression.h"
#include "BA.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include <string>

int main(int argc, char** argv)
{
    Compression compression;

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

    // load keyframe and landmark
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = compression.Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    // full ba
    ceres::Problem global_BA; 
    for(size_t j = 0; j < kfdb.size(); j++){
        std::vector<ORB_SLAM2::MapPoint*> mapKf = compression.getKeyframeMap(kfdb[j]);
        cv::Mat proj = kfdb[j]->GetPose();
        Vector6d projVec6 = ORB_SLAM2::Converter::toProjvec6(proj);
            
        cv::Vec6d CamPose(projVec6[0], projVec6[1], projVec6[2], projVec6[3], projVec6[4], projVec6[5]);
            
        for ( size_t i = 0; i < mapKf.size(); i++){
                
            int keypointIdx = (mapKf[i])->GetIndexInKeyFrame(kfdb[j]);
            cv::Point2f keyPoint = kfdb[j]->mvKeys[keypointIdx].pt;
            // std::cout << kfdb[j]->mvuRight[keypointIdx] << "  " << mapKf[i]->nObs << " " << mapKf[i]->GetObservations().size() << std::endl;
            ceres::CostFunction* map_only_cost_func = map_point_only_ReprojectionError::create(keyPoint, CamPose, (double)kfdb[j]->fx, cv::Point2d(kfdb[j]->cx, kfdb[j]->cy));

            mapKf[i]->savePoint3d(); 
                
            double* X = (double*)(&mapKf[i]->mWPos);
            global_BA.AddResidualBlock(map_only_cost_func, NULL, X); 
            
        } 
            
    }
        
        std::vector<ORB_SLAM2::MapPoint*> aaa = compression.Map->GetAllMapPoints();
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.num_threads = 8;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        std::cout << " Start optimize map point " << std::endl;
        std::cout << aaa[100]->GetWorldPos().t() << std::endl;
        std::cout << aaa[100]->mWPos << std::endl;
        ceres::Solve(options, &global_BA, &summary);
        std::cout << summary.BriefReport() << std::endl;                
        for(size_t i = 0; i < aaa.size(); i++){
            aaa[i]->saveOptimizePos();
        }
        std::cout << " End optimize map point " << std::endl;
        std::cout << aaa[100]->GetWorldPos().t() << std::endl;
        std::cout << aaa[100]->mWPos << std::endl;    




    // Save Map data
    std::cout << "Save Map ... " << std::endl;
    std::string outpath = argv[2];
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

    return 0;
}





