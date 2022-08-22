#include "gurobi_helper.h"
#include "System.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "Compression.h"

#include<iostream>
#include<algorithm>
#include<fstream>
#include <string>
// #include <cmath>







int main(int argc, char** argv)
{
    Compression compression;

    if(argc != 2){
        cerr << "not proper arguments!!";
        exit(1);
    }
    
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
    
    int oriMapPoints = (int)compression.Map->GetAllMapPoints().size();
    // double a[15] = {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.15, 0.10, 0.08, 0.06, 0.05, 0.04, 0.03};
    // std::vector<double> compressionRatios(15);
    

    // for(size_t i = 0; i < compressionRatios.size(); i++) {
    //     compressionRatios[i] = a[i];
    //     // std::cout << compressionRatios[i] << " ";
    // }
    double totalRemovedMemorys = 0;
    int lastRemovedLandmark = 0;
    compression.selectRandomNum();
    std::cout << "abc" << std::endl;

    // for debug
    for(size_t i = 0; i < compression.storage.size(); i++){
        for(size_t j = 0; j < compression.storage[i].size(); j++) std::cout << compression.storage[i][j] <<" ";
        std::cout << std::endl; 
    }
    
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = compression.Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    
    for(size_t i = 0; i < compression.storage.size(); i++){
        
        
        int removememory_ = compression.removeRandomKeyframe(i, kfdb);
        // int removememory_ = 0;
        int totalRemovedLandmark = oriMapPoints - compression.Map->MapPointsInMap() - lastRemovedLandmark;
        lastRemovedLandmark += totalRemovedLandmark;
        removememory_ += totalRemovedLandmark * 736;
        double removememory = (double)removememory_ * 1e-6;

        totalRemovedMemorys += removememory;


        // // Save Result
        // std::ofstream f;
        
        // string tmp = string(argv[1]);
        // istringstream tmpStr(tmp);
        // string trash;
        // vector<string> acc;
        // while(getline(tmpStr, trash, '/')) acc.push_back(trash);
        // string finalStr = acc[4];
        // std::cout << finalStr << std::endl;
        // f.open("result/220802/randomCompressionResult/"+finalStr+"RandomKeyframe_Compression_Result_"+to_string(compression.compressionRatios[i])+"_.txt", ios::out);
        // f << 1.0 - compression.compressionRatios[i] << " " << totalRemovedMemorys << " " << compression.Map->KeyFramesInMap() << " " << compression.Map->MapPointsInMap();
        // f.close();

        // // Save Map
        // std::string outpath = "build/randomKeyframe/"+finalStr+"RandomKeyframe_Compression_Result_"+to_string(compression.compressionRatios[i])+"_.bin";
        // std::ofstream out(outpath, std::ios_base::binary);
        // if (!out)
        // {
        //     std::cout << "Cannot Write to Database File: " << std::endl;
        //     exit(-1);
        // }
        // boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        // std::cout << " ...done" << std::endl;
        // oa << compression.Map;
        // oa << compression.dbKeyframe;
        // std::cout << " ...done" << std::endl;
        // std::cout << " ...done" << std::endl;
        // out.close();
    }


    return 0;
}