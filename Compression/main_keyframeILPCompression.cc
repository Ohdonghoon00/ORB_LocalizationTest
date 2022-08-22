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

    if(argc != 8){
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
    

    double compRatio = std::stod(argv[2]);
    compression.setInitial(compRatio);
    std::cout << "id threshold : " << compression.neighborKeyframeIdThres << std::endl;

///////////////////////////////////////////////////////////////////////////    
/////////////////////// Keyframe Compression !! ///////////////////////////
    std::cout << " remove Keyframe ... " << std::endl;
    int totalRemovedMemory = compression.removalKeyframeILP(
        stod(argv[4]),stod(argv[5]),stod(argv[6]),stod(argv[7])
    );
    std::cout << " Finish Compression !! " << std::endl;
    
    // memory for removing keyframe
    std::cout << "Original Map points Num : " << compression.originalMapPointNum << std::endl;
    int totalRemovedLandmark = compression.originalMapPointNum - compression.Map->MapPointsInMap();
    totalRemovedMemory += totalRemovedLandmark * 736;
    compression.removedMemory = (double)totalRemovedMemory * 1e-6;
    
    std::cout <<  " Total memory of removed mapPoints : " << totalRemovedMemory << std::endl;
    std::cout << " Keyframe Num : " << compression.Map->KeyFramesInMap() << " Landmark Num : " << compression.Map->MapPointsInMap() << std::endl;

    
    // Save Result
    std::ofstream f;

    string tmp = string(argv[1]);
    istringstream tmpStr(tmp);
    string trash;
    vector<string> acc;
    while(getline(tmpStr, trash, '/')) acc.push_back(trash);
    string finalStr = acc[4];

    f.open("build/Db/CompressionResult/"+finalStr+"KeyframeILP_Compression_Result_"+string(argv[2])+"_"+string(argv[4])+"_"+string(argv[5])+"_"+string(argv[6])+"_"+string(argv[7])+"_.txt", ios::out);
    f << 1.0 - compression.kfCompressedRatio << " " << compression.removedMemory << " " << compression.Map->KeyFramesInMap() << " " << compression.Map->MapPointsInMap();
    f.close();

    // Save Map data
    std::cout << "Save Map ... " << std::endl;
    std::string outpath = string(argv[3])+"_"+string(argv[4])+"_"+string(argv[5])+"_"+string(argv[6])+"_"+string(argv[7])+"_.bin";
    // std::string compRatio_string = std::to_string((int)compRatio*100);
    // std::string outpath = outName + compRatio_string;
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




