/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
// #include "Converter.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps, std::vector<double> &beforeFixTimeStamps_);

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

int findGtIdx(std::vector<double> gtposes, double currTime)
{
    int idx = -1;
    for(size_t i = 0; i < gtposes.size(); i++){
        if(gtposes[i] == currTime){
            idx = i;
        }
    }
    return idx;
}

int main(int argc, char **argv)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    std::vector<double> beforeFixTimeStamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps, beforeFixTimeStamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],argv[6], ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Load gt trajectory for Evaluation
    std::string queryGtTrajectoryPath = argv[5];
    std::vector<Vector6d> gtPoses;
    std::vector<double> gtTimeStamp;
    SLAM.Loadgt(queryGtTrajectoryPath, &gtPoses, &gtTimeStamp);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;


    // Save Result
    string tmp = string(argv[6]);
    istringstream tmpStr(tmp);
    string trash;
    vector<string> acc;
    while(getline(tmpStr, trash, '/')) acc.push_back(trash);
    string finalStr = acc[acc.size()-1];

    string tmp_ = string(argv[3]);
    istringstream tmpStr_(tmp_);
    string trash_;
    vector<string> acc_;
    while(getline(tmpStr_, trash_, '/')) acc_.push_back(trash_);
    string finalStr_ = acc_[4];

    // Main loop
    int failFrameNum(0), successFrameNum(0);
    float fail(-0.2);
    double totalTransErr(0.0), totalRotErr(0.0); 
    cv::Mat im;
    std::vector<double> TransErr, RotErr;
    std::ofstream frameResult;
    frameResult.open("result/221007/VPSResult_0.25_2.0/trash/VPS_Result"+finalStr+"Query"+finalStr_+".txt", std::ios::out);
    
    // std::ofstream queryTimeStampResult;
    // queryTimeStampResult.open("/home/ohdonghoon/ORB_LocalizationTest/result/imageEtc/MH03_MH02_timeStamp.txt");

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }


        // Pass the image to the SLAM system
        std::cout << " @@@@@@@@@@@@@@@@@@@@@@ current query image Num :    " << ni << "  @@@@@@@@@@@@@@@@@@@@@@ " << std::endl; 
        if(ni == 0) sleep(3); // wait 3 sec
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        
        cv::Mat abc = SLAM.TrackMonocular(im,tframe);
        
        if(abc.empty()){
            // ResultFile << ni << " " << SLAM.matchNum[ni] << " " << fail << " " << fail << " " << SLAM.refKFid[ni] << " " << SLAM.refKFpts[ni] << std::endl;
            failFrameNum++;
            continue;
        }
        
        // for VPS success Sequence
        double currTime = beforeFixTimeStamps[ni];
        // std::cout << std::setprecision(19) << currTime << std::endl;
        int idx = findGtIdx(gtTimeStamp, currTime);
        
        Vector6d currPose = ORB_SLAM2::Converter::Proj2Vec6(abc);
        // std::cout << " id : " << idx << " ";
        // std::cout << "final pose : " << currPose << std::endl;
        double err[2];
        SLAM.RMSError(currPose, gtPoses[idx], &err[0]);

        if(err[0] > 0.25 || ORB_SLAM2::Converter::Rad2Degree(err[1]) > 2.0){
            failFrameNum++;
            continue;
        }
        

        
        // queryTimeStampResult << std::setprecision(19) << currTime << std::endl;
        totalTransErr += err[0];
        totalRotErr += ORB_SLAM2::Converter::Rad2Degree(err[1]);
        

        // std::cout << currTime << std::endl;
        // std::string imgName = "/home/ohdonghoon/ORB_LocalizationTest/result/imageEtc/MH01_MH03_timeStamp.txt" + to_string_with_precision(currTime, 0) + ".png";
        // cv::imwrite(imgName, im);
        successFrameNum++;
        
        // std::cout << "TransError : " << err[0] << std::endl;
        // std::cout << "RotError : " << ORB_SLAM2::Converter::Rad2Degree(err[1]) << std::endl;
        
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T-ttrack)*1e6)));

        // if(ni < 5) cv::waitKey();

        // standard deviation
        TransErr.emplace_back(err[0]);
        RotErr.emplace_back(ORB_SLAM2::Converter::Rad2Degree(err[1]));

        frameResult << err[0] << " " << ORB_SLAM2::Converter::Rad2Degree(err[1]) << " " << vTimesTrack[ni] << std::endl; 
    }

    // Stop all threads
    SLAM.Shutdown();
    
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    
    // get Variance
    double TransErrAvg = totalTransErr / (double)TransErr.size();
    double RotErrAvg = totalRotErr / (double)RotErr.size();
    double stdTrans(0.0), stdRot(0.0);
    for(size_t i = 0; i < TransErr.size(); i++){
        stdTrans += (TransErr[i] - TransErrAvg) * (TransErr[i] - TransErrAvg);
        stdRot += (RotErr[i] - RotErrAvg) * (RotErr[i] - RotErrAvg);
    }
    stdTrans = std::sqrt(stdTrans / (double)TransErr.size());
    stdRot = std::sqrt(stdRot / (double)RotErr.size());


    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    std::cout << "Fail Frame Num is : " << failFrameNum << "    Success Frame Num is : " << successFrameNum << std::endl;
    SLAM.printFailinfo(); 
    std::cout << " Average Trans Err : " << totalTransErr/(nImages - failFrameNum) << std::endl;
    std::cout << " Average Rot Err : " << totalRotErr/(nImages - failFrameNum) << std::endl;
    std::cout << " standard deviation about Trans Err : " << stdTrans << std::endl;
    std::cout << " standard deviation about Rot Err : " << stdRot << std::endl;
    SLAM.getMap(); // print Landmark Num
    


    
    std::ofstream fout;
    fout.open("result/221007/VPSResult_0.25_2.0/VPS_Result"+finalStr+"Query"+finalStr_+".txt", std::ios::app);
    fout << totalTransErr/(nImages - failFrameNum) << " " << totalRotErr/(nImages - failFrameNum) << " " << (double)(nImages - failFrameNum)/(double)nImages << " " << stdTrans << " " << stdRot << " " << totaltime/nImages << std::endl;
    fout.close();
    frameResult.close();
    
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps, std::vector<double> &beforeFixTimeStamps_)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    beforeFixTimeStamps_.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            beforeFixTimeStamps_.push_back(t);
            vTimeStamps.push_back(t/1e9);

        }
    }
}
