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

#include<opencv2/core/core.hpp>

#include<System.h>
// #include "Converter.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Load gt trajectory for Evaluation
    std::string queryGtTrajectoryPath = argv[5];
    std::vector<Vector6d> gtPoses;
    SLAM.Loadgt(queryGtTrajectoryPath, &gtPoses);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    std::ofstream ResultFile;
    ResultFile.open("MH01_MH02_original_Result(full_ba).txt");

    // Main loop
    int failFrameNum(0);
    float fail(-0.2);
    double totalTransErr(0.0), totalRotErr(0.0); 
    cv::Mat im;
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

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        std::cout << " @@@@@@@@@@@@@@@@@@@@@@ current query image Num :    " << ni << "  @@@@@@@@@@@@@@@@@@@@@@ " << std::endl; 
        if(ni == 0) sleep(3); // wait 3 sec
        
        cv::Mat abc = SLAM.TrackMonocular(im,tframe);
        
        if(abc.empty()){
            // ResultFile << ni << " " << SLAM.matchNum[ni] << " " << fail << " " << fail << " " << SLAM.refKFid[ni] << " " << SLAM.refKFpts[ni] << std::endl;
            failFrameNum++;
            continue;
        }
        std::cout << abc << std::endl;
        Vector6d currPose = ORB_SLAM2::Converter::Proj2Vec6(abc);
        // std::cout << "final pose : " << currPose << std::endl;
        double err[2];
        SLAM.RMSError(gtPoses[ni], currPose, &err[0]);
        totalTransErr += err[0];
        totalRotErr += ORB_SLAM2::Converter::Rad2Degree(err[1]);
        
        std::cout << "TransError : " << err[0] << std::endl;
        std::cout << "RotError : " << ORB_SLAM2::Converter::Rad2Degree(err[1]) << std::endl;
        ResultFile << ni << " " << SLAM.matchNum[ni] << " " << err[0] << " " << ORB_SLAM2::Converter::Rad2Degree(err[1]) << " " << SLAM.refKFid[ni] << " " << SLAM.refKFpts[ni] << std::endl;

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

        if(ni < 5) cv::waitKey();
    }

    // Stop all threads
    SLAM.Shutdown();
    ResultFile.close();
    
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    std::cout << "Fail Frame Num is : " << failFrameNum << std::endl;
    std::cout << " Average Trans Err : " << totalTransErr/(nImages - failFrameNum) << std::endl;
    std::cout << " Average Rot Err : " << totalRotErr/(nImages - failFrameNum) << std::endl;
    
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
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
            vTimeStamps.push_back(t/1e9);

        }
    }
}
