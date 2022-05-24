#include "Compression.h"







void Compression::LandmarkSparsification(double CompressionRatio)
{
    // Map Compression
    std::cout << "Map Compression ... " << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    std::vector<ORB_SLAM2::MapPoint*> mpDB = Map->GetAllMapPoints();
    long unsigned int PointCloudNum = Map->MapPointsInMap();

    std::cout << "before Compression Point Num : " << PointCloudNum << std::endl;
    std::cout << " Create Variables ... " << std::endl;
    // Create Variables
    std::vector<GRBVar> x = CreateVariablesBinaryVector(PointCloudNum, model);

    std::cout << " Set Objective ... " << std::endl;
    // Set Objective
    Eigen::Matrix<double, Eigen::Dynamic, 1> q = CalculateObservationCountWeight(Map);
    SetObjectiveILP(x, q, model);

    std::cout << " Add Constraint ... " << std::endl;
    // Add Constraint
    Eigen::MatrixXd A = CalculateVisibilityMatrix(Map);
    AddConstraint(Map, model, A, x, CompressionRatio);

    std::cout << std::endl;

    std::cout << " Optimize model ... " << std::endl;
    // Optimize model
    model.optimize();

    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    std::cout << std::endl;

    // Erase Map Point
    size_t index = 0;

    int totalObs = 0;
    for (size_t i = 0; i < x.size(); i++){

        if (x[i].get(GRB_DoubleAttr_X) == 0){
            
            int obsCnt = getMemory(mpDB[i]);
            mpDB[i]->SetBadFlag();
            // Map->EraseMapPoint((Map->GetAllMapPoints())[i - index]);
            // Map->EraseMapPoint(i - index);
            index++;
            totalObs += obsCnt;
        }
    }
    // Map->clear();
    
    std::cout << " Finish Map Compression" << std::endl;
    std::cout <<  " Total memory of removed mapPoints : " << totalObs << std::endl;
    long unsigned int PointCloudNum_ = Map->MapPointsInMap();
    std::cout << "After Compression Point Num : " << PointCloudNum_ << std::endl;
}

int Compression::removalKeyframe1()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    int removeKeyframeCnt = 0;
    int removeIdx = 0;
    int totalRemovedptsNum = 0;
    std::cout << " before remove Keyframe num : " << kfdb.size() << std::endl;
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
        int lastFrameMp = kfdb[i - 1 - removeIdx]->GetMapPoints().size();
        int currFrameMp = kfdb[i]->GetMapPoints().size();
        double ratio = (double)covisibilityCnt / (double)currFrameMp; 
        // std::cout << covisibilityCnt << std::endl;
        std::cout << ratio << std::endl;
        
        // if(covisibilityCnt > 100 && ){
        if(covisibilityCnt > 50 && lastFrameMp > currFrameMp){
            int kfmp = kfdb[i]->GetMapPoints().size();
            kfdb[i]->SetBadFlag();
            removeKeyframeCnt++; 
            totalRemovedptsNum += kfmp;       
        } 
    }
    std::cout << " Removed total Keyframe num : " << removeKeyframeCnt << std::endl;
    std::cout << " Removed total Pts : " << totalRemovedptsNum << std::endl;
    return totalRemovedptsNum; 
}

int Compression::removalKeyframe2(double CompressionKfRatio)
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    int totalRemovedptsNum = 0;
    KeyframeInvWeight = invScoreVec;
    // KeyframeInvWeight = similarityMatrix * invScoreVec;
    std::vector<int> sortIdx(totalKeyframeNum);
    for(size_t i = 0; i < totalKeyframeNum; i++) sortIdx[i] = i;
    std::sort(sortIdx.begin(), sortIdx.end(), weightComp);
    
    // std::cout << KeyframeInvWeight.transpose() << std::endl;
    
    for(size_t i = 0; i < kfdb.size(); i++){
        
        int idx = sortIdx[i];
        // std::cout << idx << " " << KeyframeInvWeight[idx] << " !! ";
        kfdb[idx]->SetBadFlag();
        totalRemovedptsNum++;
        if(CompressionKfRatio >= (double)(totalKeyframeNum - totalRemovedptsNum) / (double)totalKeyframeNum) break;
    }
    std::cout << std::endl;
    return totalRemovedptsNum;
}


void Compression::initializing()
{
    totalKeyframeNum = Map->KeyFramesInMap();
    totalMapPointNum = Map->MapPointsInMap();
    
    similarityMatrix.resize(totalKeyframeNum, totalKeyframeNum);
    invScoreVec.resize(totalKeyframeNum);
    
    kfObsNums.resize(totalKeyframeNum);
    kfMpNums.resize(totalKeyframeNum);

    kfObsNumsRatio.resize(totalKeyframeNum);
    kfMpNumsRatio.resize(totalKeyframeNum);

    kfObsRank.resize(totalKeyframeNum);
    kfMpNumRank.resize(totalKeyframeNum);
}

void Compression::preparing()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);    
    
    for(size_t i = 0; i < kfdb.size(); i++){
        kfObsNums[i] = getObservation(kfdb[i]);
        kfMpNums[i] = kfdb[i]->GetMapPoints().size();
    }
    
    TotalObservation = getTotalObservation();
    TotalMapPoints = getTotalMapPoints();
    
    for(size_t i = 0; i < kfdb.size(); i++){
        kfObsNumsRatio[i] = kfObsNums[i] / TotalObservation;
        kfMpNumsRatio[i] = (double)kfMpNums[i] / (double)TotalMapPoints;
    }

    std::vector<int> sortObsIdx(totalKeyframeNum);
    std::vector<int> sortMpIdx(totalKeyframeNum);
    for(size_t i = 0; i < totalKeyframeNum; i++){
        sortObsIdx[i] = i;
        sortMpIdx[i] = i;
    } 
    std::sort(sortObsIdx.begin(), sortObsIdx.end(), obsScoreComp);
    std::sort(sortMpIdx.begin(), sortMpIdx.end(), mpNumScoreComp);

    for(size_t i = 0; i < kfdb.size(); i++){
        
        int obsIdx = sortObsIdx[i];
        int mpIdx = sortMpIdx[i];
        
        kfObsRank[obsIdx] = i;
        kfMpNumRank[mpIdx] = i;
    }    

}

void Compression::getKeyframeScoreVector()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    // int totalObs = getTotalObservation();
    // int totalMapPoints = getTotalMapPoints();

    minMaxNormalize(&kfObsNumsRatio);
    minMaxNormalize(&kfMpNumsRatio);

    for(size_t i = 0; i < kfdb.size(); i++){
        // invScoreVec[i] = 1.0 - (obsRatio * ((double)kfObsNums[i] / (double)totalObs ) + mpRatio * ((double)kfMpNums[i] / (double)totalMapPoints ));
        // std::cout << kfObsNumsRatio[i] << "  " << kfMpNumsRatio[i] << std::endl;
        invScoreVec[i] = 1.0 - (obsRatio * kfObsNumsRatio[i] + mpRatio * kfMpNumsRatio[i]);
    }
    minMaxNormalize(&invScoreVec);

}

double Compression::getTotalObservation()
{
    double total = 0;
    for(size_t i = 0; i < kfObsNums.size(); i++){
        double kfObsNum = kfObsNums[i];
        total += kfObsNum;
    }
    return total;
}

int Compression::getTotalMapPoints()
{
    int total = 0;
    for(size_t i = 0; i < kfMpNums.size(); i++){
        int mpNum = kfMpNums[i];
        total += mpNum;
    }
    return total;   
}

double Compression::getObservation(ORB_SLAM2::KeyFrame* kf)
{
    std::set<ORB_SLAM2::MapPoint*> kfmp = kf->GetMapPoints();
    double total = 0;
    for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfmp.begin(); iter != kfmp.end(); iter++){
        double mpObsNum = (double)getObservation(*iter);
        total += mpObsNum;
    }
    return total/(double)kfmp.size();
}

int Compression::getObservation(ORB_SLAM2::MapPoint* mp)
{
    int obsCnt = mp->mObservations.size();
    return obsCnt;
}

void Compression::getKeyframeSimilarityMatrix()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    for(size_t i = 0; i < kfdb.size(); i++){

        int lastKeyframeMpNum = kfdb[i]->GetMapPoints().size();
        for(size_t j = 0; j < kfdb.size(); j++){
            
            // int currKeyframeMpNum = kfdb[j]->GetMapPoints().size();
            getRelativePose(kfdb[i], kfdb[j]);
            // std::cout << relPoseErr[0] << "    " << ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) << std::endl;
            if(relPoseErr[0] < 3.0 && ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) < 60){

                int covisibilityMpNum = getCovisibilityMpNum(kfdb[i], kfdb[j]);
                similarityMatrix(i, j) = (double)covisibilityMpNum / (double)lastKeyframeMpNum;
            }
            else
                similarityMatrix(i, j) = 0.0;

        }
    
    }
    // std::cout << similarityMatrix<< std::endl;
}

int Compression::getCovisibilityMpNum(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2)
{
    std::set<ORB_SLAM2::MapPoint*> kfMapPoints = kf1->GetMapPoints();
    
    int covisibilityCnt = 0;
    for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfMapPoints.begin(); iter!=kfMapPoints.end(); iter++){
        bool covisibilityLandmark = (*iter)->IsInKeyFrame(kf2);
        if(covisibilityLandmark) covisibilityCnt++;
    }

    return covisibilityCnt;
}

void Compression::getRelativePose(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2)
{
    cv::Mat proj1 = kf1->GetPose();
    cv::Mat proj2 = kf2->GetPose();
    Vector6d Pose1 = ORB_SLAM2::Converter::Proj2Vec6(proj1);
    Vector6d Pose2 = ORB_SLAM2::Converter::Proj2Vec6(proj2);
    
    RMSError(Pose1, Pose2, &relPoseErr[0]);
}

void Compression::printKeyframeInfo()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    std::cout << "ObsNum     " << "ObsNumRatio     " << "ObsRemoveRank     " << "MpNum     " << "MpNumRatio     " << "MpNumRemoveRank     " << std::endl; 
    for(size_t i = 0; i < kfdb.size(); i++){
        std::cout << kfObsNums[i] << "     " << kfObsNumsRatio[i] << "     " << kfObsRank[i] << "     " 
        << kfMpNums[i] << "     " << kfMpNumsRatio[i] << "     " << kfMpNumRank[i] << std::endl;
    }
}

//////////////////////////////////////////////////////////////////////////

int Compression::cvMatSize(cv::Mat a)
{
    int size = a.elemSize() * a.total();
    return size;
}

int Compression::getMemory(ORB_SLAM2::MapPoint* mp)
{
    int obsCnt = mp->mObservations.size();
    obsCnt *= 16;
    obsCnt += 736;
    return obsCnt;
}

void Compression::minMaxNormalize(Eigen::VectorXd *vec)
{
    std::vector<double> vec2(vec->size());
    for(int i = 0; i < vec->size(); i++) vec2[i] = (*vec)[i];
    
    double maxVal = *max_element(vec2.begin(), vec2.end());
    double minVal = *min_element(vec2.begin(), vec2.end());
    
    for(int i = 0; i < vec->size(); i++){
        double a = (*vec)[i];
        (*vec)[i] = (a - minVal) / (maxVal - minVal);
    }
}

void Compression::minMaxNormalize(std::vector<double> *vec)
{   
    double maxVal = *max_element(vec->begin(), vec->end());
    double minVal = *min_element(vec->begin(), vec->end());
    
    for(size_t i = 0; i < vec->size(); i++){
        double a = (*vec)[i];
        (*vec)[i] = (a - minVal) / (maxVal - minVal);
    }
}

void Compression::RMSError(Vector6d EsPose, Vector6d gtPose, double *err)
{
    // err [0] -> trans , [1] -> rot
   
    // RSE Error (root - square error)
    Eigen::Matrix4d RelativePose = ORB_SLAM2::Converter::To44RT(gtPose).inverse() * ORB_SLAM2::Converter::To44RT(EsPose);
        
    // trans
    Eigen::Vector3d RelativeTrans;
    RelativeTrans << RelativePose(0, 3), RelativePose(1, 3), RelativePose(2, 3);
    err[0] = std::sqrt(RelativeTrans.dot(RelativeTrans));
        
    // rotation
    Eigen::Matrix3d RelativeRot_ = RelativePose.block<3, 3>(0, 0);
    Eigen::Vector3d RelativeRot = ORB_SLAM2::Converter::ToVec3(RelativeRot_);
    err[1] = std::sqrt(RelativeRot.dot(RelativeRot));

}
