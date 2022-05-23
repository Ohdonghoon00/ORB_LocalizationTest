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
    KeyframeInvWeight = similarityMatrix * invScoreVec;
    std::vector<int> sortIdx(totalKeyframeNum);
    for(size_t i = 0; i < totalKeyframeNum; i++) sortIdx[i] = i;
    std::sort(sortIdx.begin(), sortIdx.end(), scoreComp);

    for(size_t i = 0; i < kfdb.size(); i++){
        
        int idx = sortIdx[i];
        kfdb[idx]->SetBadFlag();
        totalRemovedptsNum++;
        if(CompressionKfRatio >= (double)(totalKeyframeNum - totalRemovedptsNum) / (double)totalKeyframeNum) break;
    }
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
}

void Compression::preparing()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);    
    
    for(size_t i = 0; i < kfdb.size(); i++){
        kfObsNums[i] = getObservation(kfdb[i]);
        kfMpNums[i] = kfdb[i]->GetMapPoints().size();
    }
}

void Compression::getKeyframeScoreVector()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    int totalObs = getTotalObservation();
    int totalMapPoints = getTotalMapPoints();

    for(size_t i = 0; i < kfdb.size(); i++){
        invScoreVec[i] = 1.0 - (obsRatio * ((double)kfObsNums[i] / (double)totalObs ) + mpRatio * ((double)kfMpNums[i] / (double)totalMapPoints ));
    }
}

int Compression::getTotalObservation()
{
    int total = 0;
    for(size_t i = 0; i < kfObsNums.size(); i++){
        int kfObsNum = kfObsNums[i];
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

int Compression::getObservation(ORB_SLAM2::KeyFrame* kf)
{
    std::set<ORB_SLAM2::MapPoint*> kfmp = kf->GetMapPoints();
    int total = 0;
    for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfmp.begin(); iter != kfmp.end(); iter++){
        int mpObsNum = getObservation(*iter);
        total += mpObsNum;
    }
    return total;
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
            int covisibilityMpNum = getCovisibilityMpNum(kfdb[i], kfdb[j]);
            similarityMatrix(i, j) = (double)covisibilityMpNum / (double)lastKeyframeMpNum;
        }
    
    }   
}

int Compression::getCovisibilityMpNum(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2)
{
    std::vector<ORB_SLAM2::MapPoint*> kfMapPoints = kf1->GetMapPointMatches();
    int covisibilityCnt = 0;
        
    for(size_t i = 0; i < kfMapPoints.size(); i++){
            
        if(!kfMapPoints[i]) continue;
        if(!kfMapPoints[i]->isBad()){
                
            bool covisibilityLandmark = kfMapPoints[i]->IsInKeyFrame(kf2);
            if(covisibilityLandmark) covisibilityCnt++;
        }
    }
    return covisibilityCnt;
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

