#include "Compression.h"
#include <random>
#include <ctime>





void Compression::LandmarkSparsification()
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
    std::vector<GRBVar> x = CreateVariablesBinaryVectorForLandmark(PointCloudNum, model);

    std::cout << " Set Objective ... " << std::endl;
    // Set Objective
    Eigen::Matrix<double, Eigen::Dynamic, 1> q = CalculateObservationCountWeight(Map);
    SetObjectiveILPforLandmark(x, q, model);

    std::cout << " Add Constraint ... " << std::endl;
    // Add Constraint
    Eigen::MatrixXd A = CalculateVisibilityMatrix(Map);
    AddConstraintForLandmark(Map, model, A, x, kfCompressedRatio);

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
        
        if(covisibilityCnt > 100){
        // if(covisibilityCnt > 50 && lastFrameMp > currFrameMp){
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

int Compression::removalKeyframe2()
{


    int totalRemovedKeyframeNum = 0;
    int iterateNum = 0;
    int totalRemoveMemory = 0;
    
    constexpr int MIN = 0;
    constexpr int MAX = 1;

    // std::cout << KeyframeInvWeight.transpose() << std::endl;
    
    while(true){
        
        std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
        std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);        
        
        iterateKeyframeRemoval();
        
        std::cout << "...................................     iterate  ....   " <<  iterateNum <<  std::endl; 
    
        // Eigen::VectorXd invScoreVec_(totalKeyframeNum);
        // for(int i = 0; i < invScoreVec_.size(); i++) invScoreVec_[i] = 1.0;        
        
        // minMaxNormalize(&reprojectionErrAvg);
        // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = 1.0 - (double)reprojectionErrInlier[i];
        for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = (double)reprojectionErrAvg[i];
        // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = invScoreVec[i] * (double)reprojectionErrAvg[i];
        // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = invScoreVec[i] * (1.0 - (double)reprojectionErrInlier[i]);
        // KeyframeInvWeight = invScoreVec;

        // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = 1.0 - obs1NumRatio[i];
        // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = (1.0 - obs1NumRatio[i]) * invScoreVec[i];
        
        // srand((unsigned int)time(NULL));
        // for(int i = 0; i < totalKeyframeNum; i++){
        //     double f = (double)rand() / RAND_MAX;
        //     KeyframeInvWeight[i] = MIN + f * (MAX - MIN);
        // }    
        


        std::vector<int> sortIdx(totalKeyframeNum);
        for(size_t j = 0; j < totalKeyframeNum; j++) sortIdx[j] = j;
            
        
        std::sort(sortIdx.begin(), sortIdx.end(), weightComp);
        long unsigned int sortArr = 0;
        int neightborIter = 0;
        bool reachRemoveKf = false;

        while(true){
            
            int idx = sortIdx[sortArr] + 1;
            // int neighborKfNum = getNeighborKeyframeNum(kfdb[idx]);
            int neighborKfNum = 100;
            int neighborKfIdDist = getNeighborKeyframeIdDist(idx);
            double obs1Ratio = getKeyframeObs1Num(kfdb[idx]) ;
            // && obs1Ratio < obs1RatioThres
            // std::cout << "idid : " << idx << std::endl;
            // std::cout << "high value : " << obs1Ratio << std::endl;
            if(neighborKfNum > neighborKeyframeNumThres && neighborKfIdDist <= neighborKeyframeIdThres){
                std::cout << " removed keyframe Id : " << kfdb[idx]->mnId << "     neighbor Keyframe Num : " << neighborKfNum << "   neighbor iter num : " << neightborIter << std::endl;
                std::cout << " Obs 1 Num Ratio : " << obs1Ratio << std::endl;
                std::cout << " id distant : " << neighborKfIdDist << std::endl;
                int currMemory = getMemory(kfdb[idx]);
                totalRemoveMemory += currMemory;
                kfdb[idx]->SetBadFlag();
                kfNewIds.erase(kfNewIds.begin() + idx);
                break;
            }
            else{
                sortArr++;
                neightborIter++;
                if(sortArr == totalKeyframeNum - 1){
                    reachRemoveKf = true;
                    std::cout << " cannot remove keyframe anymore " << std::endl;
                    break;
                } 
                continue;
            }
        }
        
        totalRemovedKeyframeNum++;
        iterateNum++;
        if(reachRemoveKf) break;
        if(kfCompressedRatio >= (double)(originalKeyframeNum - totalRemovedKeyframeNum) / (double)originalKeyframeNum) break;
        
    }
    std::cout << std::endl;
    // std::cout <<  " Total memory of removed mapPoints : " << totalRemoveMemory << std::endl;
    return totalRemoveMemory;
}

int Compression::removalKeyframe3()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);
    int totalRemoveMemory = 0;

    // Map Compression
    std::cout << "Map Compression ... " << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);    
    
    int keyframeNum = (int)Map->KeyFramesInMap();

    std::cout << "before Compression keyframe Num : " << keyframeNum << std::endl;
    std::cout << " Create Variables ... " << std::endl;
    // Create Variables
    std::vector<GRBVar> x = CreateVariablesBinaryVectorForKeyframe(keyframeNum, model);

    std::cout << " Set Objective ... " << std::endl;
    // Set Objective
    Eigen::MatrixXd S = calculateKeyframeSimilarity(Map);
    SetObjectiveILPforKeyframe(x, S, model);

    std::cout << " Add Constraint ... " << std::endl;
    // Add Constraint
    AddConstraintForKeyframe(Map, model, x, kfCompressedRatio, neighborKeyframeIdThres);

    std::cout << std::endl;

    std::cout << " Optimize model ... " << std::endl;
    // Optimize model
    model.optimize();

    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    std::cout << std::endl;

    // Erase Keyframe

    for (size_t i = 0; i < x.size(); i++){

        if (x[i].get(GRB_DoubleAttr_X) == 0){
            
            int currMemory = getMemory(kfdb[i + 1]);
            totalRemoveMemory += currMemory;
            kfdb[i + 1]->SetBadFlag();

        }
    }

    return totalRemoveMemory;

    
} // remove keyframe by ILP method

int Compression::removalKeyframe4()
{
    
    
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);
    int totalRemoveMemory = 0;

    // Map Compression
    std::cout << "Map Compression ... " << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);    
    
    int keyframeNum = (int)Map->KeyFramesInMap();
    std::cout << "before Compression keyframe Num : " << keyframeNum << std::endl;

    std::cout << " Create Variables ... " << std::endl;
    // Create Variables
    std::vector<GRBVar> x = CreateVariablesBinaryVectorForKeyframe(keyframeNum, model);
    
    std::cout << "estimate cube ... " << std::endl;
    estimateCube();
    std::cout << "VisibilityMatrix ... " << std::endl;
    getVisibilityMatrix();
    std::cout << "OriginalCubeVector ... " << std::endl;
    getOriginalCubeVector();

    // Set Objective
    std::cout << " Set Objective ... " << std::endl;
    SetObjectiveforKeyframeMapCube(x, visibilityMatrix, originalCubeVector, cubeIds, cubeIdsSet, model);

    std::cout << " Add Constraint ... " << std::endl;
    // Add Constraint
    AddConstraintForKeyframe(Map, model, x, kfCompressedRatio, neighborKeyframeIdThres);

    std::cout << " Optimize model ... " << std::endl;
    
    // try{
    // Optimize model
    model.optimize();

    // } catch (GRBException e) {
    //     cout << "Error number: " << e.getErrorCode() << endl;
    //     cout << e.getMessage() << endl;
    // } 
    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    std::cout << std::endl;

    // Erase Keyframe

    for (size_t i = 0; i < x.size(); i++){

        if (x[i].get(GRB_DoubleAttr_X) == 0){
            
            int currMemory = getMemory(kfdb[i + 1]);
            totalRemoveMemory += currMemory;
            kfdb[i + 1]->SetBadFlag();

        }
    }

    return totalRemoveMemory;

}

void Compression::initializing()
{
    totalKeyframeNum = Map->KeyFramesInMap() - 1; // Exclude first Keyframe 
    totalMapPointNum = Map->MapPointsInMap();
    
    reprojectionErrAvg.resize(totalKeyframeNum);
    reprojectionErrInlier.resize(totalKeyframeNum);
    obs1NumRatio.resize(totalKeyframeNum);
    similarityMatrix.resize(totalKeyframeNum, totalKeyframeNum);
    invScoreVec.resize(totalKeyframeNum);
    KeyframeInvWeight.resize(totalKeyframeNum);

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
    
    for(size_t i = 0; i < totalKeyframeNum; i++){
        kfObsNums[i] = getObservation(kfdb[i + 1]);
        kfMpNums[i] = getKeyframeMap(kfdb[i + 1]).size();
    }
    
    TotalObservation = getTotalObservation();
    TotalMapPoints = getTotalMapPoints();
    
    for(size_t i = 0; i < totalKeyframeNum; i++){
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

    for(size_t i = 0; i < totalKeyframeNum; i++){
        
        int obsIdx = sortObsIdx[i];
        int mpIdx = sortMpIdx[i];
        
        kfObsRank[obsIdx] = i;
        kfMpNumRank[mpIdx] = i;
    }    

}

void Compression::getKeyframeScoreVector()
{
    // std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    // std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    // int totalObs = getTotalObservation();
    // int totalMapPoints = getTotalMapPoints();

    minMaxNormalize(&kfObsNumsRatio);
    minMaxNormalize(&kfMpNumsRatio);

    for(size_t i = 0; i < totalKeyframeNum; i++){
        // invScoreVec[i] = 1.0 - (obsRatio * ((double)kfObsNums[i] / (double)totalObs ) + mpRatio * ((double)kfMpNums[i] / (double)totalMapPoints ));
        // std::cout << kfObsNumsRatio[i] << "  " << kfMpNumsRatio[i] << std::endl;
        // invScoreVec[i] = 1.0 - (obsRatio * kfObsNumsRatio[i]); // for debug
        invScoreVec[i] = (obsRatio * kfObsNumsRatio[i]);
    }
    // minMaxNormalize(&invScoreVec);

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
    // std::set<ORB_SLAM2::MapPoint*> kfmp = kf->GetMapPoints();
    
    std::vector<ORB_SLAM2::MapPoint*> kfmp = getKeyframeMap(kf);
    double total = 0;
    
    // for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfmp.begin(); iter != kfmp.end(); iter++){
    //     double mpObsNum = (double)getObservation(*iter);
    //     total += mpObsNum;
    // }

    for(size_t i = 0; i < kfmp.size(); i++){
        double mpObsNum = (double)getObservation(kfmp[i]);
        // std::cout << i << "   " << mpObsNum << std::endl;
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

    for(size_t i = 1; i < kfdb.size(); i++){

        int lastKeyframeMpNum = getKeyframeMap(kfdb[i]).size();
        for(size_t j = 1; j < kfdb.size(); j++){
            
            // getRelativePose(kfdb[i], kfdb[j]);
            // // std::cout << relPoseErr[0] << "    " << ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) << std::endl;
            // if(relPoseErr[0] < 1.0){

                int covisibilityMpNum = getCovisibilityMpNum(kfdb[i], kfdb[j]);
                similarityMatrix(i - 1, j - 1) = (double)covisibilityMpNum / (double)lastKeyframeMpNum;
            // }
            // else
            //     similarityMatrix(i - 1, j - 1) = 0.0;

        }
    
    }
    // std::cout << similarityMatrix<< std::endl;
}

void Compression::getAllKeyframeReprojErr()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);  

    for(size_t i = 1; i < kfdb.size(); i++){
        reprojectionErrAvg[i - 1] =  getreprojectionErrAvg(kfdb[i]);
        reprojectionErrInlier[i - 1] =  inlierRatioInKeyframe;
        // std::cout << std::endl;
    } 
}

float Compression::getreprojectionErrAvg(ORB_SLAM2::KeyFrame* kf)
{
    float reprojErr = 0.0;
    // std::set<ORB_SLAM2::MapPoint*> kfMapPoints = kf->GetMapPoints();
    // test
    std::vector<ORB_SLAM2::MapPoint*> kfMapPoints = getKeyframeMap(kf);

    Eigen::Matrix4Xf worldPoints(4, kfMapPoints.size());
    Eigen::Matrix3Xf imgPoints(3, kfMapPoints.size());
    Eigen::Matrix3Xf reprojectPoints(3, kfMapPoints.size());
    Eigen::Matrix<float, 3, 4> proj = ORB_SLAM2::Converter::toProj34(kf->GetPose());
    Eigen::Matrix3f K_;
    K_ <<   kf->fx, 0, kf->cx,
            0, kf->fy, kf->cy,
            0, 0, 1;
    


    for(size_t i = 0; i < kfMapPoints.size(); i++){
        
        cv::Mat mapPoint = (kfMapPoints[i])->GetWorldPos();
        int idx = (kfMapPoints[i])->GetIndexInKeyFrame(kf);

        worldPoints(0, i) = mapPoint.at<float>(0, 0);
        worldPoints(1, i) = mapPoint.at<float>(1, 0);
        worldPoints(2, i) = mapPoint.at<float>(2, 0);
        worldPoints(3, i) = 1.0f;
        
        imgPoints(0, i) = kf->mvKeys[idx].pt.x;
        imgPoints(1, i) = kf->mvKeys[idx].pt.y;
        imgPoints(2, i) = 1.0f;
        
        // std::cout << imgPoints(0, i) << " " << imgPoints(1, i) << " " << imgPoints(2, i) << std::endl;
        // std::cout << worldPoints(0, i) << " " << worldPoints(1, i) << " " << worldPoints(2, i) << " " << worldPoints(3, i) << std::endl;
    }
    
    // get reprojection Error 
    reprojectPoints = proj * worldPoints;
    
    
    for(int i = 0; i < reprojectPoints.cols(); i++){
        reprojectPoints(0, i) /= reprojectPoints(2, i);
        reprojectPoints(1, i) /= reprojectPoints(2, i);
        reprojectPoints(2, i) /= reprojectPoints(2, i);
    // std::cout << reprojectPoints(0, i) << " " << reprojectPoints(1, i) << " " << reprojectPoints(2, i) << std::endl; 
    }
    reprojectPoints = K_ * reprojectPoints;    

    int inlier = 0;
    std::vector<float> ReprojectErr(worldPoints.cols());
    for(int i = 0; i < worldPoints.cols(); i++){
        ReprojectErr[i] = std::sqrt( (imgPoints(0, i) - reprojectPoints(0, i)) * 
                                     (imgPoints(0, i) - reprojectPoints(0, i)) + 
                                     (imgPoints(1, i) - reprojectPoints(1, i)) *
                                     (imgPoints(1, i) - reprojectPoints(1, i)) );
        reprojErr += ReprojectErr[i];
        if(ReprojectErr[i] < 1.5) inlier++;
        // std::cout << ReprojectErr[i] << " ";
    }
    inlierRatioInKeyframe = (float)inlier / (float)kfMapPoints.size();
    // std::cout << reprojErr/(float)kfMapPoints.size() << std::endl;
    return reprojErr/(float)kfMapPoints.size();
}

int Compression::getCovisibilityMpNum(ORB_SLAM2::KeyFrame* kf1, ORB_SLAM2::KeyFrame* kf2)
{
    // std::set<ORB_SLAM2::MapPoint*> kfMapPoints = kf1->GetMapPoints();
    std::vector<ORB_SLAM2::MapPoint*> kfMapPoints = getKeyframeMap(kf1);
    
    int covisibilityCnt = 0;
    // for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfMapPoints.begin(); iter!=kfMapPoints.end(); iter++){
    //     bool covisibilityLandmark = (*iter)->IsInKeyFrame(kf2);
    //     if(covisibilityLandmark) covisibilityCnt++;
    // }

    for(size_t i = 0; i < kfMapPoints.size(); i++){
        bool covisibilityLandmark = (kfMapPoints[i])->IsInKeyFrame(kf2);
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

int Compression::getNeighborKeyframeNum(ORB_SLAM2::KeyFrame* kf)
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);    
    int neighborKfNum = 0;
    
    for(size_t i = 1; i < kfdb.size(); i++){
        
        getRelativePose(kf, kfdb[i]);
        if(relPoseErr[0] < neighborKeyframeTranslationThres){ // && ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) < neighborKeyframeRotationThres) neighborKfNum++;
            int shareMapPointNum = getCovisibilityMpNum(kf, kfdb[i]);
            int kfMapPointNum = getKeyframeMap(kf).size();
            double ratio = (double)shareMapPointNum / (double)kfMapPointNum; 
            // std::cout << "ratio ? : " << ratio << std::endl;
            // if(ratio > 0.2) neighborKfNum++;
            neighborKfNum++;
        } 
        // if(relPoseErr[0] < neighborKeyframeTranslationThres) neighborKfNum++;
    }
    
    return neighborKfNum;
}

int Compression::getNeighborKeyframeIdDist(int idx)
{
    int idDist = 0;
    if(kfNewIds[idx] == kfNewIds.back()) idDist = maxNewid - kfNewIds[idx - 1];
    else{
        idDist = kfNewIds[idx + 1] - kfNewIds[idx - 1];

    }
    
    return idDist;
}

void Compression::setInitial(double kfCompressedRatio_)
{
    originalKeyframeNum = (int)Map->GetAllKeyFrames().size();
    originalMapPointNum = (int)Map->GetAllMapPoints().size();
    
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);
    
    kfNewIds.resize(originalKeyframeNum); 

    for(size_t i = 0; i < kfdb.size(); i++){
        
        kfdb[i]->newId = i;
        kfNewIds[i] = i;
        if(i > 0){
            getRelativePose(kfdb[i - 1], kfdb[i]);
            neighborKeyframeTranslationThres += relPoseErr[0];
        }
    }
    maxNewid = kfdb.size() - 1;
    kfCompressedRatio = kfCompressedRatio_;
    neighborKeyframeIdThres = ((int)1/kfCompressedRatio) * 2 + 1;
    
}

void Compression::printKeyframeInfo(const std::string &file)
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    std::ofstream saveFile;
    saveFile.open(file);

    // Eigen::VectorXd invScoreVec_(totalKeyframeNum);
    // for(int i = 0; i < invScoreVec_.size(); i++) invScoreVec_[i] = 1.0;
    // Eigen::VectorXd KeyframeInvWeight_(totalKeyframeNum);        
    // KeyframeInvWeight_ = similarityMatrix * invScoreVec_;
    // minMaxNormalize(&KeyframeInvWeight);
    // KeyframeInvWeight_ = KeyframeInvWeight + invScoreVec;
    // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = invScoreVec[i] * reprojectionErrAvg[i];
    KeyframeInvWeight = invScoreVec;
    std::cout << "KeyframeId     " << "ObsNum     " << "ObsNumRatio     " << "ObsRemoveRank     " << "MpNum     " << "MpNumRatio     " << "MpNumRemoveRank     " << std::endl; 
    for(size_t i = 1; i < kfdb.size(); i++){
        std::cout << kfdb[i]->mnId << "     " << kfObsNums[i - 1] << "     " << kfObsNumsRatio[i - 1] << "     " << kfObsRank[i - 1] << "     " 
        << kfMpNums[i - 1] << "     " << kfMpNumsRatio[i - 1] << "     " << kfMpNumRank[i - 1] << "     " << KeyframeInvWeight[i - 1] << "    " << reprojectionErrAvg[i - 1] << "   " << reprojectionErrInlier[i  - 1] << std::endl;
        saveFile << kfdb[i]->mnId << " " << kfObsNums[i - 1] << " " << kfMpNums[i - 1] << " " << reprojectionErrInlier[i - 1] << " " 
        << reprojectionErrAvg[i - 1] << " " << KeyframeInvWeight[i - 1] << std::endl;
    }


}

std::vector<ORB_SLAM2::MapPoint*> Compression::getKeyframeMap(ORB_SLAM2::KeyFrame* kf)
{
    std::vector<ORB_SLAM2::MapPoint*> kfMap;
    std::set<ORB_SLAM2::MapPoint*> mp = kf->GetMapPoints();
    for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = mp.begin(); iter!=mp.end(); iter++){
        bool Inkeyframe = (*iter)->IsInKeyFrame(kf);
        if(Inkeyframe){
            kfMap.push_back(*iter);
        }
    }
    return kfMap;
}

double Compression::getKeyframeObs1Num(ORB_SLAM2::KeyFrame* kf)
{
    int obs1Num = 0;
    std::vector<ORB_SLAM2::MapPoint*> kfmpDb = getKeyframeMap(kf);
    for(size_t i = 0; i < kfmpDb.size(); i++){
        int obsNum = kfmpDb[i]->GetObservations().size();
        if(obsNum == 1) obs1Num++;
    }
    return (double)obs1Num / (double)kfmpDb.size();
}

void Compression::getAllObs1Ratio()
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);    
    
    for(size_t i = 1; i < kfdb.size(); i++){
        
        obs1NumRatio[i - 1] = getKeyframeObs1Num(kfdb[i]);

    }
}

void Compression::iterateKeyframeRemoval()
{
    initializing();
    preparing();
    getKeyframeScoreVector();
    // getKeyframeSimilarityMatrix();
    getAllKeyframeReprojErr();
    getAllObs1Ratio();   
}

void Compression::estimateCube()
{
    std::cout << "get small cube ... " << std::endl;
    getSmallCube();
    std::cout << "get CubeIds ... " << std::endl;
    getCubeIds();
}

int Compression::getOriginalCubeVector()
{
    originalCubeVector.resize(totalSmallCubeNum, 0);

    // Create original Observation Vector
    std::vector<int> observationVector(visibilityMatrix.rows()); // Landmark Num
    std::vector<int> x(visibilityMatrix.cols(), 1); // Keyframe Num
    
    std::cout << "Caculate observation vector ... " << std::endl;
    for(int i = 0; i < visibilityMatrix.rows(); i++){
        
        int obs = 0;
        for(int j = 0; j < visibilityMatrix.cols(); j++){
            obs += visibilityMatrix(i, j) * x[j];
        }
        if(obs == 0)
            observationVector[i] = 0;
        else
            observationVector[i] = 1;
    }
    
    // Caculate original MapPoint Num in Small Cube
    std::cout << "Caculate originalCubeVector ... " << std::endl;
    for(size_t i = 0; i < cubeIds.size(); i++){
        
        int cubeid_ = cubeIds[i];
        int idx = getSetIndex(cubeIdsSet, cubeid_);
        
        if(idx == -1){
            
            std::cout << "not found,,,?? " << std::endl;
        }
        else{
            originalCubeVector[idx] += 1 * observationVector[i];
        }
    }
    
    
    // delete smaller than thres
    // std::cout << "delete smaller than thres ... " << std::endl;
    // int eraseIdx = 0;
    // for(size_t i = 0; i < originalCubeVector.size(); i++){
    //     if(originalCubeVector[i - eraseIdx] < 5){
            
    //         originalCubeVector.erase(originalCubeVector.begin() + i - eraseIdx);
    //         eraseIdx++;
    //     }
    // }
}


void Compression::getBigCube(   std::vector<ORB_SLAM2::MapPoint*> mpDb,
                                Eigen::Vector3d* minPoint,
                                Eigen::Vector3d* maxPoint)
{
    for(size_t i = 0; i < mpDb.size(); i++){
        
        cv::Mat pointPos = mpDb[i]->GetWorldPos();
        Eigen::Vector3d pos;
        pos <<  (double)pointPos.at<float>(0, 0),
                (double)pointPos.at<float>(1, 0),
                (double)pointPos.at<float>(2, 0);

        minPoint->x() = std::min(minPoint->x(), pos.x());
        minPoint->y() = std::min(minPoint->y(), pos.y());
        minPoint->z() = std::min(minPoint->z(), pos.z());
	
        maxPoint->x() = std::max(maxPoint->x(), pos.x());
        maxPoint->y() = std::max(maxPoint->y(), pos.y());
        maxPoint->z() = std::max(maxPoint->z(), pos.z());
    }    
}

void Compression::getSmallCube()
{
    std::vector<ORB_SLAM2::MapPoint*> mpDB = Map->GetAllMapPoints();
    Eigen::Vector3d minp(DBL_MAX, DBL_MAX, DBL_MAX), maxp(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    getBigCube(mpDB, &minp, &maxp);
    
    minPoint.x() = (minp.x() >=0) ? std::ceil(minp.x()) : std::floor(minp.x());
    minPoint.y() = (minp.y() >=0) ? std::ceil(minp.y()) : std::floor(minp.y());
    minPoint.z() = (minp.z() >=0) ? std::ceil(minp.z()) : std::floor(minp.z());

    maxPoint.x() = (maxp.x() >=0) ? std::ceil(maxp.x()) : std::floor(maxp.x());
    maxPoint.y() = (maxp.y() >=0) ? std::ceil(maxp.y()) : std::floor(maxp.y());
    maxPoint.z() = (maxp.z() >=0) ? std::ceil(maxp.z()) : std::floor(maxp.z());
    
    // small Cube Num
    smallCubeXNum = maxPoint.x() - minPoint.x();
    smallCubeYNum = maxPoint.y() - minPoint.y();
    smallCubeZNum = maxPoint.z() - minPoint.z();
    std::cout << "SmallCubeNum    : " << smallCubeXNum << "  " << smallCubeYNum << "  " << smallCubeZNum << std::endl;
    totalSmallCubeNum = smallCubeXNum * smallCubeYNum * smallCubeZNum;
    std::cout << "totalSmallCubeNum    : " << totalSmallCubeNum << std::endl; 



}

int Compression::getCubeId(ORB_SLAM2::MapPoint* mp)
{
    int cubeId = 0;
    
    cv::Mat pointPos = mp->GetWorldPos();
    Eigen::Vector3d pos;
    pos <<  (double)pointPos.at<float>(0, 0),
            (double)pointPos.at<float>(1, 0),
            (double)pointPos.at<float>(2, 0);

    int cubeIdX = std::floor(pos.x() - (double)minPoint.x());
    int cubeIdY = std::floor(pos.y() - (double)minPoint.y());
    int cubeIdZ = std::floor(pos.z() - (double)minPoint.z());

    cubeId = cubeIdX + smallCubeXNum * cubeIdY + smallCubeXNum * smallCubeYNum * cubeIdZ;
    return cubeId;
}

void Compression::getCubeIds()
{
    std::vector<ORB_SLAM2::MapPoint*> mpDB = Map->GetAllMapPoints();
    cubeIds.resize(mpDB.size());
    for(size_t i = 0; i < mpDB.size(); i++){
        int cubeId = getCubeId(mpDB[i]);
        cubeIds[i] = cubeId;
    }

    for(auto i : cubeIds) cubeIdsSet.insert(i);
    
    std::cout << "cube Num : " << cubeIdsSet.size() << std::endl;
    totalSmallCubeNum = cubeIdsSet.size();
}


void Compression::getVisibilityMatrix()
{
    std::vector<ORB_SLAM2::MapPoint*> mpDB = Map->GetAllMapPoints();
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);
    
    visibilityMatrix.resize(originalMapPointNum, originalKeyframeNum - 1);
    visibilityMatrix.setZero();

    for(size_t i = 0; i < mpDB.size(); i++){
        for(size_t j = 1; j < kfdb.size(); j++){
            bool isInKeyframe = mpDB[i]->IsInKeyFrame(kfdb[j]);
            if(isInKeyframe){
                visibilityMatrix(i, j - 1) = 1;
            }
        }
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
    obsCnt *= 48; // observation(16) + descriptor(32)
    obsCnt += 736; // etc ... 
    return obsCnt;
}

int Compression::getMemory(ORB_SLAM2::KeyFrame* kf)
{
    int memory = 0;
    std::vector<ORB_SLAM2::MapPoint*> kfmp = getKeyframeMap(kf);
    int mapPointNum = kfmp.size();
    int kfCovisibility = kf->mConnectedKeyFrameWeights.size();
    
    memory += mapPointNum * 76; // Keypoint(28) + descriptor(32) + observation(16)
    memory += kfCovisibility * 24; // Keyframe Pointer(8) * 2 + mpCovisibilityNum(weight, 4) * 2 
    memory += 26024; // etc ... 
    
    return memory;
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

void Compression::minMaxNormalize(Eigen::VectorXf *vec)
{
    std::vector<float> vec2(vec->size());
    for(int i = 0; i < vec->size(); i++) vec2[i] = (*vec)[i];
    
    float maxVal = *max_element(vec2.begin(), vec2.end());
    float minVal = *min_element(vec2.begin(), vec2.end());
    
    for(int i = 0; i < vec->size(); i++){
        float a = (*vec)[i];
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

int Compression::getSetIndex(std::set<int> S, int K)
{
 
    int Index = 0;
 
    for (auto u : S) {
 
        if (u == K)
            return Index;
 
        Index++;
    }
    
    return -1;
}