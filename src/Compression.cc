#include "Compression.h"
#include <random>
#include <ctime>

void Compression::LandmarkSparsification()
{
    // Map Compression
    std::cout << "Map Compression ... " << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
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
    for (size_t i = 0; i < x.size(); i++)
    {

        if (x[i].get(GRB_DoubleAttr_X) == 0)
        {

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
    std::cout << " Total memory of removed mapPoints : " << totalObs << std::endl;
    long unsigned int PointCloudNum_ = Map->MapPointsInMap();
    std::cout << "After Compression Point Num : " << PointCloudNum_ << std::endl;

    remainLandmark = PointCloudNum_;
    removedMemory = (double)totalObs * 1e-6;
}

void Compression::LandmarkSparsification2(
    const double a,
    const double b,
    const double c,
    const double d)
{
    try
    {
        // Map Compression
        std::cout << "Map Compression ... " << std::endl;
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);

        std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
        long unsigned int PointCloudNum = Map->MapPointsInMap();

        std::cout << "before Compression Point Num : " << PointCloudNum << std::endl;
        std::cout << " Create Variables ... " << std::endl;
        // Create Variables
        std::vector<GRBVar> x = CreateVariablesBinaryVectorForLandmark(PointCloudNum, model);

        std::cout << " Set Objective ... " << std::endl;
        // Set Objective
        getLandmarkScore(a, b, c, d);
        std::cout << "score" << std::endl;
        Eigen::Matrix<double, Eigen::Dynamic, 1> q = setLandmarkWeight(Map, landmarkScore);
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
        for (size_t i = 0; i < x.size(); i++)
        {

            if (x[i].get(GRB_DoubleAttr_X) == 0)
            {

                int obsCnt = getMemory(mpDB[i]);
                mpDB[i]->SetBadFlag();
                index++;
                totalObs += obsCnt;
            }
        }

        std::cout << " Finish Map Compression" << std::endl;
        std::cout << " Total memory of removed mapPoints : " << totalObs << std::endl;
        long unsigned int PointCloudNum_ = Map->MapPointsInMap();
        std::cout << "After Compression Point Num : " << PointCloudNum_ << std::endl;

        remainLandmark = PointCloudNum_;
        removedMemory = (double)totalObs * 1e-6;
    }
    catch (GRBException e)
    {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
} // Remove Landmark Score ILP

void Compression::LandmarkSparsificationIQP(
    const double a,
    const double b,
    const double c,
    const double d)
{
    try
    {
        // Map Compression
        std::cout << "Map Compression ... " << std::endl;
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);

        std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
        long unsigned int PointCloudNum = Map->MapPointsInMap();

        std::cout << "before Compression Point Num : " << PointCloudNum << std::endl;
        std::cout << " Create Variables ... " << std::endl;
        // Create Variables
        std::vector<GRBVar> x = CreateVariablesBinaryVectorForLandmark(PointCloudNum, model);

        std::cout << " Set Objective ... " << std::endl;
        // Set Objective
        getLandmarkScore(a, b, c, d);
        std::cout << "set landmark score ... " << std::endl;
        Eigen::Matrix<double, Eigen::Dynamic, 1> q = setLandmarkWeight(Map, landmarkScore);
        
        // Set IQP Weight
        std::cout << " set Quardratic IQP Weight Score ... " << std::endl;
        std::map<std::tuple<int, int>, double> distWeightQ = getKeypointDistanceMatrix();
        std::cout << " set Objective ... " << std::endl;
        SetObjectiveIQPforLandmark(x, q, distWeightQ, model);

        std::cout << " Add Constraint ... " << std::endl;
        // Add Constraint
        Eigen::MatrixXd A = CalculateVisibilityMatrix(Map);
        AddConstraintForLandmark(Map, model, A, x, kfCompressedRatio);

        std::cout << std::endl;

        std::cout << " Optimize model ... " << std::endl;
        // Optimize model
        // model.set(GRB_IntParam_Crossover, 2);
        model.set(GRB_DoubleParam_TimeLimit, 5000);
        model.optimize();

        std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        std::cout << std::endl;

        // Erase Map Point
        size_t index = 0;

        int totalObs = 0;
        for (size_t i = 0; i < x.size(); i++)
        {

            if (x[i].get(GRB_DoubleAttr_X) == 0)
            {

                int obsCnt = getMemory(mpDB[i]);
                mpDB[i]->SetBadFlag();
                index++;
                totalObs += obsCnt;
            }
        }

        std::cout << " Finish Map Compression" << std::endl;
        std::cout << " Total memory of removed mapPoints : " << totalObs << std::endl;
        long unsigned int PointCloudNum_ = Map->MapPointsInMap();
        std::cout << "After Compression Point Num : " << PointCloudNum_ << std::endl;

        remainLandmark = PointCloudNum_;
        removedMemory = (double)totalObs * 1e-6;
    }
    catch (GRBException e)
    {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
} // Remove Landmark Score + Weight Q IQP

int Compression::removalKeyframe1()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    int removeKeyframeCnt = 0;
    int removeIdx = 0;
    int totalRemovedptsNum = 0;
    std::cout << " before remove Keyframe num : " << kfdb.size() << std::endl;
    for (size_t i = 0; i < kfdb.size(); i++)
    {

        if (i == 0)
            continue;
        if (kfdb[i - 1]->isBad())
            removeIdx++;
        else
            removeIdx = 0;

        std::vector<ORB_SLAM2::MapPoint *> kfMapts = kfdb[i]->GetMapPointMatches();
        int covisibilityCnt = 0;

        for (size_t j = 0; j < kfMapts.size(); j++)
        {

            if (!kfMapts[j])
                continue;
            if (!kfMapts[j]->isBad())
            {

                bool covisibilityLandmark = kfMapts[j]->IsInKeyFrame(kfdb[i - 1 - removeIdx]);
                if (covisibilityLandmark)
                    covisibilityCnt++;
            }
        }
        int lastFrameMp = kfdb[i - 1 - removeIdx]->GetMapPoints().size();
        int currFrameMp = kfdb[i]->GetMapPoints().size();
        double ratio = (double)covisibilityCnt / (double)currFrameMp;
        // std::cout << covisibilityCnt << std::endl;
        std::cout << ratio << std::endl;

        if (covisibilityCnt > 100)
        {
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

    while (true)
    {

        std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
        std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

        iterateKeyframeRemoval();

        std::cout << "...................................     iterate  ....   " << iterateNum << std::endl;

        // Eigen::VectorXd invScoreVec_(totalKeyframeNum);
        // for(int i = 0; i < invScoreVec_.size(); i++) invScoreVec_[i] = 1.0;

        // minMaxNormalize(&reprojectionErrAvg);
        // for(int i = 0; i < totalKeyframeNum; i++) KeyframeInvWeight[i] = 1.0 - (double)reprojectionErrInlier[i];
        for (int i = 0; i < totalKeyframeNum; i++)
            KeyframeInvWeight[i] = (double)reprojectionErrAvg[i];
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
        for (size_t j = 0; j < totalKeyframeNum; j++)
            sortIdx[j] = j;

        std::sort(sortIdx.begin(), sortIdx.end(), weightComp);
        long unsigned int sortArr = 0;
        int neightborIter = 0;
        bool reachRemoveKf = false;

        while (true)
        {

            int idx = sortIdx[sortArr] + 1;
            // int neighborKfNum = getNeighborKeyframeNum(kfdb[idx]);
            int neighborKfNum = 100;
            int neighborKfIdDist = getNeighborKeyframeIdDist(idx);
            double obs1Ratio = getKeyframeObs1Num(kfdb[idx]);
            // && obs1Ratio < obs1RatioThres
            // std::cout << "idid : " << idx << std::endl;
            // std::cout << "high value : " << obs1Ratio << std::endl;
            if (neighborKfNum > neighborKeyframeNumThres && neighborKfIdDist <= neighborKeyframeIdThres)
            {
                std::cout << " removed keyframe Id : " << kfdb[idx]->mnId << "     neighbor Keyframe Num : " << neighborKfNum << "   neighbor iter num : " << neightborIter << std::endl;
                std::cout << " Obs 1 Num Ratio : " << obs1Ratio << std::endl;
                std::cout << " id distant : " << neighborKfIdDist << std::endl;
                int currMemory = getMemory(kfdb[idx]);
                totalRemoveMemory += currMemory;
                kfdb[idx]->SetBadFlag();
                kfNewIds.erase(kfNewIds.begin() + idx);
                break;
            }
            else
            {
                sortArr++;
                neightborIter++;
                if (sortArr == totalKeyframeNum - 1)
                {
                    reachRemoveKf = true;
                    std::cout << " cannot remove keyframe anymore " << std::endl;
                    break;
                }
                continue;
            }
        }

        totalRemovedKeyframeNum++;
        iterateNum++;
        if (reachRemoveKf)
            break;
        if (kfCompressedRatio >= (double)(originalKeyframeNum - totalRemovedKeyframeNum) / (double)originalKeyframeNum)
            break;
    }
    std::cout << std::endl;
    // std::cout <<  " Total memory of removed mapPoints : " << totalRemoveMemory << std::endl;
    return totalRemoveMemory;
}

int Compression::removalKeyframe3()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    int totalRemoveMemory = 0;
    try
    {
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
        // Eigen::MatrixXd S = calculateKeyframeSimilarity(Map);
        SetObjectiveILPforKeyframe(x, keyframeScore, model);

        std::cout << " Add Constraint ... " << std::endl;
        // Add Constraint
        AddConstraintForKeyframe(Map, model, x, kfCompressedRatio, neighborKeyframeIdThres);

        std::cout << std::endl;

        // if (model.get(GRB_IntAttr_IsMIP) == 0) {
        //   throw GRBException("Model is not a MIP");
        // }

        std::cout << " Optimize model ... " << std::endl;
        // Optimize model
        // model.set(GRB_DoubleParam_IterationLimit, 100000);
        model.optimize();

        // int optimstatus = model.get(GRB_IntAttr_Status);
        // cout << "Optimization complete" << endl;
        // double objval = 0;
        // if (optimstatus == GRB_OPTIMAL) {
        //   objval = model.get(GRB_DoubleAttr_ObjVal);
        //   cout << "Optimal objective: " << objval << endl;
        // } else if (optimstatus == GRB_INF_OR_UNBD) {
        //   cout << "Model is infeasible or unbounded" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_INFEASIBLE) {
        //   cout << "Model is infeasible" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_UNBOUNDED) {
        //   cout << "Model is unbounded" << endl;
        //   return 0;
        // } else {
        //   cout << "Optimization was stopped with status = "
        //        << optimstatus << endl;
        //   return 0;
        // }

        model.set(GRB_IntParam_OutputFlag, 0);

        cout << endl;
        for (int k = 0; k < model.get(GRB_IntAttr_SolCount); ++k)
        {
            model.set(GRB_IntParam_SolutionNumber, k);
            double objn = model.get(GRB_DoubleAttr_PoolObjVal);

            cout << "Solution " << k << " has objective: " << objn << endl;
        }
        cout << endl;

        // std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        model.set(GRB_IntParam_OutputFlag, 1);

        /* Create a fixed model, turn off presolve and solve */

        GRBModel fixed = model.fixedModel();

        fixed.set(GRB_IntParam_Presolve, 0);

        fixed.optimize();

        int foptimstatus = fixed.get(GRB_IntAttr_Status);

        if (foptimstatus != GRB_OPTIMAL)
        {
            cerr << "Error: fixed model isn't optimal" << endl;
            return 0;
        }

        double fobjval = fixed.get(GRB_DoubleAttr_ObjVal);

        // if (fabs(fobjval - objval) > 1.0e-6 * (1.0 + fabs(objval))) {
        //   cerr << "Error: objective values are different" << endl;
        //   return 0;
        // }

        // int numvars = model.get(GRB_IntAttr_NumVars);
        // fvars = fixed.getVars();
        // for (int j = 0; j < numvars; j++) {
        //   GRBVar v = fvars[j];
        //   if (v.get(GRB_DoubleAttr_X) != 0.0) {
        //     cout << v.get(GRB_StringAttr_VarName) << " "
        //          << v.get(GRB_DoubleAttr_X) << endl;
        // std::cout << std::endl;
        //   }
        // }

        for (size_t i = 0; i < x.size(); i++)
        {

            if (x[i].get(GRB_DoubleAttr_X) == 0)
            {

                int currMemory = getMemory(kfdb[i + 1]);
                totalRemoveMemory += currMemory;
                kfdb[i + 1]->SetBadFlag();
            }
        }
    }
    catch (GRBException e)
    {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    return totalRemoveMemory;

} // remove keyframe by ILP method

int Compression::removalKeyframe4()
{

    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
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

    Eigen::MatrixXd S = calculateKeyframeSimilarity(Map);
    // SetObjectiveILPforKeyframe(x, S, model);

    // Set Objective
    std::cout << " Set Objective ... " << std::endl;
    SetObjectiveforKeyframeMapCube(x, visibilityMatrix, originalCubeVector, cubeIds, cubeIdsSet, S, model);

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

    for (size_t i = 0; i < x.size(); i++)
    {

        if (x[i].get(GRB_DoubleAttr_X) == 0)
        {

            int currMemory = getMemory(kfdb[i + 1]);
            totalRemoveMemory += currMemory;
            kfdb[i + 1]->SetBadFlag();
        }
    }

    return totalRemoveMemory;
}

int Compression::removalKeyframeSimilarity(
    const double a,
    const double b,
    const double c,
    const double d)
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    int totalRemoveMemory = 0;
    try
    {
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

        std::cout << " Calculate Similarity ... " << std::endl;
        Eigen::MatrixXd S = calculateKeyframeSimilarity(Map);

        // std::cout << " Calculate Landmark and Keyframe Score ... " << std::endl;
        // getLandmarkScore(a, b, c, d);
        // getKeyframeScore();

        SetObjectiveSimilarityforKeyframe(x, S, model);
        // SetObjectiveILPforKeyframe(x, keyframeScore, model);
        
        std::cout << " Add Constraint ... " << std::endl;
        // Add Constraint
        AddConstraintForKeyframe(Map, model, x, kfCompressedRatio, neighborKeyframeIdThres);

        std::cout << std::endl;

        // if (model.get(GRB_IntAttr_IsMIP) == 0) {
        //   throw GRBException("Model is not a MIP");
        // }

        std::cout << " Optimize model ... " << std::endl;
        // Optimize model
        model.set(GRB_DoubleParam_TimeLimit, 300);
        model.set(GRB_IntParam_NonConvex, 2);
        model.optimize();

        // int optimstatus = model.get(GRB_IntAttr_Status);
        // cout << "Optimization complete" << endl;
        // double objval = 0;
        // if (optimstatus == GRB_OPTIMAL) {
        //   objval = model.get(GRB_DoubleAttr_ObjVal);
        //   cout << "Optimal objective: " << objval << endl;
        // } else if (optimstatus == GRB_INF_OR_UNBD) {
        //   cout << "Model is infeasible or unbounded" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_INFEASIBLE) {
        //   cout << "Model is infeasible" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_UNBOUNDED) {
        //   cout << "Model is unbounded" << endl;
        //   return 0;
        // } else {
        //   cout << "Optimization was stopped with status = "
        //        << optimstatus << endl;
        //   return 0;
        // }

        model.set(GRB_IntParam_OutputFlag, 0);

        cout << endl;
        for (int k = 0; k < model.get(GRB_IntAttr_SolCount); ++k)
        {
            model.set(GRB_IntParam_SolutionNumber, k);
            double objn = model.get(GRB_DoubleAttr_PoolObjVal);

            cout << "Solution " << k << " has objective: " << objn << endl;
        }
        cout << endl;

        // std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        model.set(GRB_IntParam_OutputFlag, 1);

        /* Create a fixed model, turn off presolve and solve */

        GRBModel fixed = model.fixedModel();

        fixed.set(GRB_IntParam_Presolve, 0);

        fixed.optimize();

        int foptimstatus = fixed.get(GRB_IntAttr_Status);

        if (foptimstatus != GRB_OPTIMAL)
        {
            cerr << "Error: fixed model isn't optimal" << endl;
            return 0;
        }

        double fobjval = fixed.get(GRB_DoubleAttr_ObjVal);

        // if (fabs(fobjval - objval) > 1.0e-6 * (1.0 + fabs(objval))) {
        //   cerr << "Error: objective values are different" << endl;
        //   return 0;
        // }

        // int numvars = model.get(GRB_IntAttr_NumVars);
        // fvars = fixed.getVars();
        // for (int j = 0; j < numvars; j++) {
        //   GRBVar v = fvars[j];
        //   if (v.get(GRB_DoubleAttr_X) != 0.0) {
        //     cout << v.get(GRB_StringAttr_VarName) << " "
        //          << v.get(GRB_DoubleAttr_X) << endl;
        // std::cout << std::endl;
        //   }
        // }

        for (size_t i = 0; i < x.size(); i++)
        {

            if (x[i].get(GRB_DoubleAttr_X) == 0)
            {

                int currMemory = getMemory(kfdb[i + 1]);
                totalRemoveMemory += currMemory;
                kfdb[i + 1]->SetBadFlag();
            }
        }
    }
    catch (GRBException e)
    {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    return totalRemoveMemory;

} // remove keyframe by Similarity method

int Compression::removalKeyframeIQP(
    const double a,
    const double b,
    const double c,
    const double d)
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    int totalRemoveMemory = 0;
    try
    {
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

        std::cout << " Calculate Similarity ... " << std::endl;
        Eigen::MatrixXd S = calculateKeyframeSimilarity(Map);

        std::cout << " Calculate Landmark and Keyframe Score ... " << std::endl;
        getLandmarkScore(a, b, c, d);
        getKeyframeScore();

        SetObjectiveIQPforKeyframe(x, S, keyframeScore, model);
        // SetObjectiveILPforKeyframe(x, keyframeScore, model);
        
        std::cout << " Add Constraint ... " << std::endl;
        // Add Constraint
        AddConstraintForKeyframe(Map, model, x, kfCompressedRatio, neighborKeyframeIdThres);

        std::cout << std::endl;

        // if (model.get(GRB_IntAttr_IsMIP) == 0) {
        //   throw GRBException("Model is not a MIP");
        // }

        std::cout << " Optimize model ... " << std::endl;
        // Optimize model
        model.set(GRB_DoubleParam_TimeLimit, 50);
        model.optimize();

        // int optimstatus = model.get(GRB_IntAttr_Status);
        // cout << "Optimization complete" << endl;
        // double objval = 0;
        // if (optimstatus == GRB_OPTIMAL) {
        //   objval = model.get(GRB_DoubleAttr_ObjVal);
        //   cout << "Optimal objective: " << objval << endl;
        // } else if (optimstatus == GRB_INF_OR_UNBD) {
        //   cout << "Model is infeasible or unbounded" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_INFEASIBLE) {
        //   cout << "Model is infeasible" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_UNBOUNDED) {
        //   cout << "Model is unbounded" << endl;
        //   return 0;
        // } else {
        //   cout << "Optimization was stopped with status = "
        //        << optimstatus << endl;
        //   return 0;
        // }

        model.set(GRB_IntParam_OutputFlag, 0);

        cout << endl;
        for (int k = 0; k < model.get(GRB_IntAttr_SolCount); ++k)
        {
            model.set(GRB_IntParam_SolutionNumber, k);
            double objn = model.get(GRB_DoubleAttr_PoolObjVal);

            cout << "Solution " << k << " has objective: " << objn << endl;
        }
        cout << endl;

        // std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        model.set(GRB_IntParam_OutputFlag, 1);

        /* Create a fixed model, turn off presolve and solve */

        GRBModel fixed = model.fixedModel();

        fixed.set(GRB_IntParam_Presolve, 0);

        fixed.optimize();

        int foptimstatus = fixed.get(GRB_IntAttr_Status);

        if (foptimstatus != GRB_OPTIMAL)
        {
            cerr << "Error: fixed model isn't optimal" << endl;
            return 0;
        }

        double fobjval = fixed.get(GRB_DoubleAttr_ObjVal);

        // if (fabs(fobjval - objval) > 1.0e-6 * (1.0 + fabs(objval))) {
        //   cerr << "Error: objective values are different" << endl;
        //   return 0;
        // }

        // int numvars = model.get(GRB_IntAttr_NumVars);
        // fvars = fixed.getVars();
        // for (int j = 0; j < numvars; j++) {
        //   GRBVar v = fvars[j];
        //   if (v.get(GRB_DoubleAttr_X) != 0.0) {
        //     cout << v.get(GRB_StringAttr_VarName) << " "
        //          << v.get(GRB_DoubleAttr_X) << endl;
        // std::cout << std::endl;
        //   }
        // }

        for (size_t i = 0; i < x.size(); i++)
        {

            if (x[i].get(GRB_DoubleAttr_X) == 0)
            {

                int currMemory = getMemory(kfdb[i + 1]);
                totalRemoveMemory += currMemory;
                kfdb[i + 1]->SetBadFlag();
            }
        }
    }
    catch (GRBException e)
    {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    return totalRemoveMemory;

} // remove keyframe by IQP method

int Compression::removalKeyframeILP(
    const double a,
    const double b,
    const double c,
    const double d)
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    int totalRemoveMemory = 0;
    try
    {
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

        // std::cout << " Calculate Similarity ... " << std::endl;
        // Eigen::MatrixXd S = calculateKeyframeSimilarity(Map);

        std::cout << " Calculate Landmark and Keyframe Score ... " << std::endl;
        getLandmarkScore(a, b, c, d);
        getKeyframeScore();

        // SetObjectiveIQPforKeyframe(x, S, keyframeScore, model);
        SetObjectiveILPforKeyframe(x, keyframeScore, model);
        
        std::cout << " Add Constraint ... " << std::endl;
        // Add Constraint
        AddConstraintForKeyframe(Map, model, x, kfCompressedRatio, neighborKeyframeIdThres);

        std::cout << std::endl;

        // if (model.get(GRB_IntAttr_IsMIP) == 0) {
        //   throw GRBException("Model is not a MIP");
        // }

        std::cout << " Optimize model ... " << std::endl;
        // Optimize model
        model.set(GRB_DoubleParam_TimeLimit, 300);
        model.optimize();

        // int optimstatus = model.get(GRB_IntAttr_Status);
        // cout << "Optimization complete" << endl;
        // double objval = 0;
        // if (optimstatus == GRB_OPTIMAL) {
        //   objval = model.get(GRB_DoubleAttr_ObjVal);
        //   cout << "Optimal objective: " << objval << endl;
        // } else if (optimstatus == GRB_INF_OR_UNBD) {
        //   cout << "Model is infeasible or unbounded" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_INFEASIBLE) {
        //   cout << "Model is infeasible" << endl;
        //   return 0;
        // } else if (optimstatus == GRB_UNBOUNDED) {
        //   cout << "Model is unbounded" << endl;
        //   return 0;
        // } else {
        //   cout << "Optimization was stopped with status = "
        //        << optimstatus << endl;
        //   return 0;
        // }

        model.set(GRB_IntParam_OutputFlag, 0);

        cout << endl;
        for (int k = 0; k < model.get(GRB_IntAttr_SolCount); ++k)
        {
            model.set(GRB_IntParam_SolutionNumber, k);
            double objn = model.get(GRB_DoubleAttr_PoolObjVal);

            cout << "Solution " << k << " has objective: " << objn << endl;
        }
        cout << endl;

        // std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        model.set(GRB_IntParam_OutputFlag, 1);

        /* Create a fixed model, turn off presolve and solve */

        GRBModel fixed = model.fixedModel();

        fixed.set(GRB_IntParam_Presolve, 0);

        fixed.optimize();

        int foptimstatus = fixed.get(GRB_IntAttr_Status);

        if (foptimstatus != GRB_OPTIMAL)
        {
            cerr << "Error: fixed model isn't optimal" << endl;
            return 0;
        }

        double fobjval = fixed.get(GRB_DoubleAttr_ObjVal);

        // if (fabs(fobjval - objval) > 1.0e-6 * (1.0 + fabs(objval))) {
        //   cerr << "Error: objective values are different" << endl;
        //   return 0;
        // }

        // int numvars = model.get(GRB_IntAttr_NumVars);
        // fvars = fixed.getVars();
        // for (int j = 0; j < numvars; j++) {
        //   GRBVar v = fvars[j];
        //   if (v.get(GRB_DoubleAttr_X) != 0.0) {
        //     cout << v.get(GRB_StringAttr_VarName) << " "
        //          << v.get(GRB_DoubleAttr_X) << endl;
        // std::cout << std::endl;
        //   }
        // }

        for (size_t i = 0; i < x.size(); i++)
        {

            if (x[i].get(GRB_DoubleAttr_X) == 0)
            {

                int currMemory = getMemory(kfdb[i + 1]);
                totalRemoveMemory += currMemory;
                kfdb[i + 1]->SetBadFlag();
            }
        }
    }
    catch (GRBException e)
    {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    return totalRemoveMemory;

} // remove keyframe by ILP method

void Compression::selectRandomNum()
{

    std::cout << "abc" << std::endl;

    int oriKeyframeNum = Map->GetAllKeyFrames().size();
    std::vector<int> remainedKfsId(oriKeyframeNum);
    for(int i = 0; i < oriKeyframeNum; i++) remainedKfsId[i] = i;
    // std::vector<int> selectedkfId; 
    int selectedSize = 0;
    
    storage.resize(compressionRatios.size());
    std::cout << "Compression size : " << compressionRatios.size() << " " << compressionRatios.capacity()<<std::endl;
    for(size_t i = 0; i < compressionRatios.size(); i++) {
        std::cout << (compressionRatios)[i] << " ";
    }
    // std::random_device rd;
    std::mt19937 gen;
    std::cout << std::endl;
    int var = 0;
    for(size_t i = 0; i < compressionRatios.size(); i++){
        
        std::cout << "Compression size : " << compressionRatios.size() << std::endl;
        std::cout << "abc   " << i <<  std::endl;
        std::cout << (int)(oriKeyframeNum * (1.0 - (compressionRatios)[i])) << " " << (1.0 - (compressionRatios)[i]) << " " << (compressionRatios).at(i) << " " << selectedSize << std::endl;
        int randomSelectNum = (int)(oriKeyframeNum * (1.0 - (compressionRatios)[i])) - selectedSize;
        
        // random num

        std::uniform_int_distribution<int> dis(1, remainedKfsId.size() - 1);

        std::cout << randomSelectNum << " " << remainedKfsId.size() << std::endl;
        std::vector<int> idxs(randomSelectNum);
        std::vector<int> newRemovingkfId;
        for(int j = 0; j < randomSelectNum; j++){
            
            // int a = 1;
            while(true){
                var = dis(gen);
                // var = randomNum(1, remainedKfsId.size() - 1, clock());
                // int var = j;
                // std::cout << var << "   ";
                auto it = std::find(idxs.begin(), idxs.end(), var);
                if(it == idxs.end()){
                    std::cout << var << " ";
                    idxs[j] = var;
                    break;
                }
                // a++; 
            }    
        }
        
        selectedSize += idxs.size();        
        
        newRemovingkfId.resize(idxs.size());    
        for(size_t j = 0; j < idxs.size(); j++) {
            // std::cout << idxs[j] << " ";
            newRemovingkfId[j] = remainedKfsId[idxs[j]];
        }
        
        int eraseIdx = 0;
        for(size_t j = 0; j < idxs.size(); j++){
            // std::cout << "remainkfs size : " << remainedKfsId.size() << " " << "erase idx : " << idxs[j] << std::endl;
            remainedKfsId.erase(remainedKfsId.begin() + idxs[j] - eraseIdx);
            eraseIdx++;
        }
        
        storage[i] = newRemovingkfId;
        // for debug
        // for(size_t i = 0; i < storage.size(); i++){
        //     for(size_t j = 0; j < storage[i].size(); j++) std::cout << storage[i][j] <<" ";
        //     std::cout << std::endl; 
        // }
        std::cout << std::endl;
    }
    std::cout << "abcd" << std::endl;
}

int Compression::removeRandomKeyframe(int storageIdx, std::vector<ORB_SLAM2::KeyFrame *> kfdb)
{

    int totalRandomRemovedMemory = 0;
    for(size_t i = 0; i < storage[storageIdx].size(); i++){

        int currMemory = getMemory(kfdb[storage[storageIdx][i]]);
        totalRandomRemovedMemory += currMemory;
        kfdb[storage[storageIdx][i]]->SetBadFlag();
    }

    return totalRandomRemovedMemory;
}       

int Compression::randomNum(int low, int high, unsigned int seed)
{
    // std::uniform_int_distribution<int> dis(low, high);
    // return dis(gen);
    
    // srand(seed);
    // int f = rand();

    // return f + rand() * (high - low);
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
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    for (size_t i = 0; i < totalKeyframeNum; i++)
    {
        kfObsNums[i] = getObservation(kfdb[i + 1]);
        kfMpNums[i] = getKeyframeMap(kfdb[i + 1]).size();
    }

    TotalObservation = getTotalObservation();
    TotalMapPoints = getTotalMapPoints();

    for (size_t i = 0; i < totalKeyframeNum; i++)
    {
        kfObsNumsRatio[i] = kfObsNums[i] / TotalObservation;
        kfMpNumsRatio[i] = (double)kfMpNums[i] / (double)TotalMapPoints;
    }

    std::vector<int> sortObsIdx(totalKeyframeNum);
    std::vector<int> sortMpIdx(totalKeyframeNum);
    for (size_t i = 0; i < totalKeyframeNum; i++)
    {
        sortObsIdx[i] = i;
        sortMpIdx[i] = i;
    }
    std::sort(sortObsIdx.begin(), sortObsIdx.end(), obsScoreComp);
    std::sort(sortMpIdx.begin(), sortMpIdx.end(), mpNumScoreComp);

    for (size_t i = 0; i < totalKeyframeNum; i++)
    {

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

    for (size_t i = 0; i < totalKeyframeNum; i++)
    {
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
    for (size_t i = 0; i < kfObsNums.size(); i++)
    {
        double kfObsNum = kfObsNums[i];
        total += kfObsNum;
    }
    return total;
}

int Compression::getTotalMapPoints()
{
    int total = 0;
    for (size_t i = 0; i < kfMpNums.size(); i++)
    {
        int mpNum = kfMpNums[i];
        total += mpNum;
    }
    return total;
}

double Compression::getObservation(ORB_SLAM2::KeyFrame *kf)
{
    // std::set<ORB_SLAM2::MapPoint*> kfmp = kf->GetMapPoints();

    std::vector<ORB_SLAM2::MapPoint *> kfmp = getKeyframeMap(kf);
    double total = 0;

    // for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfmp.begin(); iter != kfmp.end(); iter++){
    //     double mpObsNum = (double)getObservation(*iter);
    //     total += mpObsNum;
    // }

    for (size_t i = 0; i < kfmp.size(); i++)
    {
        double mpObsNum = (double)getObservation(kfmp[i]);
        // std::cout << i << "   " << mpObsNum << std::endl;
        total += mpObsNum;
    }

    return total / (double)kfmp.size();
}

int Compression::getObservation(ORB_SLAM2::MapPoint *mp)
{
    int obsCnt = mp->mObservations.size();
    return obsCnt;
}

void Compression::getKeyframeSimilarityMatrix()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    for (size_t i = 1; i < kfdb.size(); i++)
    {

        int lastKeyframeMpNum = getKeyframeMap(kfdb[i]).size();
        for (size_t j = 1; j < kfdb.size(); j++)
        {

            // getRelativePose(kfdb[i], kfdb[j]);
            // // std::cout << relPoseErr[0] << "    " << ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) << std::endl;
            // if(relPoseErr[0] < 1.0){

            int covisibilityMpNum = getCovisibilityMpNum(kfdb[i], kfdb[j]);
            similarityMatrix(i - 1, j - 1) = (double)covisibilityMpNum / (double)lastKeyframeMpNum;
            if(lastKeyframeMpNum < 50.0) similarityMatrix(i - 1, j - 1) = 1.0;
            
            // }
            // else
            //     similarityMatrix(i - 1, j - 1) = 0.0;
        }
    }
    // std::cout << similarityMatrix<< std::endl;
}

void Compression::getAllKeyframeReprojErr()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    for (size_t i = 1; i < kfdb.size(); i++)
    {
        reprojectionErrAvg[i - 1] = getreprojectionErrAvg(kfdb[i]);
        reprojectionErrInlier[i - 1] = inlierRatioInKeyframe;
        // std::cout << std::endl;
    }
}

float Compression::getreprojectionErrAvg(ORB_SLAM2::KeyFrame *kf)
{
    float reprojErr = 0.0;
    // std::set<ORB_SLAM2::MapPoint*> kfMapPoints = kf->GetMapPoints();
    // test
    std::vector<ORB_SLAM2::MapPoint *> kfMapPoints = getKeyframeMap(kf);

    Eigen::Matrix4Xf worldPoints(4, kfMapPoints.size());
    Eigen::Matrix3Xf imgPoints(3, kfMapPoints.size());
    Eigen::Matrix3Xf reprojectPoints(3, kfMapPoints.size());
    Eigen::Matrix<float, 3, 4> proj = ORB_SLAM2::Converter::toProj34(kf->GetPose());
    Eigen::Matrix3f K_;
    K_ << kf->fx, 0, kf->cx,
        0, kf->fy, kf->cy,
        0, 0, 1;

    for (size_t i = 0; i < kfMapPoints.size(); i++)
    {

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

    for (int i = 0; i < reprojectPoints.cols(); i++)
    {
        reprojectPoints(0, i) /= reprojectPoints(2, i);
        reprojectPoints(1, i) /= reprojectPoints(2, i);
        reprojectPoints(2, i) /= reprojectPoints(2, i);
        // std::cout << reprojectPoints(0, i) << " " << reprojectPoints(1, i) << " " << reprojectPoints(2, i) << std::endl;
    }
    reprojectPoints = K_ * reprojectPoints;

    int inlier = 0;
    std::vector<float> ReprojectErr(worldPoints.cols());
    for (int i = 0; i < worldPoints.cols(); i++)
    {
        ReprojectErr[i] = std::sqrt((imgPoints(0, i) - reprojectPoints(0, i)) *
                                        (imgPoints(0, i) - reprojectPoints(0, i)) +
                                    (imgPoints(1, i) - reprojectPoints(1, i)) *
                                        (imgPoints(1, i) - reprojectPoints(1, i)));
        reprojErr += ReprojectErr[i];
        if (ReprojectErr[i] < 1.5)
            inlier++;
        // std::cout << ReprojectErr[i] << " ";
    }
    inlierRatioInKeyframe = (float)inlier / (float)kfMapPoints.size();
    // std::cout << reprojErr/(float)kfMapPoints.size() << std::endl;
    return reprojErr / (float)kfMapPoints.size();
}

float Compression::getreprojectionErrAvg(ORB_SLAM2::MapPoint *mp)
{
    float reprojErr = 0.0f;

    std::map<ORB_SLAM2::KeyFrame *, size_t> Observations = mp->GetObservations();

    Eigen::Matrix4Xf worldPoints(4, 1);
    Eigen::Matrix3Xf imgPoints(3, 1);
    Eigen::Matrix3Xf reprojectPoints(3, 1);

    cv::Mat mapPoint = mp->GetWorldPos();
    worldPoints(0, 0) = mapPoint.at<float>(0, 0);
    worldPoints(1, 0) = mapPoint.at<float>(1, 0);
    worldPoints(2, 0) = mapPoint.at<float>(2, 0);
    worldPoints(3, 0) = 1.0f;

    // std::cout << "obser size : " << Observations.size() << std::endl;
    for (auto iter = Observations.begin(); iter != Observations.end(); iter++)
    {

        ORB_SLAM2::KeyFrame *kf = iter->first;
        Eigen::Matrix<float, 3, 4> proj = ORB_SLAM2::Converter::toProj34(kf->GetPose());
        Eigen::Matrix3f K_;
        K_ << kf->fx, 0, kf->cx,
            0, kf->fy, kf->cy,
            0, 0, 1;

        int idx = mp->GetIndexInKeyFrame(kf);
        imgPoints(0, 0) = kf->mvKeys[idx].pt.x;
        imgPoints(1, 0) = kf->mvKeys[idx].pt.y;
        imgPoints(2, 0) = 1.0f;

        // get reprojection Error
        reprojectPoints = proj * worldPoints;

        reprojectPoints(0, 0) /= reprojectPoints(2, 0);
        reprojectPoints(1, 0) /= reprojectPoints(2, 0);
        reprojectPoints(2, 0) /= reprojectPoints(2, 0);
        reprojectPoints = K_ * reprojectPoints;
        reprojErr += std::sqrt((imgPoints(0, 0) - reprojectPoints(0, 0)) *
                                   (imgPoints(0, 0) - reprojectPoints(0, 0)) +
                               (imgPoints(1, 0) - reprojectPoints(1, 0)) *
                                   (imgPoints(1, 0) - reprojectPoints(1, 0)));
    }

    return reprojErr / (float)Observations.size();
}

int Compression::getCovisibilityMpNum(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2)
{
    // std::set<ORB_SLAM2::MapPoint*> kfMapPoints = kf1->GetMapPoints();
    std::vector<ORB_SLAM2::MapPoint *> kfMapPoints = getKeyframeMap(kf1);

    int covisibilityCnt = 0;
    // for(std::set<ORB_SLAM2::MapPoint*>::iterator iter = kfMapPoints.begin(); iter!=kfMapPoints.end(); iter++){
    //     bool covisibilityLandmark = (*iter)->IsInKeyFrame(kf2);
    //     if(covisibilityLandmark) covisibilityCnt++;
    // }

    for (size_t i = 0; i < kfMapPoints.size(); i++)
    {
        bool covisibilityLandmark = (kfMapPoints[i])->IsInKeyFrame(kf2);
        if (covisibilityLandmark)
            covisibilityCnt++;
    }

    return covisibilityCnt;
}

void Compression::getRelativePose(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2)
{
    cv::Mat proj1 = kf1->GetPose();
    cv::Mat proj2 = kf2->GetPose();
    Vector6d Pose1 = ORB_SLAM2::Converter::Proj2Vec6(proj1);
    Vector6d Pose2 = ORB_SLAM2::Converter::Proj2Vec6(proj2);

    RMSError(Pose1, Pose2, &relPoseErr[0]);
}

float Compression::getRelativeTranslation(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2)
{
    cv::Mat proj1 = kf1->GetPose();
    cv::Mat proj2 = kf2->GetPose();
    Eigen::Vector3f Pose1 = ORB_SLAM2::Converter::toVec3f(proj1);
    Eigen::Vector3f Pose2 = ORB_SLAM2::Converter::toVec3f(proj2);

    float traslation = 0;
    traslation = std::sqrt((Pose1.x() - Pose2.x()) * (Pose1.x() - Pose2.x()) + (Pose1.y() - Pose2.y()) * (Pose1.y() - Pose2.y()) + (Pose1.z() - Pose2.z()) * (Pose1.z() - Pose2.z()));

    return traslation;
}

float Compression::getRelativeRotation(ORB_SLAM2::KeyFrame *kf1, ORB_SLAM2::KeyFrame *kf2, Eigen::Vector3f landmarkPos)
{
    cv::Mat proj1 = kf1->GetPose();
    cv::Mat proj2 = kf2->GetPose();
    Eigen::Vector3f Pose1 = ORB_SLAM2::Converter::toVec3f(proj1);
    Eigen::Vector3f Pose2 = ORB_SLAM2::Converter::toVec3f(proj2);

    Eigen::Vector3f ray1 = (landmarkPos - Pose1) / (landmarkPos - Pose1).norm();
    Eigen::Vector3f ray2 = (landmarkPos - Pose2) / (landmarkPos - Pose1).norm();

    float rad = std::acoshf(ray1.dot(ray2));

    return rad;
}

int Compression::getNeighborKeyframeNum(ORB_SLAM2::KeyFrame *kf)
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    int neighborKfNum = 0;

    for (size_t i = 1; i < kfdb.size(); i++)
    {

        getRelativePose(kf, kfdb[i]);
        if (relPoseErr[0] < neighborKeyframeTranslationThres)
        { // && ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) < neighborKeyframeRotationThres) neighborKfNum++;
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
    if (kfNewIds[idx] == kfNewIds.back())
        idDist = maxNewid - kfNewIds[idx - 1];
    else
    {
        idDist = kfNewIds[idx + 1] - kfNewIds[idx - 1];
    }

    return idDist;
}

void Compression::setInitial(double kfCompressedRatio_)
{
    originalKeyframeNum = (int)Map->GetAllKeyFrames().size();
    originalMapPointNum = (int)Map->GetAllMapPoints().size();

    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    kfNewIds.resize(originalKeyframeNum);

    for (size_t i = 0; i < kfdb.size(); i++)
    {

        kfdb[i]->newId = i;
        kfNewIds[i] = i;
        if (i > 0)
        {
            getRelativePose(kfdb[i - 1], kfdb[i]);
            neighborKeyframeTranslationThres += relPoseErr[0];
        }
    }
    maxNewid = kfdb.size() - 1;
    kfCompressedRatio = kfCompressedRatio_;
    neighborKeyframeIdThres = ((int)1 / kfCompressedRatio) * 2 + 1;
}

void Compression::printKeyframeInfo(const std::string &file)
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

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
    std::cout << "KeyframeId     "
              << "ObsNum     "
              << "ObsNumRatio     "
              << "ObsRemoveRank     "
              << "MpNum     "
              << "MpNumRatio     "
              << "MpNumRemoveRank     " << std::endl;
    for (size_t i = 1; i < kfdb.size(); i++)
    {
        std::cout << kfdb[i]->mnId << "     " << kfObsNums[i - 1] << "     " << kfObsNumsRatio[i - 1] << "     " << kfObsRank[i - 1] << "     "
                  << kfMpNums[i - 1] << "     " << kfMpNumsRatio[i - 1] << "     " << kfMpNumRank[i - 1] << "     " << KeyframeInvWeight[i - 1] << "    " << reprojectionErrAvg[i - 1] << "   " << reprojectionErrInlier[i - 1] << std::endl;
        saveFile << kfdb[i]->mnId << " " << kfObsNums[i - 1] << " " << kfMpNums[i - 1] << " " << reprojectionErrInlier[i - 1] << " "
                 << reprojectionErrAvg[i - 1] << " " << KeyframeInvWeight[i - 1] << std::endl;
    }
}

std::vector<ORB_SLAM2::MapPoint *> Compression::getKeyframeMap(ORB_SLAM2::KeyFrame *kf)
{
    std::vector<ORB_SLAM2::MapPoint *> kfMap;
    std::set<ORB_SLAM2::MapPoint *> mp = kf->GetMapPoints();
    for (std::set<ORB_SLAM2::MapPoint *>::iterator iter = mp.begin(); iter != mp.end(); iter++)
    {
        bool Inkeyframe = (*iter)->IsInKeyFrame(kf);
        if (Inkeyframe)
        {
            kfMap.push_back(*iter);
        }
    }
    return kfMap;
}

double Compression::getKeyframeObs1Num(ORB_SLAM2::KeyFrame *kf)
{
    int obs1Num = 0;
    std::vector<ORB_SLAM2::MapPoint *> kfmpDb = getKeyframeMap(kf);
    for (size_t i = 0; i < kfmpDb.size(); i++)
    {
        int obsNum = kfmpDb[i]->GetObservations().size();
        if (obsNum == 1)
            obs1Num++;
    }
    return (double)obs1Num / (double)kfmpDb.size();
}

void Compression::getAllObs1Ratio()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    for (size_t i = 1; i < kfdb.size(); i++)
    {

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
    std::vector<int> x(visibilityMatrix.cols(), 1);              // Keyframe Num

    std::cout << "Caculate observation vector ... " << std::endl;
    for (int i = 0; i < visibilityMatrix.rows(); i++)
    {

        int obs = 0;
        for (int j = 0; j < visibilityMatrix.cols(); j++)
        {
            obs += visibilityMatrix(i, j) * x[j];
        }
        if (obs == 0)
            observationVector[i] = 0;
        else
            observationVector[i] = 1;
    }

    // Caculate original MapPoint Num in Small Cube
    std::cout << "Caculate originalCubeVector ... " << std::endl;
    for (size_t i = 0; i < cubeIds.size(); i++)
    {

        int cubeid_ = cubeIds[i];
        int idx = getSetIndex(cubeIdsSet, cubeid_);

        if (idx == -1)
        {

            std::cout << "not found,,,?? " << std::endl;
        }
        else
        {
            originalCubeVector[idx] += 1 * observationVector[i];
        }
    }

    // delete smaller than thres
    std::cout << "delete smaller than thres ... " << std::endl;
    int eraseIdx = 0;
    int totalNum = originalCubeVector.size();
    for (size_t i = 0; i < totalNum; i++)
    {
        if (originalCubeVector[i - eraseIdx] < 5)
        {

            originalCubeVector.erase(originalCubeVector.begin() + i - eraseIdx);
            eraseIdx++;
        }
    }
}

void Compression::getBigCube(std::vector<ORB_SLAM2::MapPoint *> mpDb,
                             Eigen::Vector3d *minPoint,
                             Eigen::Vector3d *maxPoint)
{
    for (size_t i = 0; i < mpDb.size(); i++)
    {

        cv::Mat pointPos = mpDb[i]->GetWorldPos();
        Eigen::Vector3d pos;
        pos << (double)pointPos.at<float>(0, 0),
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
    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
    Eigen::Vector3d minp(DBL_MAX, DBL_MAX, DBL_MAX), maxp(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    getBigCube(mpDB, &minp, &maxp);

    minPoint.x() = (minp.x() >= 0) ? std::ceil(minp.x()) : std::floor(minp.x());
    minPoint.y() = (minp.y() >= 0) ? std::ceil(minp.y()) : std::floor(minp.y());
    minPoint.z() = (minp.z() >= 0) ? std::ceil(minp.z()) : std::floor(minp.z());

    maxPoint.x() = (maxp.x() >= 0) ? std::ceil(maxp.x()) : std::floor(maxp.x());
    maxPoint.y() = (maxp.y() >= 0) ? std::ceil(maxp.y()) : std::floor(maxp.y());
    maxPoint.z() = (maxp.z() >= 0) ? std::ceil(maxp.z()) : std::floor(maxp.z());

    // small Cube Num
    smallCubeXNum = maxPoint.x() - minPoint.x();
    smallCubeYNum = maxPoint.y() - minPoint.y();
    smallCubeZNum = maxPoint.z() - minPoint.z();
    std::cout << "SmallCubeNum    : " << smallCubeXNum << "  " << smallCubeYNum << "  " << smallCubeZNum << std::endl;
    totalSmallCubeNum = smallCubeXNum * smallCubeYNum * smallCubeZNum;
    std::cout << "totalSmallCubeNum    : " << totalSmallCubeNum << std::endl;
}

int Compression::getCubeId(ORB_SLAM2::MapPoint *mp)
{
    int cubeId = 0;

    cv::Mat pointPos = mp->GetWorldPos();
    Eigen::Vector3d pos;
    pos << (double)pointPos.at<float>(0, 0),
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
    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
    cubeIds.resize(mpDB.size());
    for (size_t i = 0; i < mpDB.size(); i++)
    {
        int cubeId = getCubeId(mpDB[i]);
        cubeIds[i] = cubeId;
    }

    for (auto i : cubeIds)
        cubeIdsSet.insert(i);

    std::cout << "cube Num : " << cubeIdsSet.size() << std::endl;
    totalSmallCubeNum = cubeIdsSet.size();
}

void Compression::getVisibilityMatrix()
{
    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    visibilityMatrix.resize(originalMapPointNum, originalKeyframeNum - 1);
    visibilityMatrix.setZero();

    for (size_t i = 0; i < mpDB.size(); i++)
    {
        for (size_t j = 1; j < kfdb.size(); j++)
        {
            bool isInKeyframe = mpDB[i]->IsInKeyFrame(kfdb[j]);
            if (isInKeyframe)
            {
                visibilityMatrix(i, j - 1) = 1;
            }
        }
    }
}

float Compression::getMaxTrackDistance(ORB_SLAM2::MapPoint *mp)
{
    float maxDist = FLT_MIN;
    std::map<ORB_SLAM2::KeyFrame *, size_t> Observations = mp->GetObservations();
    if (Observations.size() < 2)
        return 0.0f;
    else
    {
        for (auto iter1 = Observations.begin(); iter1 != Observations.end(); iter1++)
        {
            for (auto iter2 = Observations.begin(); iter2 != Observations.end(); iter2++)
            {
                ORB_SLAM2::KeyFrame *kf1 = iter1->first;
                ORB_SLAM2::KeyFrame *kf2 = iter2->first;
                float dist = getRelativeTranslation(kf1, kf2);
                if (maxDist < dist)
                {
                    maxDist = dist;
                }
            }
        }

        return maxDist;
    }
}

float Compression::getMaxAngle(ORB_SLAM2::MapPoint *mp)
{
    float maxAngle = FLT_MIN;

    std::map<ORB_SLAM2::KeyFrame *, size_t> Observations = mp->GetObservations();

    cv::Mat world3dPoint = mp->GetWorldPos();
    Eigen::Vector3f world3dPoint_;
    world3dPoint_ << world3dPoint.at<float>(0, 0), world3dPoint.at<float>(1, 0), world3dPoint.at<float>(2, 0);

    if (Observations.size() < 2)
        return 0.0f;
    else
    {

        for (auto iter1 = Observations.begin(); iter1 != Observations.end(); iter1++)
        {

            for (auto iter2 = Observations.begin(); iter2 != Observations.end(); iter2++)
            {

                ORB_SLAM2::KeyFrame *kf1 = iter1->first;
                ORB_SLAM2::KeyFrame *kf2 = iter2->first;

                float angle = getRelativeRotation(kf1, kf2, world3dPoint_);
                if (maxAngle < angle)
                {
                    maxAngle = angle;
                }
            }
        }

        return maxAngle;
    }
}

void Compression::getLandmarkScore(
    const double a,
    const double b,
    const double c,
    const double d)
{
    calculateVariousScore();
    // TODO
    // z-score normalize
    // min-max normalize
    std::cout << "get normalize" << std::endl;
    std::vector<float> obsNum_ = minMaxNormalize(&obsNum);
    minMaxNormalize(&maxTrackDist);
    minMaxNormalize(&maxAngle);
    minMaxNormalizeInv(&avgReprojectionErr);

    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
    std::cout << "get landmark score !!" << std::endl;
    landmarkScore.resize(originalMapPointNum);
    for (size_t i = 0; i < landmarkScore.size(); i++)
    {
        // std::cout << obsNum_[i] << " " << maxTrackDist[i] << " " << maxAngle[i] << " " << avgReprojectionErr[i] << std::endl;
        // landmarkScore[i] = a * obsNum_[i] + b * maxTrackDist[i] + c * maxAngle[i] + d * avgReprojectionErr[i];
        landmarkScore[i] = a * obsNum_[i];
        mpDB[i]->score = landmarkScore[i];
    }
    minMaxNormalize(&landmarkScore);
}

void Compression::getKeyframeScore()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);

    keyframeScore.resize(kfdb.size());

    for (size_t i = 0; i < kfdb.size(); i++)
    {

        std::vector<ORB_SLAM2::MapPoint *> keyframeMap = getKeyframeMap(kfdb[i]);
        for (size_t j = 0; j < keyframeMap.size(); j++)
        {
            keyframeScore[i] += keyframeMap[j]->score;
        }
        keyframeScore[i] /= (float)keyframeMap.size();
    }
    minMaxNormalize(&keyframeScore);
}

void Compression::calculateVariousScore()
{
    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();

    obsNum.resize(mpDB.size());
    maxTrackDist.resize(mpDB.size());
    maxAngle.resize(mpDB.size());
    avgReprojectionErr.resize(mpDB.size());
    for (size_t i = 0; i < mpDB.size(); i++)
    {

        // Observation Count
        // std::cout << "get observation" << std::endl;
        obsNum[i] = getObservation(mpDB[i]);

        // Max Track Distance
        // std::cout << "get MaxTrackDistance" << std::endl;
        maxTrackDist[i] = getMaxTrackDistance(mpDB[i]);

        // Max Angle
        // std::cout << "get MaxAngle" << std::endl;
        maxAngle[i] = getMaxAngle(mpDB[i]);

        // Average Reprojection Error
        // std::cout << "get Average Reprojection Error" << std::endl;
        avgReprojectionErr[i] = getreprojectionErrAvg(mpDB[i]);

        // debug
        // std::cout << obsNum[i] << " " << maxTrackDist[i] << " " << maxAngle[i] << " " << avgReprojectionErr[i] << std::endl;
    }
}

std::map<std::tuple<int, int>, double> Compression::getKeypointDistanceMatrix()
{
    std::vector<ORB_SLAM2::KeyFrame *> kfdb = Map->GetAllKeyFrames();
    std::sort(kfdb.begin(), kfdb.end(), ORB_SLAM2::KeyFrame::lId);
    
    std::vector<ORB_SLAM2::MapPoint *> mpDB = Map->GetAllMapPoints();
    // keypointDistanceMatrix.resize(mpDB.size(), mpDB.size());
    // keypointDistanceMatrix.setZero();
    for(size_t i = 0; i < mpDB.size(); i++){
        
        // keypointDistanceMatrix(i, i) = 1.0;
        keypointDistanceQ.insert(std::pair<std::tuple<int, int>, double>(std::make_tuple(i, i), 1.0));
        std::vector<int> KfIdx = getKeyframeObsId(kfdb, mpDB[i]);
        
        for(size_t j = i + 1; j < mpDB.size(); j++){
            
                
            std::vector<int> covKfIdx = getKeyframeCovisibleMp(kfdb, KfIdx, mpDB[j]);
            if(covKfIdx.size() == 0) continue;

            double KeypointDistance = 0;
            for(size_t k = 0; k < covKfIdx.size(); k++){
                    
                int mpIdx1 = mpDB[i]->GetIndexInKeyFrame(kfdb[covKfIdx[k]]);
                int mpIdx2 = mpDB[j]->GetIndexInKeyFrame(kfdb[covKfIdx[k]]);

                Eigen::Vector2d keyPoint1, keyPoint2;
                keyPoint1 << kfdb[covKfIdx[k]]->mvKeys[mpIdx1].pt.x, kfdb[covKfIdx[k]]->mvKeys[mpIdx1].pt.y;
                keyPoint2 << kfdb[covKfIdx[k]]->mvKeys[mpIdx2].pt.x, kfdb[covKfIdx[k]]->mvKeys[mpIdx2].pt.y;

                double featureDist = (keyPoint1 - keyPoint2).norm();
                // std::cout << featureDist << " ";
                KeypointDistance += std::max(0.0, (keypointDistThres - featureDist)/keypointDistThres);
            }
                
            KeypointDistance /= (double)covKfIdx.size();
            // std::cout << KeypointDistance << " ";
            if(KeypointDistance != 0.0) {
                // std::cout << KeypointDistance << " ";
                keypointDistanceQ.insert(std::pair<std::tuple<int, int>, double>(std::make_tuple(i, j), KeypointDistance));    
            }
            // keypointDistanceMatrix(i, j) = KeypointDistance;
            // keypointDistanceMatrix(j, i) = KeypointDistance;
            

        }
        // std::cout << i << std::endl;
        // if((i % 100) == 0){
        //     double ing = (double)i / (double)mpDB.size();
        //     std::cout << ing * 100  << " ";
        // }
    }
    std::cout << "size of map : " << keypointDistanceQ.size() << std::endl;
    std::cout << std::endl;

    return keypointDistanceQ;
}

std::vector<int> Compression::getKeyframeObsId( std::vector<ORB_SLAM2::KeyFrame *> &kfdb,
                                                ORB_SLAM2::MapPoint* mp)
{
    std::vector<int> kfIdxs;
    for(size_t i = 0; i < kfdb.size(); i++){

        bool isInKf = mp->IsInKeyFrame(kfdb[i]);
        if(isInKf) kfIdxs.emplace_back(i);
    }
    return kfIdxs;
}

std::vector<int> Compression::getKeyframeCovisibleMp(std::vector<ORB_SLAM2::KeyFrame *> &kfdb,
                                        std::vector<int> KfIdx1,
                                        ORB_SLAM2::MapPoint* mp2 )
{
    std::vector<int> kfIdxs;
    for(size_t i = 0; i < KfIdx1.size(); i++){

        bool isInKf2 = mp2->IsInKeyFrame(kfdb[KfIdx1[i]]);
        if(isInKf2) kfIdxs.emplace_back(KfIdx1[i]);
    }
    return kfIdxs;
}

//////////////////////////////////////////////////////////////////////////

int Compression::cvMatSize(cv::Mat a)
{
    int size = a.elemSize() * a.total();
    return size;
}

int Compression::getMemory(ORB_SLAM2::MapPoint *mp)
{
    int obsCnt = mp->mObservations.size();
    obsCnt *= 48;  // observation(16) + descriptor(32)
    obsCnt += 736; // etc ...
    return obsCnt;
}

int Compression::getMemory(ORB_SLAM2::KeyFrame *kf)
{
    int memory = 0;
    std::vector<ORB_SLAM2::MapPoint *> kfmp = getKeyframeMap(kf);
    int mapPointNum = kfmp.size();
    int kfCovisibility = kf->mConnectedKeyFrameWeights.size();

    memory += mapPointNum * 76;    // Keypoint(28) + descriptor(32) + observation(16)
    memory += kfCovisibility * 24; // Keyframe Pointer(8) * 2 + mpCovisibilityNum(weight, 4) * 2
    memory += 26024;               // etc ...

    return memory;
}

void Compression::minMaxNormalize(Eigen::VectorXd *vec)
{
    std::vector<double> vec2(vec->size());
    for (int i = 0; i < vec->size(); i++)
        vec2[i] = (*vec)[i];

    double maxVal = *max_element(vec2.begin(), vec2.end());
    double minVal = *min_element(vec2.begin(), vec2.end());

    for (int i = 0; i < vec->size(); i++)
    {
        double a = (*vec)[i];
        (*vec)[i] = (a - minVal) / (maxVal - minVal);
    }
}

void Compression::minMaxNormalize(Eigen::VectorXf *vec)
{
    std::vector<float> vec2(vec->size());
    for (int i = 0; i < vec->size(); i++)
        vec2[i] = (*vec)[i];

    float maxVal = *max_element(vec2.begin(), vec2.end());
    float minVal = *min_element(vec2.begin(), vec2.end());

    for (int i = 0; i < vec->size(); i++)
    {
        float a = (*vec)[i];
        (*vec)[i] = (a - minVal) / (maxVal - minVal);
    }
}

void Compression::minMaxNormalize(std::vector<double> *vec)
{
    double maxVal = *max_element(vec->begin(), vec->end());
    double minVal = *min_element(vec->begin(), vec->end());

    for (size_t i = 0; i < vec->size(); i++)
    {
        double a = (*vec)[i];
        (*vec)[i] = (a - minVal) / (maxVal - minVal);
    }
}

void Compression::minMaxNormalize(std::vector<float> *vec)
{
    float maxVal = *max_element(vec->begin(), vec->end());
    float minVal = *min_element(vec->begin(), vec->end());

    for (size_t i = 0; i < vec->size(); i++)
    {
        float a = (*vec)[i];
        (*vec)[i] = (a - minVal) / (maxVal - minVal);
    }
}

void Compression::minMaxNormalizeInv(std::vector<float> *vec)
{
    float maxVal = *max_element(vec->begin(), vec->end());
    float minVal = *min_element(vec->begin(), vec->end());

    for (size_t i = 0; i < vec->size(); i++)
    {
        float a = (*vec)[i];
        (*vec)[i] = std::fabs(a - maxVal) / (maxVal - minVal);
    }
}

std::vector<float> Compression::minMaxNormalize(std::vector<int> *vec)
{
    int maxVal = *max_element(vec->begin(), vec->end());
    int minVal = *min_element(vec->begin(), vec->end());

    std::vector<float> vec_(vec->size());
    for (size_t i = 0; i < vec->size(); i++)
    {
        int a = (*vec)[i];
        vec_[i] = (float)(a - minVal) / (float)(maxVal - minVal);
    }

    return vec_;
}

void Compression::zScoreNormalize(std::vector<int> *vec)
{
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

    for (auto u : S)
    {

        if (u == K)
            return Index;

        Index++;
    }

    return -1;
}