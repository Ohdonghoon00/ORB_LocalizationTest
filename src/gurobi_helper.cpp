#include "gurobi_helper.h"


std::vector<GRBVar> CreateVariablesBinaryVectorForLandmark(int PointCloudNum, GRBModel& model_)
{
    
    std::vector<GRBVar> x;
    x.resize(PointCloudNum);
    for(int i = 0; i < PointCloudNum; i++ )
    {
        x[i] = model_.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    return x;
}

std::vector<GRBVar> CreateVariablesBinaryVectorForKeyframe(int keyframeNum, GRBModel& model_)
{
    
    std::vector<GRBVar> x;
    x.resize(keyframeNum - 1);
    for(int i = 0; i < keyframeNum - 1; i++ )
    {
        x[i] = model_.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    return x;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> CalculateObservationCountWeight(ORB_SLAM2::Map* map_data)
{
    int totalVal = 0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q;
    int PointCloudNum_ = map_data->MapPointsInMap();
    int KeyframeNum = map_data->KeyFramesInMap();
    q.resize(PointCloudNum_);
    
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = map_data->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);
    for(int i = 0; i < KeyframeNum; i++) totalVal += kfdb[i]->GetMapPoints().size();
    std::cout << "total value : " <<  totalVal << std::endl;
    std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
    for(int i = 0; i < PointCloudNum_; i++)
    {
        int kfTotalPts = 0;
        for(int j = 0; j < KeyframeNum; j++){
            bool IsInKF = AllMpptr[i]->IsInKeyFrame(kfdb[j]);
            if(IsInKF) kfTotalPts += kfdb[j]->GetMapPoints().size();
        }
        // std::cout << kfTotalPts << std::endl;
        // q[i] = (((double)KeyframeNum - (double)AllMpptr[i]->GetObservations().size()) / (double)KeyframeNum) + (0.6) * ((double)kfTotalPts / (double)totalVal);
        q[i] = (((double)KeyframeNum - (double)AllMpptr[i]->GetObservations().size()) / (double)KeyframeNum);
        // q[i] = ((double)AllMpptr[i]->GetObservations().size() / (double)KeyframeNum); // for debug
    }
    return q;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> setLandmarkWeight(ORB_SLAM2::Map* map_data, std::vector<float> weight)
{
    int totalVal = 0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> q;
    int PointCloudNum_ = map_data->MapPointsInMap();
    q.resize(PointCloudNum_);
    for(int i = 0; i < PointCloudNum_; i++)
        q[i] = (double)weight[i];
    return q;
}

    

Eigen::MatrixXd calculateKeyframeSimilarity(ORB_SLAM2::Map* map_data)
{
    std::vector<ORB_SLAM2::KeyFrame*> kfdb = map_data->GetAllKeyFrames();
    std::sort(kfdb.begin(),kfdb.end(),ORB_SLAM2::KeyFrame::lId);

    int keyframeNum = kfdb.size();

    Eigen::MatrixXd S;
    S.resize(keyframeNum - 1, keyframeNum - 1);

    for(size_t i = 1; i < kfdb.size(); i++){

        int lastKeyframeMpNum = Compression::getKeyframeMap(kfdb[i]).size();
        for(size_t j = 1; j < kfdb.size(); j++){
            
            // getRelativePose(kfdb[i], kfdb[j]);
            // // std::cout << relPoseErr[0] << "    " << ORB_SLAM2::Converter::Rad2Degree(relPoseErr[1]) << std::endl;
            // if(relPoseErr[0] < 1.0){

                int covisibilityMpNum = Compression::getCovisibilityMpNum(kfdb[i], kfdb[j]);
                S(i - 1, j - 1) = (double)covisibilityMpNum / (double)lastKeyframeMpNum;
            // }
            // else
            //     similarityMatrix(i - 1, j - 1) = 0.0;

        }
    
    }

    return S;
}


// Eigen::Matrix<double, Eigen::Dynamic, 1> CalculateObservationCountWeight2(DataBase* DB)
// {
//     Eigen::Matrix<double, Eigen::Dynamic, 1> q;
//     int PointCloudNum_ = DB->Landmarks.size();
//     q.resize(PointCloudNum_);
//     int KeyframeNum = DB->KFtoMPIdx.size();
//     // std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
//     std::cout << "Keyframe num : " << KeyframeNum << std::endl;
//     for(int i = 0; i < PointCloudNum_; i++)
//     {
//         q[i] = ((double)KeyframeNum - (double)(DB->GetObservationCount(i))) / (double)KeyframeNum;
//         // std::cout << q[i] << " ";
//     }
//     return q;
// }

void SetObjectiveILPforLandmark(std::vector<GRBVar> x_, 
                                Eigen::Matrix<double, Eigen::Dynamic, 1> q_, 
                                GRBModel& model_)
{
    GRBLinExpr obj = 0;
    for(size_t i = 0; i < x_.size(); i++)
        obj += q_[i] * x_[i];
    
    model_.setObjective(obj);
}
    

void SetObjectiveILPforKeyframe(std::vector<GRBVar> x_, 
                                std::vector<float> keyframeScore, 
                                GRBModel& model_)
{
    GRBLinExpr obj = 0;
    std::vector<GRBLinExpr> similarityKeyframe(x_.size());
    for(size_t i = 0; i < x_.size(); i++){
        
        obj +=  (double)keyframeScore[i + 1] * x_[i];
            
    }    

    

    model_.setObjective(obj);
}

void SetObjectiveIQPforKeyframe(std::vector<GRBVar> x_, 
                                Eigen::MatrixXd S,
                                std::vector<float> keyframeScore, 
                                GRBModel& model_)
{
    GRBQuadExpr obj = 0;
    for(size_t i = 0; i < x_.size(); i++){
        
            for(size_t j = 0; j < x_.size(); j++){
                obj += S(i, j) * x_[j] * x_[i];
            }

            obj += (double)keyframeScore[i + 1] * x_[i];
    }    
    

    

    model_.setObjective(obj);
}

            
        
void SetObjectiveforKeyframeMapCube(std::vector<GRBVar> x_, 
                                    Eigen::MatrixXi visibilityMatrix,
                                    std::vector<int> originalCubeVector,
                                    std::vector<int> cubeIds,
                                    std::set<int> cubeIdsSet,
                                    Eigen::MatrixXd S,
                                    GRBModel& model_)
{
    // try{
        GRBQuadExpr obj = 0;
        
        // Create Observation Vector
        std::cout << "create observation vector ... " << std::endl;
        std::vector<GRBLinExpr> observationVector(visibilityMatrix.rows());
        for(int i = 0; i < visibilityMatrix.rows(); i++){
            
            GRBLinExpr obs = 0;
            for(int j = 0; j < visibilityMatrix.cols(); j++){
                obs += (double)visibilityMatrix(i, j) * x_[j];
            }
            if(obs.getConstant() == 0)
                observationVector[i] = 0;
            else
                observationVector[i] = 1;
        }
                

        // Caculate MapPoint Num in Small Cube
        std::cout << "Caculate compressedCubeVector ... " << std::endl;
        std::vector<GRBLinExpr> compressedCubeVector(cubeIdsSet.size(), 0);
        for(size_t i = 0; i < cubeIds.size(); i++){
            
            int cubeid_ = cubeIds[i];
            int idx = Compression::getSetIndex(cubeIdsSet, cubeid_);
            
            if(idx == -1){
                
                std::cout << "not found,,,?? " << std::endl;
            }
            else{
                compressedCubeVector[idx] += 1 * observationVector[i];
            }        
        }
        
        // delete smaller than thres
        std::cout << "delete smaller than thres ... " << std::endl;
        int eraseIdx = 0;
        for(size_t i = 0; i < originalCubeVector.size(); i++){
            if(originalCubeVector[i - eraseIdx] < 4){
                
                originalCubeVector.erase(originalCubeVector.begin() + i - eraseIdx);
                compressedCubeVector.erase(compressedCubeVector.begin() + i - eraseIdx);
                eraseIdx++;
            }
        }
                
        std::cout << "small cube size : " << originalCubeVector.size() << std::endl;
        
        // Ratio of CubeVector between Original and Compressed Map
        std::cout << "Caculate Ratio of CubeVector ... " << std::endl;
        std::vector<GRBLinExpr> cubeVectorRatio(compressedCubeVector.size());
        GRBLinExpr total = 0;
        for(size_t i = 0; i < compressedCubeVector.size(); i++){

            cubeVectorRatio[i] * (double)originalCubeVector[i] = compressedCubeVector[i];
            // cubeVectorRatio[i] =  (double)originalCubeVector[i] - compressedCubeVector[i];
            total += cubeVectorRatio[i];
        }
        GRBLinExpr avg = total * (double)compressedCubeVector.size();
        
        // Caculate variance for objective
        std::cout << "Caculate variance ... " << std::endl; 
        GRBQuadExpr variance = 0;
        for(size_t i = 0; i < cubeVectorRatio.size(); i++){
            variance += (cubeVectorRatio[i] - avg) * (cubeVectorRatio[i] - avg);
        }
        obj * (double)cubeVectorRatio.size() = variance;
        
        //////////// debug
        std::vector<GRBLinExpr> similarityKeyframe(x_.size());
        for(size_t i = 0; i < x_.size(); i++){
        
        // if(x_[i].get(GRB_DoubleAttr_X) == 1){
            for(size_t j = 0; j < x_.size(); j++){
                obj += S(i, j) * x_[j];
                // similarityKeyframe[i] += S(i, j) * x_[j];
            }                
        // }
        }    
    
        // for(size_t i = 0; i < x_.size(); i++){
        
        //     obj += similarityKeyframe[i] * x_[i];
        // }      
        
        model_.setObjective(obj);
    
    // } catch (GRBException e) {
    //     cout << "Error number: " << e.getErrorCode() << endl;
    //     cout << e.getMessage() << endl;
    //   } 
}    


Eigen::MatrixXd CalculateVisibilityMatrix(ORB_SLAM2::Map* map_data)
{

    std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
    std::vector<ORB_SLAM2::KeyFrame*> AllKFptr = map_data->GetAllKeyFrames();
    Eigen::MatrixXd A(map_data->KeyFramesInMap(), map_data->MapPointsInMap()); 
    A.setZero();
    for(int i = 0; i < A.rows(); i++ ){ // keyFrame Num
            for(int j = 0; j < A.cols(); j++){ // mapPoint Num
            
                bool IsInKF = AllMpptr[j]->IsInKeyFrame(AllKFptr[i]);
                if(IsInKF)
                    A(i, j) = 1.0;
            }
        }
    return A;
}
                
                   


// Eigen::MatrixXd CalculateVisibilityMatrix2(DataBase* DB)
// {

//     int PointCloudNum_ = DB->Landmarks.size();
//     int KeyframeNum = DB->KFtoMPIdx.size();
//     Eigen::MatrixXd A(KeyframeNum, PointCloudNum_); 
//     A.setZero();
//     for(int i = 0; i < DB->KFtoMPIdx.size(); i++ )
//     {
//         for(int j = 0; j < DB->KFtoMPIdx[i].size(); j++)
//         {
//             int idx = DB->KFtoMPIdx[i][j];
//             A(i, idx) = 1.0;
//         }

//     }
//     return A;
// }
                
                


void AddConstraintForLandmark(  ORB_SLAM2::Map* map_data, 
                                GRBModel& model_, 
                                Eigen::MatrixXd A, 
                                std::vector<GRBVar> x, 
                                double CompressionRatio)
{
    GRBLinExpr MinKeyframePointNum = 0;
    GRBLinExpr TotalPointNum = 0;

    double b = 30.0; // Minimum point num by one Keyframe
    // double CompressionRatio = 0.7; // Compression Ratio to Landmarks

    double totalNum = (double)(int)(map_data->MapPointsInMap() * CompressionRatio);
    // double totalNum = 4000.0;
std::cout << "total Pointcloud num : " << totalNum << std::endl;
    for(size_t i = 0; i < map_data->KeyFramesInMap(); i++)
    {
       MinKeyframePointNum.clear();
       for(size_t j = 0; j < map_data->MapPointsInMap(); j++)
       {
        
            MinKeyframePointNum += A(i, j) * x[j];

       }
       model_.addConstr(MinKeyframePointNum >= b);

    }
    for(size_t i = 0; i < map_data->MapPointsInMap(); i++){

        TotalPointNum = TotalPointNum + x[i];
    }
    model_.addConstr(TotalPointNum, GRB_EQUAL, totalNum);
}

void AddConstraintForKeyframe(  ORB_SLAM2::Map* map_data, 
                                GRBModel& model_, 
                                std::vector<GRBVar> x, 
                                double CompressionRatio,
                                int neighborKeyframeIdThres)
{
    // try{
    GRBLinExpr totalKeyframeNum = 0;
    double totalNum = (double)(int)(map_data->KeyFramesInMap() * CompressionRatio);

    for(size_t i = 0; i < x.size() - neighborKeyframeIdThres + 1; i++){
        
        GRBLinExpr neighborKeyframe = 0;
        for(int j = 0; j < neighborKeyframeIdThres; j++){
            neighborKeyframe += x[i + j]; 
        }
        model_.addConstr(neighborKeyframe >= 1.0);
    }

    for(size_t i = 0; i < map_data->KeyFramesInMap() - 1; i++){

        totalKeyframeNum = totalKeyframeNum + x[i];
    }

    model_.addConstr(totalKeyframeNum, GRB_EQUAL, totalNum);

    //     } catch (GRBException e) {
    //     cout << "Error number: " << e.getErrorCode() << endl;
    //     cout << e.getMessage() << endl;
    // } 
}

// void AddConstraint2(DataBase* DB, GRBModel& model_, Eigen::MatrixXd A, std::vector<GRBVar> x, double CompressionRatio)
// {
    
//     GRBLinExpr TotalPointNum = 0;
//     GRBLinExpr MinKeyframePointNum = 0;

//     double b = 30.0; // Minimum point num by one Keyframe
//     // double CompressionRatio = 0.7; // Compression Ratio to Landmarks
    
//     double PointCloudNum_ = (double)DB->Landmarks.size();
//     int KeyframeNum = DB->KFtoMPIdx.size();
    
//     double totalNum = (double)(int)(PointCloudNum_ * CompressionRatio);
// std::cout << "Total Landmark num after compression  : " << totalNum << std::endl;
//     for(int i = 0; i < KeyframeNum; i++)
//     {
       
//        double MaxPointNumInKeyframe = (double)DB->KFtoMPIdx[i].size();
//        MinKeyframePointNum.clear();
//        for(int j = 0; j < (int)PointCloudNum_; j++)
//        {
        
//             MinKeyframePointNum += A(i, j) * x[j];

//        }
//        if(MaxPointNumInKeyframe < b) b = MaxPointNumInKeyframe;
//        model_.addConstr(MinKeyframePointNum >= b);
//        b = 30.0;
//     }
//     for(int i = 0; i < (int)PointCloudNum_; i++){

//         TotalPointNum = TotalPointNum + x[i];
//     }
//     model_.addConstr(TotalPointNum, GRB_EQUAL, totalNum);
// }