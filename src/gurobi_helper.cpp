#include "gurobi_helper.h"


std::vector<GRBVar> CreateVariablesBinaryVector(int PointCloudNum, GRBModel& model_)
{
    
    std::vector<GRBVar> x;
    x.resize(PointCloudNum);
    for(int i = 0; i < PointCloudNum; i++ )
    {
        x[i] = model_.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    return x;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> CalculateObservationCountWeight(ORB_SLAM2::Map* map_data)
{
    Eigen::Matrix<double, Eigen::Dynamic, 1> q;
    int PointCloudNum_ = map_data->MapPointsInMap();
    q.resize(PointCloudNum_);
    int KeyframeNum = map_data->KeyFramesInMap();
    std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
    
    for(int i = 0; i < PointCloudNum_; i++)
    {
        q[i] = ((double)KeyframeNum - (double)AllMpptr[i]->GetObservations().size()) / (double)KeyframeNum;
        // q[i] = ((double)AllMpptr[i]->GetObservations().size()) / (double)KeyframeNum;
    }
    return q;
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

void SetObjectiveILP(std::vector<GRBVar> x_, Eigen::Matrix<double, Eigen::Dynamic, 1> q_, GRBModel& model_)
{
    GRBLinExpr obj = 0;
    for(size_t i = 0; i < x_.size(); i++)
    {
        obj += q_[i] * x_[i];
    } 
    model_.setObjective(obj);
    
}

Eigen::MatrixXd CalculateVisibilityMatrix(ORB_SLAM2::Map* map_data)
{

    std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
    std::vector<ORB_SLAM2::KeyFrame*> AllKFptr = map_data->GetAllKeyFrames();
    Eigen::MatrixXd A(map_data->KeyFramesInMap(), map_data->MapPointsInMap()); 
    A.setZero();
    for(int i = 0; i < A.rows(); i++ )
        {
            for(int j = 0; j < A.cols(); j++)
            {
                
                bool IsInKF = AllMpptr[j]->IsInKeyFrame(AllKFptr[i]);
                if(IsInKF){
                    A(i, j) = 1.0;
                }    
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
                
                


void AddConstraint(ORB_SLAM2::Map* map_data, GRBModel& model_, Eigen::MatrixXd A, std::vector<GRBVar> x, double CompressionRatio)
{
    GRBLinExpr MinKeyframePointNum = 0;
    GRBLinExpr TotalPointNum = 0;

    double b = 100.0; // Minimum point num by one Keyframe
    // double CompressionRatio = 0.7; // Compression Ratio to Landmarks

    double totalNum = (double)(int)(map_data->MapPointsInMap() * CompressionRatio);
    // double totalNum = 4000.0;
std::cout << totalNum << std::endl;
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