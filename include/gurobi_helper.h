#pragma once

#include "gurobi_c++.h"
#include <vector>
#include <Eigen/Dense>
#include "algorithm"
#include "Map.h"
#include "System.h"
#include "Compression.h"


std::vector<GRBVar> CreateVariablesBinaryVectorForLandmark( int PointCloudNum, 
                                                            GRBModel& model_);
std::vector<GRBVar> CreateVariablesBinaryVectorForKeyframe( int keyframeNum, 
                                                            GRBModel& model_);

Eigen::Matrix<double, Eigen::Dynamic, 1> CalculateObservationCountWeight(ORB_SLAM2::Map* map_data);
Eigen::Matrix<double, Eigen::Dynamic, 1> setLandmarkWeight(ORB_SLAM2::Map* map_data, std::vector<float> weight);
Eigen::MatrixXd calculateKeyframeSimilarity(ORB_SLAM2::Map* map_data);
// Eigen::Matrix<double, Eigen::Dynamic, 1> CalculateObservationCountWeight2(DataBase* DB);

void SetObjectiveILPforLandmark(std::vector<GRBVar> x_, 
                                Eigen::Matrix<double, Eigen::Dynamic, 1> q_,
                                GRBModel& model_);
void SetObjectiveILPforKeyframe(std::vector<GRBVar> x_, 
                                std::vector<float> keyframeScore, 
                                GRBModel& model_);
void SetObjectiveIQPforKeyframe(std::vector<GRBVar> x_, 
                                Eigen::MatrixXd S,
                                std::vector<float> keyframeScore, 
                                GRBModel& model_);
void SetObjectiveforKeyframeMapCube(std::vector<GRBVar> x_, 
                                    Eigen::MatrixXi visibilityMatrix,
                                    std::vector<int> originalCubeVector,
                                    std::vector<int> cubeIds,
                                    std::set<int> cubeIdsSet,
                                    Eigen::MatrixXd S,
                                    GRBModel& model_);


Eigen::MatrixXd CalculateVisibilityMatrix(ORB_SLAM2::Map* map_data);
// Eigen::MatrixXd CalculateVisibilityMatrix2(DataBase* DB);

void AddConstraintForLandmark( ORB_SLAM2::Map* map_data, 
                    GRBModel& model_, 
                    Eigen::MatrixXd A, 
                    std::vector<GRBVar> x, 
                    double CompressionRatio);
void AddConstraintForKeyframe( ORB_SLAM2::Map* map_data, 
                    GRBModel& model_, 
                    std::vector<GRBVar> x, 
                    double CompressionRatio,
                    int neighborKeyframeIdThres);

// void AddConstraint2(DataBase* DB, GRBModel& model_, Eigen::MatrixXd A, std::vector<GRBVar> x, double CompressionRatio);
