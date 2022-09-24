from matplotlib import pyplot as plt
import numpy as np
# from scipy.interpolate import make_interp_spline

removedMemory1 = []
transErr1 = []
rotErr1 = []
successRate1 = []
stdTrans1 = []
stdRot1 = []

removedMemory2 = []
transErr2 = []
rotErr2 = []
successRate2 = []
stdTrans2 = []
stdRot2 = []

removedMemory3 = []
transErr3 = []
rotErr3 = []
successRate3 = []
stdTrans3 = []
stdRot3 = []

def readResult1(path):
    
    # read file
    file = open(path, "r")
    lines = file.readlines()
    for line in lines:
        line_list = line.split()
        
        removedMemory1.append(float(line_list[1]))
        transErr1.append(float(line_list[4]))
        rotErr1.append(float(line_list[5]))
        successRate1.append(float(line_list[6]))
        stdTrans1.append(float(line_list[7]))
        stdRot1.append(float(line_list[8]))

    file.close()

def readResult2(path):
    
    # read file
    file = open(path, "r")
    lines = file.readlines()
    for line in lines:
        line_list = line.split()
        
        removedMemory2.append(float(line_list[1]))
        transErr2.append(float(line_list[4]))
        rotErr2.append(float(line_list[5]))
        successRate2.append(float(line_list[6]))
        stdTrans2.append(float(line_list[7]))
        stdRot2.append(float(line_list[8]))



    file.close()

def readResult3(path):
    
    # read file
    file = open(path, "r")
    lines = file.readlines()
    for line in lines:
        line_list = line.split()
        
        removedMemory3.append(float(line_list[1]))
        transErr3.append(float(line_list[4]))
        rotErr3.append(float(line_list[5]))
        successRate3.append(float(line_list[6]))
        stdTrans3.append(float(line_list[7]))
        stdRot3.append(float(line_list[8]))



    file.close()

# def readfile3(path):

#     # read file
#     file = open(path, "r")
#     while True:
#         line = file.read().splitlines()
#         if not line:
#             break
#         # print(line)
#         lines = line
#     file.close()
    
#     removed_memory_1 = lines[0]
#     trans_err_1 = lines[1]
#     rot_err_1 = lines[2]
#     fail_Num_1 = lines[3]

#     removed_memory_2 = lines[4]
#     trans_err_2 = lines[5]
#     rot_err_2 = lines[6]
#     fail_Num_2 = lines[7]

#     removed_memory_3 = lines[8]
#     trans_err_3 = lines[9]
#     rot_err_3 = lines[10]
#     fail_Num_3 = lines[11]

#     removed_memory1 = removed_memory_1.split('\t')
#     trans_err1 = trans_err_1.split('\t')
#     rot_err1 = rot_err_1.split('\t')
#     fail_Num1 = fail_Num_1.split('\t')

#     removed_memory2 = removed_memory_2.split('\t')
#     trans_err2 = trans_err_2.split('\t')
#     rot_err2 = rot_err_2.split('\t')
#     fail_Num2 = fail_Num_2.split('\t')

#     removed_memory3 = removed_memory_3.split('\t')
#     trans_err3 = trans_err_3.split('\t')
#     rot_err3 = rot_err_3.split('\t')
#     fail_Num3 = fail_Num_3.split('\t')

#     for i in range (0, len(removed_memory1)):
#         removedMemory1.append(float(removed_memory1[i]))
#         transErr1.append(float(trans_err1[i]))
#         rotErr1.append(float(rot_err1[i]))
#         successRate1.append(float(fail_Num1[i]))

#     for i in range (0, len(removed_memory2)):
#         removedMemory2.append(float(removed_memory2[i]))
#         transErr2.append(float(trans_err2[i]))
#         rotErr2.append(float(rot_err2[i]))
#         successRate2.append(float(fail_Num2[i]))

#     for i in range (0, len(removed_memory3)):
#         removedMemory3.append(float(removed_memory3[i]))
#         transErr3.append(float(trans_err3[i]))
#         rotErr3.append(float(rot_err3[i]))
#         successRate3.append(float(fail_Num3[i]))

# def readfileLandmarkNum(path):
    
#     # read file
#     file = open(path, "r")
#     while True:
#         line = file.read().splitlines()
#         if not line:
#             break
#         # print(line)
#         lines = line
#     file.close()

#     prunedKeyframeFraction1 = lines[0]
#     obs1Ratio1 = lines[1]
#     observation1 = lines[2]
#     reprojectionErrAvg1 = lines[3]

#     prunedKeyframeFraction1_ = prunedKeyframeFraction1.split('\t')
#     obs1Ratio1_ = obs1Ratio1.split('\t')
#     observation1_ = observation1.split('\t')
#     reprojectionErrAvg1_ = reprojectionErrAvg1.split('\t')

#     for i in range (0, len(prunedKeyframeFraction1_)):
#         prunedKeyframeFraction.append(float(prunedKeyframeFraction1_[i]))
#         obs1Ratio.append(float(obs1Ratio1_[i]))
#         observation.append(float(observation1_[i]))
#         reprojectionErrAvg.append(float(reprojectionErrAvg1_[i]))

if __name__ == '__main__':
    # VPSResult_0.25_2.0 VPSResult_0.5_5.0
    filePath1 = 'assembleResult/220917/VPSResult_0.25_2.0/220905_MH03_MH02_finalKeyframeSimilarity_CompressionResult(1111).txt'
    filePath2 = 'assembleResult/220917/VPSResult_0.25_2.0/220905_MH03_MH02_finalLandmark_CompressionResult(1111).txt'
    filePath3 = 'assembleResult/220917/VPSResult_0.25_2.0/220905_MH03_MH02_finalLandmarkIQP_CompressionResult(1111).txt'
    
    readResult1(filePath1)
    readResult2(filePath2)
    readResult3(filePath3)
    # readfileLandmarkNum(filePath)
    
# data dictionary
    data_dict1 = {'removed_memory1' : removedMemory1, 'trans_err1' : transErr1, 'rot_err1' : rotErr1, 'success_Rate1' : successRate1, 'std_Trans1' : stdTrans1, 'std_Rot1' : stdRot1}
    data_dict2 = {'removed_memory2' : removedMemory2, 'trans_err2' : transErr2, 'rot_err2' : rotErr2, 'success_Rate2' : successRate2, 'std_Trans2' : stdTrans2, 'std_rot2' : stdRot2}
    data_dict3 = {'removed_memory3' : removedMemory3, 'trans_err3' : transErr3, 'rot_err3' : rotErr3, 'success_Rate3' : successRate3, 'std_Trans3' : stdTrans3, 'std_rot3' : stdRot3}
    
    # data_dict_landmark = {'prunedKeyframeFraction' : prunedKeyframeFraction, 'obs1Ratio' : obs1Ratio, 'observation' : observation, 'reprojectionErrAvg' : reprojectionErrAvg}

# plot x, y
    # plt.plot('removed_memory1', 'success_Rate1', data = data_dict1, label = 'remove keyframe')
    # plt.plot('removed_memory2', 'success_Rate2', data = data_dict2, label = 'remove landmark')
    
    plt.plot('removed_memory1', 'success_Rate1', data = data_dict1, label = 'KeyframeSimilarity')
    plt.plot('removed_memory2', 'success_Rate2', data = data_dict2, label = 'LandmarkILP')
    plt.plot('removed_memory3', 'success_Rate3', data = data_dict3, label = 'LandmarkIQP')
    

    # plt.errorbar(removedMemory1, transErr1, yerr = stdTrans1, label = 'Similarity',  marker='D', linestyle='dotted', capsize=3 )
    # plt.errorbar(removedMemory2, transErr2, yerr = stdTrans2, label = 'KeyframeScore',  marker='D', linestyle='dotted', capsize=3 )
    # plt.errorbar(removedMemory3, transErr3, yerr = stdTrans3, label = 'KeyframeScore + Similarity',  marker='D', linestyle='dotted', capsize=3 )

    # plt.plot('removed_memory1', 'rot_err1', data = data_dict1, label = 'remove keyframe')
    # plt.plot('removed_memory2', 'rot_err2', data = data_dict2, label = 'remove landmark')

    
    # fig, plt1 = plt.subplots()
    # plt1.plot('removed_memory1', 'trans_err1', data = data_dict1, label = 'Remove Keyframe', color = 'green')
    # plt1.plot('removed_memory2', 'trans_err2', data = data_dict2, label = 'Remove Landmark', color = 'blue')
    # plt2 = plt1.twinx()
    # plt2.plot('removed_memory1', 'std_Trans1', data = data_dict1,  marker='D', linestyle='dotted' )
    # plt2.plot('removed_memory2', 'std_Trans2', data = data_dict2,  marker='D', linestyle='dotted' )
    
    # plt.plot('prunedKeyframeFraction', 'obs1Ratio', data = data_dict_landmark, label = 'obs1Ratio')
    # plt.plot('prunedKeyframeFraction', 'observation', data = data_dict_landmark, label = 'observationAvg')
    # plt.plot('prunedKeyframeFraction', 'reprojectionErrAvg', data = data_dict_landmark, label = 'reprojectionErrAvg')

# label x, y
    plt.xlabel('Removed Memory(MB)')
    plt.ylabel('Recall')
    # plt.ylabel('Translation Error(m)')
    # plt.ylim(0, None)
    
    # plt1.set_xlabel('Removed Memory(MB)')
    # plt1.set_ylabel('Translation Error(m)')
    # plt2.set_ylabel('Standard Deviation')

    # plt.xlabel('pruned Keyframe Fraction')
    # plt.ylabel('Landmark Num')
    
    plt.legend()
    plt.show()