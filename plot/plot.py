from matplotlib import pyplot as plt
import numpy as np

removedMemory1 = []
transErr1 = []
rotErr1 = []
failNum1 = []

removedMemory2 = []
transErr2 = []
rotErr2 = []
failNum2 = []

removedMemory3 = []
transErr3 = []
rotErr3 = []
failNum3 = []

prunedKeyframeFraction = []
obs1Ratio = []
observation = []
reprojectionErrAvg= []

def readfile3(path):

    # read file
    file = open(path, "r")
    while True:
        line = file.read().splitlines()
        if not line:
            break
        # print(line)
        lines = line
    file.close()
    
    removed_memory_1 = lines[0]
    trans_err_1 = lines[1]
    rot_err_1 = lines[2]
    fail_Num_1 = lines[3]

    removed_memory_2 = lines[4]
    trans_err_2 = lines[5]
    rot_err_2 = lines[6]
    fail_Num_2 = lines[7]

    removed_memory_3 = lines[8]
    trans_err_3 = lines[9]
    rot_err_3 = lines[10]
    fail_Num_3 = lines[11]

    removed_memory1 = removed_memory_1.split('\t')
    trans_err1 = trans_err_1.split('\t')
    rot_err1 = rot_err_1.split('\t')
    fail_Num1 = fail_Num_1.split('\t')

    removed_memory2 = removed_memory_2.split('\t')
    trans_err2 = trans_err_2.split('\t')
    rot_err2 = rot_err_2.split('\t')
    fail_Num2 = fail_Num_2.split('\t')

    removed_memory3 = removed_memory_3.split('\t')
    trans_err3 = trans_err_3.split('\t')
    rot_err3 = rot_err_3.split('\t')
    fail_Num3 = fail_Num_3.split('\t')

    for i in range (0, len(removed_memory1)):
        removedMemory1.append(float(removed_memory1[i]))
        transErr1.append(float(trans_err1[i]))
        rotErr1.append(float(rot_err1[i]))
        failNum1.append(float(fail_Num1[i]))

    for i in range (0, len(removed_memory2)):
        removedMemory2.append(float(removed_memory2[i]))
        transErr2.append(float(trans_err2[i]))
        rotErr2.append(float(rot_err2[i]))
        failNum2.append(float(fail_Num2[i]))

    for i in range (0, len(removed_memory3)):
        removedMemory3.append(float(removed_memory3[i]))
        transErr3.append(float(trans_err3[i]))
        rotErr3.append(float(rot_err3[i]))
        failNum3.append(float(fail_Num3[i]))

def readfileLandmarkNum(path):
    
    # read file
    file = open(path, "r")
    while True:
        line = file.read().splitlines()
        if not line:
            break
        # print(line)
        lines = line
    file.close()

    prunedKeyframeFraction1 = lines[0]
    obs1Ratio1 = lines[1]
    observation1 = lines[2]
    reprojectionErrAvg1 = lines[3]

    prunedKeyframeFraction1_ = prunedKeyframeFraction1.split('\t')
    obs1Ratio1_ = obs1Ratio1.split('\t')
    observation1_ = observation1.split('\t')
    reprojectionErrAvg1_ = reprojectionErrAvg1.split('\t')

    for i in range (0, len(prunedKeyframeFraction1_)):
        prunedKeyframeFraction.append(float(prunedKeyframeFraction1_[i]))
        obs1Ratio.append(float(obs1Ratio1_[i]))
        observation.append(float(observation1_[i]))
        reprojectionErrAvg.append(float(reprojectionErrAvg1_[i]))

if __name__ == '__main__':
    
    filePath = '/home/ohdonghoon/ORB_LocalizationTest/result/220614_test/test_random_constraint_obsKeyframe'
    readfile3(filePath)
    # readfileLandmarkNum(filePath)
    
# data dictionary
    data_dict1 = {'removed_memory1' : removedMemory1, 'trans_err1' : transErr1, 'rot_err1' : rotErr1, 'fail_Num1' : failNum1}
    data_dict2 = {'removed_memory2' : removedMemory2, 'trans_err2' : transErr2, 'rot_err2' : rotErr2, 'fail_Num2' : failNum2}
    data_dict3 = {'removed_memory3' : removedMemory3, 'trans_err3' : transErr3, 'rot_err3' : rotErr3, 'fail_Num3' : failNum3}
    
    # data_dict_landmark = {'prunedKeyframeFraction' : prunedKeyframeFraction, 'obs1Ratio' : obs1Ratio, 'observation' : observation, 'reprojectionErrAvg' : reprojectionErrAvg}

# plot x, y
    plt.plot('removed_memory1', 'fail_Num1', data = data_dict1, label = 'Random')
    plt.plot('removed_memory2', 'fail_Num2', data = data_dict2, label = 'Random, Constraint')
    plt.plot('removed_memory3', 'fail_Num3', data = data_dict3, label = 'ObservationAvg')
    
    # plt.plot('prunedKeyframeFraction', 'obs1Ratio', data = data_dict_landmark, label = 'obs1Ratio')
    # plt.plot('prunedKeyframeFraction', 'observation', data = data_dict_landmark, label = 'observationAvg')
    # plt.plot('prunedKeyframeFraction', 'reprojectionErrAvg', data = data_dict_landmark, label = 'reprojectionErrAvg')

# label x, y
    plt.xlabel('Removed Memory(MB)')
    plt.ylabel('Fail Frame Num')
    # plt.ylabel('Translation Error(m)')

    # plt.xlabel('pruned Keyframe Fraction')
    # plt.ylabel('Landmark Num')
    
    plt.legend()
    plt.show()