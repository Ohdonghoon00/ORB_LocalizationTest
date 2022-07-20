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

def readResult1(path):
    
    # read file
    file = open(path, "r")
    lines = file.readlines()
    for line in lines:
        line_list = line.split()
        
        removedMemory1.append(float(line_list[1]))
        transErr1.append(float(line_list[4]))
        rotErr1.append(float(line_list[5]))
        failNum1.append(float(line_list[6]))

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
        failNum2.append(float(line_list[6]))

    file.close()        
    
    
    

if __name__ == '__main__':

    filePath1 = 'finalKeyframeCompressionResult.txt'
    filePath2 = 'finalLandmarkCompressionResult.txt'

    readResult1(filePath1)    
    readResult2(filePath2)
    
    
    # print(removedMemory1)
    # print(removedMemory2)
    
    
    # data dictionary
    data_dict1 = {'removed_memory1' : removedMemory1, 'trans_err1' : transErr1, 'rot_err1' : rotErr1, 'fail_Num1' : failNum1}
    data_dict2 = {'removed_memory2' : removedMemory2, 'trans_err2' : transErr2, 'rot_err2' : rotErr2, 'fail_Num2' : failNum2}
    # # data_dict3 = {'removed_memory3' : removedMemory3, 'trans_err3' : transErr3, 'rot_err3' : rotErr3, 'fail_Num3' : failNum3}

    # # plot x, y
    plt.plot('removed_memory1', 'trans_err1', data = data_dict1, label = 'Remove Keyframe')
    plt.plot('removed_memory2', 'trans_err2', data = data_dict2, label = 'Remove Landmark')

    # # label x, y
    plt.xlabel('Removed Memory(MB)')
    # plt.ylabel('Fail Frame Num')
    plt.ylabel('Translation Error(m)')

    plt.legend()
    plt.show()