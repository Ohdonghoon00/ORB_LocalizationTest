from matplotlib import pyplot as plt
import numpy as np

removedMemory1 = []
transErr1 = []
rotErr1 = []
failNum1 = []
coeff1 = []

removedMemory2 = []
transErr2 = []
rotErr2 = []
failNum2 = []
coeff2 = []

removedMemory3 = []
transErr3 = []
rotErr3 = []
failNum3 = []
coeff3 = []

removedMemory4 = []
transErr4 = []
rotErr4 = []
failNum4 = []
coeff4 = []

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
        coeff1.append(float(line_list[7]))

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
        coeff2.append(float(line_list[7]))

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
        failNum3.append(float(line_list[6]))
        coeff3.append(float(line_list[7]))

    file.close()    
    
def readResult4(path):
    
    # read file
    file = open(path, "r")
    lines = file.readlines()
    for line in lines:
        line_list = line.split()
        
        removedMemory4.append(float(line_list[1]))
        transErr4.append(float(line_list[4]))
        rotErr4.append(float(line_list[5]))
        failNum4.append(float(line_list[6]))
        coeff4.append(float(line_list[7]))

    file.close()


if __name__ == '__main__':

    filePath1 = 'finalLandmarkCompressionResult(observation).txt'
    filePath2 = 'finalLandmarkCompressionResult(trackDist).txt'
    filePath3 = 'finalLandmarkCompressionResult(angle).txt'
    filePath4 = 'finalLandmarkCompressionResult(reproj).txt'

    readResult1(filePath1)    
    readResult2(filePath2)
    readResult3(filePath3)
    readResult4(filePath4)
    
    print(removedMemory1)
    print(removedMemory2)
    # print(removedMemory3)
    
    # data dictionary
    data_dict1 = {'removed_memory1' : removedMemory1, 'trans_err1' : transErr1, 'rot_err1' : rotErr1, 'fail_Num1' : failNum1, 'coeff1': coeff1}
    data_dict2 = {'removed_memory2' : removedMemory2, 'trans_err2' : transErr2, 'rot_err2' : rotErr2, 'fail_Num2' : failNum2, 'coeff2': coeff2}
    data_dict3 = {'removed_memory3' : removedMemory3, 'trans_err3' : transErr3, 'rot_err3' : rotErr3, 'fail_Num3' : failNum3, 'coeff3': coeff3}
    data_dict4 = {'removed_memory4' : removedMemory4, 'trans_err4' : transErr4, 'rot_err4' : rotErr4, 'fail_Num4' : failNum4, 'coeff4': coeff4}
    
    # # plot x, y
    plt.plot('coeff1', 'fail_Num1', data = data_dict1, label = 'Observation', marker='D', linestyle='dotted')
    plt.plot('coeff2', 'fail_Num2', data = data_dict2, label = 'TrackDist',marker='D', linestyle='dotted')
    plt.plot('coeff3', 'fail_Num3', data = data_dict3, label = 'Angle',marker='D', linestyle='dotted')
    plt.plot('coeff4', 'fail_Num4', data = data_dict4, label = 'ReprojectionErr',marker='D', linestyle='dotted')

    # # label x, y
    xvalues = [0.25,0.40,0.5]
    xlabel = ["0.25", "0.40", "0.50"]
    plt.xticks(xvalues, xlabel)
    plt.xlim(0.2, 0.55)
    plt.xlabel('Coefficient')
    plt.ylabel('Fail Frame Num')
    # plt.ylabel('Translation Error(m)')
    # plt.ylabel('Rotation Error(degree)')

    plt.legend()
    plt.show()