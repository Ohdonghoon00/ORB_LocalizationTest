from matplotlib import pyplot as plt
import numpy as np
import math

transErr1 = []
rotErr1 = []
successRate1 = []
stdTrans1 = []
stdRot1 = []
time1 = []

# db = ["MH01", "MH02", "MH03"]
# query = ["MH01", "MH02", "MH03"]
# method = ["KeyframeSimilarity", "Landmark", "LandmarkIQP"]
# recall = ["VPSResult_0.5_5.0", "VPSResult_0.25_2.0"]
db = "MH03" # MH01 MH02 MH03
query = "MH02"
method = "KeyframeSimilarity" # KeyframeILP, KeyframeIQP, KeyframeSimilarity, Landmark, KeyframeILPIQP, LandmarkIQP
recall = "VPSResult_0.5_5.0" # VPSResult_0.5_5.0  VPSResult_0.25_2.0
# compressionRatioArr=(0.8 0.6 0.4 0.2 0.10 0.08 0.05 0.03) # 8

compressionResultPath = [
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.9_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.8_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.7_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.6_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.5_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.4_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.3_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.2_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.15_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.10_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.08_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.06_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.05_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.04_1._1._1._1._.txt',
    '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.03_1._1._1._1._.txt'
]

VPSResultPath = [
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.9.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.8.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.7.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.6.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.5.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.4.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.3.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.2.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.15.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.10.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.08.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.06.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.05.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.04.bin_1._1._1._1._.binQuery' + query + '.txt',
    '../result/221007/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.03.bin_1._1._1._1._.binQuery' + query + '.txt'
]

# compressionResultPath = [
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.8_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.6_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.4_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.2_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.10_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.08_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.05_1._1._1._1._.txt',
#     '../build/Db/CompressionResult/' + db + method + '_Compression_Result_0.03_1._1._1._1._.txt'
# ]

# VPSResultPath = [
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.8.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.6.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.4.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.2.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.10.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.08.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.05.bin_1._1._1._1._.binQuery' + query + '.txt',
#     '../result/220816/' + recall + '/VPS_Result' + db + '_' + method +'_compression_test_0.03.bin_1._1._1._1._.binQuery' + query + '.txt'
# ]

# compressionResultPath = [
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_1._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_2._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_3._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_4._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_5._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_6._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_7._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH03Landmark_Compression_Result_0.10_8._1._1._1._.txt'
# ]

# VPSResultPath = [
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_1._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_2._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_3._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_4._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_5._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_6._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_7._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_8._1._1._1._.binQueryMH02.txt'
# ]

# compressionResultPath = [
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._1._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._2._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._3._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._4._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._5._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._6._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._7._.txt',
#     '../result/220809/CompressionResult/MH01Landmark_Compression_Result_0.10_1._1._1._8._.txt'
# ]

# VPSResultPath = [
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._2._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._3._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._4._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._5._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._6._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._7._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH01_Landmark_compression_test_0.10.bin_1._1._1._8._.binQueryMH02.txt'
# ]

# compressionResultPath = [
#     '../result/220802/CompressionResult/MH03Landmark_Compression_Result_0.10_1._1._1._1._.txt',
#     '../result/220802/CompressionResult/MH03Landmark_Compression_Result_0.10_2._1._1._1._.txt',
#     '../result/220802/CompressionResult/MH03Landmark_Compression_Result_0.10_4._1._1._1._.txt',
#     '../result/220802/CompressionResult/MH03Landmark_Compression_Result_0.10_6._1._1._1._.txt',
#     '../result/220802/CompressionResult/MH03Landmark_Compression_Result_0.10_8._1._1._1._.txt'
# ]

# VPSResultPath = [
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_1._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_2._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_4._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_6._1._1._1._.binQueryMH02.txt',
#     '../result/220809/VPSResult/VPS_ResultMH03_Landmark_compression_test_0.10.bin_8._1._1._1._.binQueryMH02.txt'
# ]

def readfile(path):
    
    # read file
    file = open(path, "r")
    
    line = file.readline()

       
    lines = line
    file.close()
    return lines

def readfileAll(path):
    
    # read file
    file = open(path, "r")
    lines = file.readlines()
    for line in lines:
        line_list = line.split()
        
        transErr1.append(float(line_list[0]))
        rotErr1.append(float(line_list[1]))
        successRate1.append(float(line_list[2]))
        stdTrans1.append(float(line_list[3]))
        stdRot1.append(float(line_list[4]))
        time1.append(float(line_list[5]))

    file.close()

def writefile(path, result1, result2):
    
    # write file
    file = open(path, "a")
    file.write(result1)
    file.write(result2)
    file.close()



if __name__ == '__main__':

    resultFilePath = 'assembleResult/221007/' + recall + '/221007_' + db + '_' + query + '_final' + method + '_CompressionResult(1111).txt'
    file = open(resultFilePath, 'w')
    for i in range(0, len(compressionResultPath)):
        
        compressionResult = readfile(compressionResultPath[i]) + ' '
        # VPSResult = readfile(VPSResultPath[i])
        readfileAll(VPSResultPath[i])
        transErr = 0.0
        rotErr = 0.0
        successRate = 0.0
        stdTrans = 0.0
        stdRot = 0.0
        # time = 0.0
        
        # for j in range(0, len(transErr1)):
            # listSuccessRate.append(successRate1[j])
        tmp = max(successRate1)
        index = successRate1.index(tmp)
        #     transErr += transErr1[j]
        #     rotErr += rotErr1[j]
        #     successRate += successRate1[j]
        #     stdTrans += stdTrans1[j]
        #     stdRot += stdRot1[j]
        # transErr /= len(transErr1)
        # rotErr /= len(transErr1)
        # successRate /= len(transErr1)
        # stdTrans /= len(transErr1)
        # stdRot /= len(transErr1)
        transErr = transErr1[index]
        rotErr = rotErr1[index]
        successRate = successRate1[index]
        stdTrans = stdTrans1[index]
        stdRot = stdRot1[index]
        time = time1[index]
        # print(transErr)
        
        del transErr1[0:]
        del rotErr1[0:]
        del successRate1[0:]
        del stdTrans1[0:]
        del stdRot1[0:]
        del time1[0:]
        VPSResult = str(transErr) + " " + str(rotErr) + " " + str(successRate) + " " + str(stdTrans) + " " + str(stdRot) + " " + str(time) + "\n"
        # VPSResult = str(transErr) + " " + str(rotErr) + " " + str(successRate) + " " + str(stdTrans) + " " + str(stdRot) + "\n"
        # print(VPSResult)
        
        writefile(resultFilePath, compressionResult, VPSResult)

