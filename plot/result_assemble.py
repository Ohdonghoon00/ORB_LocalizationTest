from matplotlib import pyplot as plt
import numpy as np


# compressionResultPath = [
#     '../result/CompressionResult/keyframe/Compression_Result_0.9_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.8_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.7_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.6_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.5_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.4_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.3_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.2_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.15_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.10_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.08_1._1._1._1._.txt',
#     '../result/CompressionResult/keyframe/Compression_Result_0.06_1._1._1._1._.txt'
# ]

# VPSResultPath = [
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_90.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_80.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_70.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_60.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_50.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_40.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_30.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_20.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_15.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_10.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_08.bin_1._1._1._1._.bin.txt',
#     '../result/VPSResult/keyframe/VPS_ResultMH01_Keyframe_compression_test_06.bin_1._1._1._1._.bin.txt',
# ]

compressionResultPath = [
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt',
    '../result/CompressionResult/landmark/Compression_Result_0.2_1._1._1._1._.txt'
]

VPSResultPath = [
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_90.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_80.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_70.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_60.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_50.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_40.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_30.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_20.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_15.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_10.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_08.bin_1._1._1._1._.bin.txt',
    '../result/VPSResult/landmark/VPS_ResultMH01_Landmark_compression_test_06.bin_1._1._1._1._.bin.txt',
]

def readfile(path):
    
    # read file
    file = open(path, "r")
    
    line = file.readline()

       
    lines = line
    file.close()
    return lines

def writefile(path, result1, result2):
    
    # write file
    file = open(path, "a")
    file.write(result1)
    file.write(result2)
    file.close()



if __name__ == '__main__':

    resultFilePath = 'finalKeyframeCompressionResult.txt'
    for i in range(0, len(compressionResultPath)):
        
        compressionResult = readfile(compressionResultPath[i]) + ' '
        VPSResult = readfile(VPSResultPath[i])
        writefile(resultFilePath, compressionResult, VPSResult)

