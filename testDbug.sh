#!/bin/bash

# ./Examples/Monocular/mono_euroc     ./Vocabulary/ORBvoc.txt     ./Examples/Monocular/EuRoC.yaml     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose     /home/ohdonghoon/NewEuroC/MH02/MH02_MH03_timeStamp.txt     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH03cam0Pose.txt     ./build/Db/MH02_KeyframeILP_compression_test_0.4.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc     ./Vocabulary/ORBvoc.txt     ./Examples/Monocular/EuRoC.yaml     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose     /home/ohdonghoon/NewEuroC/MH02/MH02_MH03_timeStamp.txt     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH03cam0Pose.txt     ./build/Db/MH02_KeyframeILP_compression_test_0.4.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc     ./Vocabulary/ORBvoc.txt     ./Examples/Monocular/EuRoC.yaml     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose     /home/ohdonghoon/NewEuroC/MH02/MH02_MH03_timeStamp.txt     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH03cam0Pose.txt     ./build/Db/MH02_KeyframeILP_compression_test_0.4.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc     ./Vocabulary/ORBvoc.txt     ./Examples/Monocular/EuRoC.yaml     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose     /home/ohdonghoon/NewEuroC/MH02/MH02_MH03_timeStamp.txt     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH03cam0Pose.txt     ./build/Db/MH02_KeyframeILP_compression_test_0.4.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc     ./Vocabulary/ORBvoc.txt     ./Examples/Monocular/EuRoC.yaml     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose     /home/ohdonghoon/NewEuroC/MH02/MH02_MH03_timeStamp.txt     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH03cam0Pose.txt     ./build/Db/MH02_KeyframeILP_compression_test_0.4.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc     ./Vocabulary/ORBvoc.txt     ./Examples/Monocular/EuRoC.yaml     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose     /home/ohdonghoon/NewEuroC/MH02/MH02_MH03_timeStamp.txt     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH03cam0Pose.txt     ./build/Db/MH02_KeyframeILP_compression_test_0.4.bin_1._1._1._1._.bin


# for debug
# compressionRatioArr=(0.03)
# mapSeqArr=(03)

# Common 
mapSeqArr=(01 02 03)

# Comprssion
compressionRatioArr=(0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15 0.10 0.08 0.06 0.05 0.04 0.03) # 15
# compressionRatioArr=(0.8 0.6 0.4 0.2 0.10 0.08 0.05 0.03) # 8

saveFileName1="build/Db/MH"
saveFileName2="_LandmarkIQP_compression_test_"
saveFileName3=".bin"
saveFileName4="build/Db/MH"

loadMapName1="/home/ohdonghoon/NewEuroC/MH"
loadMapName2="/MH"
loadMapName3="_ba.bin"

    for seq in "${mapSeqArr[@]}"
    do    
        sequenceName=$seq

        for var in "${compressionRatioArr[@]}"
        do
            compressionVar=$var
            saveFileName="$saveFileName1$sequenceName$saveFileName2$compressionVar$saveFileName3"
            loadMapName="$loadMapName1$sequenceName$loadMapName2$sequenceName$loadMapName3"
            # echo $saveFileName $loadMapName
            
            ./Compression/landmarkIQPCompression ${loadMapName} ${var} ${saveFileName} 1. 1. 1. 1.
        done
    done