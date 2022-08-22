#!/bin/bash

mapSeqArr=(01 02 03)
compressionRatioArr=(0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15 0.10 0.08 0.06 0.05 0.04 0.03) # 15
# compressionRatioArr=(0.8 0.6 0.4 0.2 0.10 0.08 0.05 0.03) # 8
# compressionRatioArrtest=(0.9 0.5)

# ##################################################################################################
# ############################## Compression!!!!!!!!!!!!!!!!! ######################################

# for seq in "${mapSeqArr[@]}"
# do    
#     saveFileName1="build/Db/MH"
#     saveFileName2="_KeyframeILP_compression_test_"
#     saveFileName3=".bin"
    
#     sequenceName=$seq
    
#     loadMapName1="/home/ohdonghoon/NewEuroC/MH"
#     loadMapName2="/MH"
#     loadMapName3="_ba.bin"

#     for var in "${compressionRatioArr[@]}"
#     do
#         compressionVar=$var
#         saveFileName="$saveFileName1$sequenceName$saveFileName2$compressionVar$saveFileName3"
#         loadMapName="$loadMapName1$sequenceName$loadMapName2$sequenceName$loadMapName3"
#         # echo $saveFileName $loadMapName
        
#         ./Compression/keyframeILPCompression ${loadMapName} ${var} ${saveFileName} 1. 1. 1. 1.
#     done
# done

# ######################################################################################################
# #####################################################################################################

# #################################################################################################
# ############################## Run VPS !!!!!!!!!!!!!!!!! ######################################

initialFilePath="/home/ohdonghoon/NewEuroC/MH"
queryImg="/RectCam0_for_EsPose"
timeCen="/MH"
timeCen2="_MH"
queryTime="_timeStamp.txt"
dbCen="db_MH"
queryGt="cam0Pose.txt"

initalMapPath="./build/Db/MH"
mapCen="_KeyframeILP_compression_test_"
endMapPath=".bin_1._1._1._1._.bin"

for mapSeq in "${mapSeqArr[@]}"
do
    
    for querySeq in "${mapSeqArr[@]}"
    do
        if(($mapSeq == $querySeq))
        then
            continue
        fi

        queryImgName="$initialFilePath$querySeq$queryImg"
        queryTimeStampsName="$initialFilePath$mapSeq$timeCen$mapSeq$timeCen2$querySeq$queryTime"
        queryGTName="$initialFilePath$mapSeq$timeCen$mapSeq$dbCen$querySeq$queryGt"

        for var in "${compressionRatioArr[@]}"
        do
            mapDbName="$initalMapPath$mapSeq$mapCen$var$endMapPath"

            ./Examples/Monocular/mono_euroc \
            ./Vocabulary/ORBvoc.txt \
            ./Examples/Monocular/EuRoC.yaml \
            ${queryImgName} \
            ${queryTimeStampsName} \
            ${queryGTName} \
            ${mapDbName}
            # echo $mapSeq $querySeq
            # echo $queryImgName
            # echo $queryTimeStampsName
            # echo $queryGTName
            # echo $mapDbName           
        done 
        # echo $mapSeq $querySeq
    done
done

# Keyframe Compression
# ./Compression/compression build/MH01_ba.bin 0.9 build/MH01_Keyframe_compression_test_90.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.8 build/MH01_Keyframe_compression_test_80.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.7 build/MH01_Keyframe_compression_test_70.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.6 build/MH01_Keyframe_compression_test_60.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.5 build/MH01_Keyframe_compression_test_50.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.4 build/MH01_Keyframe_compression_test_40.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.3 build/MH01_Keyframe_compression_test_30.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.15 build/MH01_Keyframe_compression_test_15.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.10 build/MH01_Keyframe_compression_test_10.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.08 build/MH01_Keyframe_compression_test_08.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.06 build/MH01_Keyframe_compression_test_06.bin 1. 1. 1. 1.

#        # plus 
# ./Compression/compression build/MH01_ba.bin 0.05 build/MH01_Keyframe_compression_test_05.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.04 build/MH01_Keyframe_compression_test_04.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.03 build/MH01_Keyframe_compression_test_03.bin 1. 1. 1. 1.


# ./Compression/compression build/MH01_ba.bin 0.9 build/MH01_Keyframe_compression_test_90.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.8 build/MH01_Keyframe_compression_test_80.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.7 build/MH01_Keyframe_compression_test_70.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.6 build/MH01_Keyframe_compression_test_60.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.5 build/MH01_Keyframe_compression_test_50.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.4 build/MH01_Keyframe_compression_test_40.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.3 build/MH01_Keyframe_compression_test_30.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.15 build/MH01_Keyframe_compression_test_15.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.10 build/MH01_Keyframe_compression_test_10.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.08 build/MH01_Keyframe_compression_test_08.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.06 build/MH01_Keyframe_compression_test_06.bin 3. 1. 1. 1.

#        # plus 
# ./Compression/compression build/MH01_ba.bin 0.05 build/MH01_Keyframe_compression_test_05.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.04 build/MH01_Keyframe_compression_test_04.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.03 build/MH01_Keyframe_compression_test_03.bin 3. 1. 1. 1.

# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 1. 1. 2.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 1. 2. 2.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 2. 1. 2.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 2. 2. 2.

# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 2. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 2. 1. 2. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 2. 2. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 2. 2. 2. 1.

# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 1. 1. 3.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 0. 0. 0. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 1. 0. 0. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 0. 1. 0. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Keyframe_compression_test_20.bin 0. 0. 1. 1.





#############################################################
# run VPS (keyframe compression)

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH03/MH03_MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH03/MH03db_MH02cam0Pose.txt \
#     ./build/Db/MH03_Keyframe_compression_test_0.05.bin_1._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_80.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_70.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_60.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_50.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH01/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_MH01_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH02/MH02db_MH01cam0Pose.txt \
#     ./build/KeyframeIQP/MH02_Keyframe_compression_test_0.5.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_40.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_30.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_15.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_10.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_08.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_06.bin_1._1._1._1._.bin

#                 # plus
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_05.bin_1._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_04.bin_1._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_03.bin_1._1._1._1._.bin

#################################################################
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_90.bin_3._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_80.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_70.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_60.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_50.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_40.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_30.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_15.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_10.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_08.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_06.bin_3._1._1._1._.bin

#                 # plus
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_05.bin_3._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_04.bin_3._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_03.bin_3._1._1._1._.bin



# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._1._1._2._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._1._2._2._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._2._1._2._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._2._2._2._.bin




# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_2._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_2._1._2._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_2._2._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_2._2._2._1._.bin



# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._1._1._3._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_0._0._0._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_1._0._0._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_0._1._0._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Keyframe_compression_test_20.bin_0._0._1._1._.bin


