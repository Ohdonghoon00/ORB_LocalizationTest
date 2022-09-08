#!/bin/bash

mapSeqArr=(01 02 03)
compressionRatioArr=(0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15 0.10 0.08 0.06 0.05 0.04 0.03) # 15
# compressionRatioArr=(0.8 0.6 0.4 0.2 0.10 0.08 0.05 0.03) # 8
# compressionRatioArrtest=(0.9 0.5)

# ##################################################################################################
# ############################## Compression!!!!!!!!!!!!!!!!! ######################################

for seq in "${mapSeqArr[@]}"
do    
    saveFileName1="build/Db/MH"
    saveFileName2="_KeyframeIQP_compression_test_"
    saveFileName3=".bin"
    
    sequenceName=$seq
    
    loadMapName1="/home/ohdonghoon/NewEuroC/MH"
    loadMapName2="/MH"
    loadMapName3="_ba.bin"

    for var in "${compressionRatioArr[@]}"
    do
        compressionVar=$var
        saveFileName="$saveFileName1$sequenceName$saveFileName2$compressionVar$saveFileName3"
        loadMapName="$loadMapName1$sequenceName$loadMapName2$sequenceName$loadMapName3"
        # echo $saveFileName $loadMapName
        
        ./Compression/keyframeIQPCompression ${loadMapName} ${var} ${saveFileName} 1. 1. 1. 1.
    done
done

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
mapCen="_KeyframeIQP_compression_test_"
endMapPath=".bin_1._1._1._1._.bin"

# for mapSeq in "${mapSeqArr[@]}"
# do
    
#     for querySeq in "${mapSeqArr[@]}"
#     do
#         if(($mapSeq == $querySeq))
#         then
#             continue
#         fi

#         queryImgName="$initialFilePath$querySeq$queryImg"
#         queryTimeStampsName="$initialFilePath$mapSeq$timeCen$mapSeq$timeCen2$querySeq$queryTime"
#         queryGTName="$initialFilePath$mapSeq$timeCen$mapSeq$dbCen$querySeq$queryGt"

#         for var in "${compressionRatioArr[@]}"
#         do
#             mapDbName="$initalMapPath$mapSeq$mapCen$var$endMapPath"

#             ./Examples/Monocular/mono_euroc \
#             ./Vocabulary/ORBvoc.txt \
#             ./Examples/Monocular/EuRoC.yaml \
#             ${queryImgName} \
#             ${queryTimeStampsName} \
#             ${queryGTName} \
#             ${mapDbName}
#             # echo $mapSeq $querySeq
#             # echo $queryImgName
#             # echo $queryTimeStampsName
#             # echo $queryGTName
#             # echo $mapDbName           
#         done 
#         # echo $mapSeq $querySeq
#     done
# done