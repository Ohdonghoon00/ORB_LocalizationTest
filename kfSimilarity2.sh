#!/bin/bash

# mapSeqArr=(01 02 03)
mapSeqArr=(03)
querySeqArr=(02)
compressionRatioArr=(0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15 0.10 0.08 0.06 0.05 0.04 0.03) # 15
# compressionRatioArr=(0.8 0.6 0.4 0.2 0.10 0.08 0.05 0.03) # 8
# compressionRatioArrtest=(0.9 0.5)

# argv
runCompression="compression"
runVPS="vps"

initialFilePath="/home/ohdonghoon/NewEuroC/MH"
initialFilePath2="/home/ohdonghoon/NewEuroC/timeStamp_0.25,2/MH"
queryImg="/RectCam0_for_EsPose"
timeCen="/MH"
timeCen2="_MH"
queryTime="_timeStamp.txt"
dbCen="db_MH"
queryGt="cam0Pose.txt"

initalMapPath="./build/Db/MH"
mapCen="_KeyframeSimilarity_compression_test_"
endMapPath=".bin_1._1._1._1._.bin"

# ##################################################################################################
# ############################## Compression!!!!!!!!!!!!!!!!! ######################################

if [ "${1}" == "$runCompression" ]
then
    
    echo "Run Compression mode ... "

    for seq in "${mapSeqArr[@]}"
    do    
        saveFileName1="build/Db/MH"
        saveFileName2="_KeyframeSimilarity_compression_test_"
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
            
            ./Compression/keyframeSimilarityCompression ${loadMapName} ${var} ${saveFileName} 1. 1. 1. 1.
        done
    done
fi
# ######################################################################################################
# #####################################################################################################

# #################################################################################################
# ############################## Run VPS !!!!!!!!!!!!!!!!! ######################################



if [ "${1}" == "$runVPS" ]
then

    echo "Run vps mode ... "

    for mapSeq in "${mapSeqArr[@]}"
    do
        
        for querySeq in "${querySeqArr[@]}"
        do
            if(($mapSeq == $querySeq))
            then
                continue
            fi

            queryImgName="$initialFilePath$querySeq$queryImg"
            # queryTimeStampsName="$initialFilePath$mapSeq$timeCen$mapSeq$timeCen2$querySeq$queryTime"
            queryTimeStampsName="$initialFilePath2$mapSeq$timeCen2$querySeq$queryTime"
            queryGTName="$initialFilePath$mapSeq$timeCen$mapSeq$dbCen$querySeq$queryGt"

            for var in "${compressionRatioArr[@]}"
            do
                mapDbName="$initalMapPath$mapSeq$mapCen$var$endMapPath"

                ./Examples/Monocular/mono_euroc2 \
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
fi