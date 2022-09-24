#!/bin/bash

# for debug
# compressionRatioArr=(0.03)
# mapSeqArr=(03)

# Common 
# mapSeqArr=(01 02 03)
mapSeqArr=(03)
querySeqArr=(02)

# Comprssion
compressionRatioArr=(0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15 0.10 0.08 0.06 0.05 0.04 0.03) # 15
# compressionRatioArr=(0.8 0.6 0.4 0.2 0.10 0.08 0.05 0.03) # 8

saveFileName1="build/Db/MH"
saveFileName2="_Landmark_compression_test_"
saveFileName3=".bin"
saveFileName4="build/Db/MH"

loadMapName1="/home/ohdonghoon/NewEuroC/MH"
loadMapName2="/MH"
loadMapName3="_ba.bin"

# VPS run
initialFilePath="/home/ohdonghoon/NewEuroC/MH"
initialFilePath2="/home/ohdonghoon/NewEuroC/timeStamp_0.25,2/MH"
queryImg="/RectCam0_for_EsPose"
timeCen="/MH"
timeCen2="_MH"
queryTime="_timeStamp.txt"
dbCen="db_MH"
queryGt="cam0Pose.txt"

initalMapPath="./build/Db/MH"
mapCen="_Landmark_compression_test_"

endMapPath=".bin_1._1._1._1._.bin"

# Coeff
CompressionRatio=(0.10)
coeff=(2. 3. 4. 5. 6. 7. 8.) # 7
# coeff=(2. 5. 8.)

initalMapPath1="./build/Db/MH"
endMapPath1=".bin_"
endMapPath2=".bin"

coeffOri="1._"
coeffName=(2._ 3._ 4._ 5._ 6._ 7._ 8._)
# coeffName=(2._ 5._ 8._)

# argv
runCoeff="coeff"
runComp="comp"

# echo ${1} $runCoeff $runComp

##################################################################################################
############################### Compression!!!!!!!!!!!!!!!!! ######################################

if [ "${1}" == "$runComp" ]
then 
    
    # echo "Run comp mode ... "


    # for seq in "${mapSeqArr[@]}"
    # do    
    #     sequenceName=$seq

    #     for var in "${compressionRatioArr[@]}"
    #     do
    #         compressionVar=$var
    #         saveFileName="$saveFileName1$sequenceName$saveFileName2$compressionVar$saveFileName3"
    #         loadMapName="$loadMapName1$sequenceName$loadMapName2$sequenceName$loadMapName3"
    #         # echo $saveFileName $loadMapName
            
    #         ./Compression/landmarkCompression ${loadMapName} ${var} ${saveFileName} 1. 1. 1. 1.
    #     done
    # done
        
    # ######################################################################################################
    # #####################################################################################################
        
    # #################################################################################################
    # ############################## Run VPS !!!!!!!!!!!!!!!!! ######################################

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
        done
    done

    ######################################################################################################
    #####################################################################################################
fi







#####################################################################################################
######################### Coeff Compression!!!!!!!!!!!!!!!!! ######################################


if [ ${1} == ${runCoeff} ]
then 
    
    echo "Run coeff mode ... "


    # for seq in "${mapSeqArr[@]}"
    # do 
    #     for coeff1 in "${coeff[@]}"
    #     do
    #         sequenceName=$seq

    #         for var in "${CompressionRatio[@]}"
    #         do
    #             compressionVar=$var
    #             saveFileName="$saveFileName4$sequenceName$saveFileName2$compressionVar$saveFileName3"
    #             loadMapName="$loadMapName1$sequenceName$loadMapName2$sequenceName$loadMapName3"
    #             # echo $saveFileName $loadMapName
                
    #             ./Compression/landmarkCompression ${loadMapName} ${var} ${saveFileName} ${coeff1} 1. 1. 1.
    #             ./Compression/landmarkCompression ${loadMapName} ${var} ${saveFileName} 1. ${coeff1} 1. 1.
    #             ./Compression/landmarkCompression ${loadMapName} ${var} ${saveFileName} 1. 1. ${coeff1} 1.
    #             ./Compression/landmarkCompression ${loadMapName} ${var} ${saveFileName} 1. 1. 1. ${coeff1}
    #         done    
    #     done
    # done

    ######################################################################################################
    #####################################################################################################

    ################################################################################################
    ############################## Run VPS !!!!!!!!!!!!!!!!! ######################################

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

            for var in "${CompressionRatio[@]}"
            do
                
                for coeff1 in "${coeffName[@]}"
                do
                    coeff2=$coeff1
                    mapDbName1="$initalMapPath1$mapSeq$mapCen$var$endMapPath1$coeff2$coeffOri$coeffOri$coeffOri$endMapPath2"
                    mapDbName2="$initalMapPath1$mapSeq$mapCen$var$endMapPath1$coeffOri$coeff2$coeffOri$coeffOri$endMapPath2"
                    mapDbName3="$initalMapPath1$mapSeq$mapCen$var$endMapPath1$coeffOri$coeffOri$coeff2$coeffOri$endMapPath2"
                    mapDbName4="$initalMapPath1$mapSeq$mapCen$var$endMapPath1$coeffOri$coeffOri$coeffOri$coeff2$endMapPath2"

                    ./Examples/Monocular/mono_euroc \
                    ./Vocabulary/ORBvoc.txt \
                    ./Examples/Monocular/EuRoC.yaml \
                    ${queryImgName} \
                    ${queryTimeStampsName} \
                    ${queryGTName} \
                    ${mapDbName1}

                    ./Examples/Monocular/mono_euroc \
                    ./Vocabulary/ORBvoc.txt \
                    ./Examples/Monocular/EuRoC.yaml \
                    ${queryImgName} \
                    ${queryTimeStampsName} \
                    ${queryGTName} \
                    ${mapDbName2}

                    ./Examples/Monocular/mono_euroc \
                    ./Vocabulary/ORBvoc.txt \
                    ./Examples/Monocular/EuRoC.yaml \
                    ${queryImgName} \
                    ${queryTimeStampsName} \
                    ${queryGTName} \
                    ${mapDbName3}

                    ./Examples/Monocular/mono_euroc \
                    ./Vocabulary/ORBvoc.txt \
                    ./Examples/Monocular/EuRoC.yaml \
                    ${queryImgName} \
                    ${queryTimeStampsName} \
                    ${queryGTName} \
                    ${mapDbName4}                
                    
                    # echo $mapSeq $querySeq
                    # echo $queryImgName
                    # echo $queryTimeStampsName
                    # echo $queryGTName
                    # echo $mapDbName1
                    # echo $mapDbName2
                    # echo $mapDbName3
                    # echo $mapDbName4

                done           
            done 
        done
    done

fi

######################################################################################################
#####################################################################################################

            
            






# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH01/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH01/MH01_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH01cam0Pose.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01_ba.bin

# ./Examples/Monocular/mono_euroc2 \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/ORB_LocalizationTest/result/imageEtc/MH03_MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH03/MH03db_MH02cam0Pose.txt \
#     /home/ohdonghoon/NewEuroC/MH03/MH03_ba.bin

# ./Examples/Monocular/mono_euroc2 \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH03/MH03_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH03cam0Pose.txt \
#     /home/ohdonghoon/NewEuroC/MH03/MH03_ba.bin

# Landmark Compression
# ./Compression/compression build/MH01_ba.bin 0.9 build/MH01_Landmark_compression_test_90.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.8 build/MH01_Landmark_compression_test_80.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.7 build/MH01_Landmark_compression_test_70.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.6 build/MH01_Landmark_compression_test_60.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.5 build/MH01_Landmark_compression_test_50.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.4 build/MH01_Landmark_compression_test_40.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.3 build/MH01_Landmark_compression_test_30.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Landmark_compression_test_20.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.15 build/MH01_Landmark_compression_test_15.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.10 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.08 build/MH01_Landmark_compression_test_08.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.06 build/MH01_Landmark_compression_test_06.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.05 build/MH01_Landmark_compression_test_05.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.04 build/MH01_Landmark_compression_test_04.bin 1. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.03 build/MH01_Landmark_compression_test_03.bin 1. 1. 1. 1.


# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 2.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 3.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 4.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 5.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 6.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 7.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 1. 8.

# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 2. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 3. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 4. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 5. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 6. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 7. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 1. 8. 1.

# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 2. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 3. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 4. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 5. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 6. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 7. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 8. 1. 1.


# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 2. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 3. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 4. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 5. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 6. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 7. 1. 1. 1.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 8. 1. 1. 1.


# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Landmark_compression_test_20.bin 1. 1. 1. 3.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Landmark_compression_test_20.bin 0. 0. 0. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Landmark_compression_test_20.bin 1. 0. 0. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Landmark_compression_test_20.bin 0. 1. 0. 1.
# ./Compression/compression build/MH01_ba.bin 0.2 build/MH01_Landmark_compression_test_20.bin 0. 0. 1. 1.

        # plus
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 1. 0. 0. 0.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 0. 1. 0. 0.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 0. 0. 1. 0.
# ./Compression/compression build/MH01_ba.bin 0.1 build/MH01_Landmark_compression_test_10.bin 0. 0. 0. 1.



# run VPS (landmark compression)

# ./Examples/Monocular/mono_euroc2 \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH03/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH03/MH03_MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/Db/MH03_KeyframeSimilarity_compression_test_0.9.bin_1._1._1._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_80.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_70.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_60.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_50.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_40.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_30.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_20.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_15.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_08.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_06.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_05.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_04.bin_1._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_03.bin_1._1._1._1._.bin






# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._2._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._3._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._4._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._5._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._6._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._7._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._1._8._.bin


# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._2._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._3._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._4._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._5._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._6._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._7._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._1._8._1._.bin



# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._2._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._3._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._4._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._5._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._6._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._7._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._8._1._1._.bin



# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_2._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_3._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_4._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_5._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_6._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_7._1._1._1._.bin
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_8._1._1._1._.bin



# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_20.bin_1._1._1._3._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_20.bin_0._0._0._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_20.bin_1._0._0._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_20.bin_0._1._0._1._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_20.bin_0._0._1._1._.bin


        #plus
# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_1._0._0._0._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_0._1._0._0._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_0._0._1._0._.bin

# ./Examples/Monocular/mono_euroc \
#     ./Vocabulary/ORBvoc.txt \
#     ./Examples/Monocular/EuRoC.yaml \
#     /home/ohdonghoon/NewEuroC/MH02/RectCam0_for_EsPose \
#     /home/ohdonghoon/NewEuroC/MH02/MH02_timeStamp.txt \
#     /home/ohdonghoon/NewEuroC/MH01/MH01db_MH02cam0Pose.txt \
#     ./build/MH01_Landmark_compression_test_10.bin_0._0._0._1._.bin

