export TEST_SEQ=/u/goodfellow/our_datasets/stanford_cart/sequences/2008-09-09-1.xml
export TEST_LAB=/u/goodfellow/our_datasets/stanford_cart/sequences/2008-09-09-1-labels.xml

#name of the experiment
export NAME=-ir_hog	

export OBJ_TYPE=person
export OBJ_WIDTH=64
export OBJ_HEIGHT=128

export DATA_CHANNELS="INTENSITY .bmp"
export PROCESSED_CHANNELS="INTENSITY .bmp";  #if you had an edge map channel, this is where it would go
export BASE_EXTENSION=.bmp


export BIN=bin${NAME}

rm -r -f ${BIN} #if this has been run before, it will still contain outdated code

echo "Copying binaries" #this is so that it's still possible to edit/recompile the code while a long experiment is running
mkdir ${BIN}
cp -r `rospack find stairvision_latest`/build/stairvision-svn/bin/* ${BIN}/
cp -r bin/* ${BIN}/ #copy our binaries second so that our copies of the apps that know about our plugins overwrite the default ones

echo "Building test set"
#Make room to build the set of test windows
export TEST_EXAMPLES_DIR=data/objects${NAME}-direct-test
export TEST_POS_EXAMPLES=${TEST_EXAMPLES_DIR}/${OBJ_TYPE}
export TEST_NEG_EXAMPLES=${TEST_EXAMPLES_DIR}/negative
rm -r -f ${TEST_POS_EXAMPLES}
rm -r -f ${TEST_NEG_EXAMPLES}
mkdir -p ${TEST_POS_EXAMPLES}
mkdir -p ${TEST_NEG_EXAMPLES}

#Build the test set
${BIN}/buildTrainingImageDataset -set svlVision.svlImageLoader channels "${DATA_CHANNELS}" -set svlVision.svlImageLoader defaultExtension ${BASE_EXTENSION} -set svlVision.svlTrainingDatasetBuilder objects ${OBJ_TYPE} -set svlVision.svlTrainingDatasetBuilder resizeWidth ${OBJ_WIDTH} -set svlVision.svlTrainingDatasetBuilder resizeHeight ${OBJ_HEIGHT} -set svlVision.svlTrainingDatasetBuilder baseDir ${TEST_EXAMPLES_DIR} ${TEST_SEQ} ${TEST_LAB} 

cat learning_curve_points.txt | while read line; do
#export line=-0.1
#export line=-1.0
    echo "At learning curve point $line"

    #Choose sequence to train on
    export SEQ=/u/goodfellow/our_datasets/stanford_cart/sequences/restricted/merge234${line}.xml
    export LAB=/u/goodfellow/our_datasets/stanford_cart/sequences/restricted/merge234${line}-label.xml




    #Clear old training set and make sure directories exist for new training set
    export TRAIN_EXAMPLES_DIR=data/objects${NAME}${line}
    export TRAIN_POS_EXAMPLES=${TRAIN_EXAMPLES_DIR}/${OBJ_TYPE}
    export TRAIN_NEG_EXAMPLES=${TRAIN_EXAMPLES_DIR}/negative
    rm -r -f ${TRAIN_POS_EXAMPLES}
    rm -r -f ${TRAIN_NEG_EXAMPLES}
    mkdir -p ${TRAIN_POS_EXAMPLES}
    mkdir -p ${TRAIN_NEG_EXAMPLES}

    #Build the new training set
    echo "Building training image dataset..."
    ${BIN}/buildTrainingImageDataset -set svlVision.svlImageLoader channels "${DATA_CHANNELS}" -set svlVision.svlImageLoader defaultExtension ${BASE_EXTENSION} -set svlVision.svlTrainingDatasetBuilder objects ${OBJ_TYPE} -set svlVision.svlTrainingDatasetBuilder resizeWidth ${OBJ_WIDTH} -set svlVision.svlTrainingDatasetBuilder resizeHeight ${OBJ_HEIGHT} -set svlVision.svlTrainingDatasetBuilder baseDir ${TRAIN_EXAMPLES_DIR} ${SEQ} ${LAB}
 
    #No need to build patch dictionary, let's just benchmark HOG alone for now
    #Build the patch dictionary
    #echo "Building patch dictionary..."
     #${BIN}/buildPatchDictionary -dmsize 64 128 -ch INTENSITY .png -ch EDGE .png -n 10 -o models/cups${NAME}${line}.dictionary.xml data/objects${NAME}${line}/cups/ 32 64

     #Build the overall feature extractor
#echo "Calling extractorCat"
     #${BIN}/extractorCat models/cups${NAME}${line}.dictionary.xml models/haar_ave.xml -o models/cups${NAME}${line}.composite.xml

    export EXTRACTOR_NAME=defaultHogChannel0
    export EXTRACTOR=${EXTRACTOR_NAME}.xml

    export POS_CACHE=~/tmp_cache-${OBJ_TYPE}-pos${NAME}${line}
    export NEG_CACHE=~/tmp_cache-${OBJ_TYPE}-neg${NAME}${line}

    #Clear the old patch response cache
    echo "Building patch response cache..."
    rm -r -f ${POS_CACHE}
    rm -r -f ${NEG_CACHE}
    mkdir -p ${POS_CACHE}
	echo "Making ${NEG_CACHE}"
    mkdir -p ${NEG_CACHE}
	

    #Build the patch response cache. I have taken out the limit on the number of images. first arg used to be -maxImages 1000
echo "First call to buildPatchResponseCache"
    ${BIN}/buildPatchResponseCache -set svlVision.svlImageLoader channels "${PROCESSED_CHANNELS}" -maxImages 60000 ${TRAIN_POS_EXAMPLES} ${POS_CACHE} ${EXTRACTOR}
echo "Second call to buildPatchResponseCache"
    ${BIN}/buildPatchResponseCache -v -set svlVision.svlImageLoader channels "${PROCESSED_CHANNELS}" -maxImages 60000 ${TRAIN_NEG_EXAMPLES} ${NEG_CACHE} ${EXTRACTOR}

export MODEL_NAME=models/${OBJ_TYPE}${NAME}${line}
export MODEL=${MODEL_NAME}.model

mkdir -p model
mkdir -p experiments

    #Train the boosted detector
echo "Training boosted detector"
    ${BIN}/trainObjectDetector -v -f -shuffle -cvn 3 -set svlML.svlBoostedClassifier boostingRounds 300 -o ${MODEL} ${POS_CACHE} ${NEG_CACHE}  >& experiments/result-${NAME}${line}-train.txt
echo "Done training boosted detector"


export MODEL_TRIMMED=${MODEL_NAME}-trimmed.model
export EXTRACTOR_TRIMMED=${EXTRACTOR_NAME}-trimmed.xml

#Can't trim the feature extractor
	cp ${MODEL} ${MODEL_TRIMMED}
	cp ${EXTRACTOR} ${EXTRACTOR_TRIMMED}
	
    #Trim the dictionary
#echo "Trimming the dictionary"
    #${BIN}/trimFeatureExtractor -m models/cups${NAME}${line}.model -ie models/cups${NAME}${line}.dictionary.xml -oe models/cups${NAME}${line}.dictionary-trimmed.xml
#	cp models/${NAME}${line}.dictionary.xml models/cups${NAME}${line}.dictionary-trimmed.xml
#	cp models/cups${NAME}${line}.model models/cups${NAME}${line}-trimmed.model  	
#	svl/scripts/trimDictionary.pl models/cups${NAME}${line}.dictionary-trimmed.xml models/cups${NAME}${line}-trimmed.model
#echo "Done trimming the dictionary"

    #Evaluate performance on the test set
export DETECTION_SAVE_THRESHOLD=0.05

echo "Evaluating performance on the test set"
    export TEST_DETECTIONS=~/tmp_detections${NAME}${line}.xml
    ${BIN}/classifyImages -n ${OBJ_TYPE} -o ${TEST_DETECTIONS} -t ${DETECTION_SAVE_THRESHOLD} -v -set svlVision.svlImageLoader channels "${PROCESSED_CHANNELS}" ${EXTRACTOR_TRIMMED} ${MODEL_TRIMMED} ${TEST_SEQ}
    ${BIN}/scoreDetections -pr experiments/pr${NAME}${line} ${TEST_LAB} ${TEST_DETECTIONS}
    #not updated for willow ${BIN}/scoreDetections -allowDoubles -pr experiments/pr-allowDoubles-${NAME}${line} ${TEST_LAB} /tmp/detections${NAME}${line}.xml
    #not update for willow ${BIN}/scoreDetections -allowDoubles -noNonmax -pr experiments/pr-allowDoubles-noNonmax-${NAME}${line} ${TEST_LAB} /tmp/detections${NAME}${line}.xml

    #Evaluate performance on the train set
echo "Evaluating performance on the train set"
    export TRAIN_DETECTIONS=/tmp/detections_train${NAME}${line}.xml
    ${BIN}/classifyImages -n ${OBJ_TYPE} -o ${TRAIN_DETECTIONS} -t ${DETECTION_SAVE_THRESHOLD} -v -set svlVision.svlImageLoader channels "${PROCESSED_CHANNELS}" ${EXTRACTOR_TRIMMED} ${MODEL_TRIMMED} ${SEQ}
    ${BIN}/scoreDetections -pr experiments/pr-train${NAME}${line} ${LAB} ${TRAIN_DETECTIONS}
    #not updated for willow ${BIN}/scoreDetections -allowDoubles -pr experiments/pr-allowDoubles-train${NAME}${line} ${LAB} /tmp/detections_train${NAME}${line}.xml
    #not updated for willow ${BIN}/scoreDetections -noNonmax -allowDoubles -pr experiments/pr-allowDoubles-noNonmax-train${NAME}${line} ${LAB} /tmp/detections_train${NAME}${line}.xml


 #Clear the old patch response cache for test set
    echo "Building test patch response cache..."
    export POS_CACHE_TEST=/tmp/cache-${OBJ_TYPE}-pos${NAME}${line}-direct-test
    export NEG_CACHE_TEST=/tmp/cache-${OBJ_TYPE}-neg${NAME}${line}-direct-test
    rm -r -f ${POS_CACHE_TEST}
    rm -r -f ${NEG_CACHE_TEST}
    mkdir ${POS_CACHE_TEST}
    mkdir ${NEG_CACHE_TEST}

${BIN}/buildPatchResponseCache -set svlVision.svlImageLoader channels "${PROCESSED_CHANNELS}" -maxImages 6000000 ${TEST_POS_EXAMPLES} ${POS_CACHE_TEST} ${EXTRACTOR_TRIMMED}
    ${BIN}/buildPatchResponseCache -set svlVision.svlImageLoader channels "${PROCESSED_CHANNELS}" -maxImages 6000000 ${TEST_NEG_EXAMPLES} ${NEG_CACHE_TEST} ${EXTRACTOR_TRIMMED}


    #Evaluate the boosted detector on a per-window basis
echo "Doing per-window evaluation"
    ${BIN}/trainObjectDetector -maxInstances 6000000 -i ${EXTRACTOR_TRIMMED} ${MODEL_TRIMMED} ${POS_CACHE_TEST} ${NEG_CACHE_TEST} >& experiments/result-${NAME}${line}-test.txt
done

