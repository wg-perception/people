
pushd ~/stair/lasik

export TEST_SEQ=/afs/cs/u/ia3n/willow_data/dataset_seq_test.xml
export TEST_LAB=/afs/cs/u/ia3n/willow_data/dataset_labels_test.xml

export NAME=-willow

export BIN=/afs/cs/u/ia3n/willow_data/bin/bin${NAME}

rm -r -f ${BIN} #if this has been run before, it will still contain outdated code

echo "Copying binaries"
cp -r bin ${BIN}

echo "Building test set"
#Make room to build the set of test windows
rm -r -f data/objects${NAME}-direct-test/cups
rm -r -f data/objects${NAME}-direct-test/negative
mkdir data/objects${NAME}-direct-test
mkdir data/objects${NAME}-direct-test/cups
mkdir data/objects${NAME}-direct-test/negative

#Build the test set
#note-- dmMatchImg will also be needed for classifyImages, try to implement it in such a way that it will work in both
#places
echo "call 1"
${BIN}/buildTrainingImageDataset -set svlVision.svlImageLoader channels "INTENSITY .png" -set svlVision.svlImageLoader defaultExtension .png -set svlVision.svlTrainingDatasetBuilder objects cups -set svlVision.svlTrainingDatasetBuilder resizeWidth 32 -set svlVision.svlTrainingDatasetBuilder resizeHeight 64 -set svlVision.svlTrainingDatasetBuilder baseDir data/objects${NAME}-direct-test ${TEST_SEQ} ${TEST_LAB} 
echo "/call 1"

cat learning_curve_points.txt | while read line; do
#export line=-1.0
    echo "At learning curve point $line"

    #Choose sequence to train on
    export SEQ=~/willow_data/dataset_seq_train${line}.xml
    export LAB=~/willow_data/dataset_labels_train${line}.xml

    #Make room for new dataset
    rm -r -f data/objects${NAME}${line}/cups
    rm -r -f data/objects${NAME}${line}/negative
    mkdir data/objects${NAME}${line}
    mkdir data/objects${NAME}${line}/cups
    mkdir data/objects${NAME}${line}/negative

    #Build the new training set
    echo "Building training image dataset..."
    ${BIN}/buildTrainingImageDataset -set svlVision.svlImageLoader channels "INTENSITY .png" -set svlVision.svlImageLoader defaultExtension .png -set svlVision.svlTrainingDatasetBuilder objects cups -set svlVision.svlTrainingDatasetBuilder resizeWidth 32 -set svlVision.svlTrainingDatasetBuilder resizeHeight 64 -set svlVision.svlTrainingDatasetBuilder baseDir data/objects${NAME}${line} ${SEQ} ${LAB}

    #Build the patch dictionary
    echo "Building patch dictionary..."
     ${BIN}/buildPatchDictionary -dmsize 32 64 -ch INTENSITY .png -ch EDGE .png -n 10 -o models/cups${NAME}${line}.dictionary.xml data/objects${NAME}${line}/cups/ 32 64

     #Build the overall feature extractor
#echo "Calling extractorCat"
     #${BIN}/extractorCat models/cups${NAME}${line}.dictionary.xml models/haar_ave.xml -o models/cups${NAME}${line}.composite.xml


    #Clear the old patch response cache
    echo "Building patch response cache..."
    rm -r -f /tmp/cache-cups-pos${NAME}${line}
    rm -r -f /tmp/cache-cups-neg${NAME}${line}
    mkdir /tmp/cache-cups-pos${NAME}${line}/
    mkdir /tmp/cache-cups-neg${NAME}${line}/

    #Build the patch response cache. I have taken out the limit on the number of images. first arg used to be -maxImages 1000
echo "First call to buildPatchResponseCache"
    ${BIN}/buildPatchResponseCache -dmsize 32 64 -set svlVision.svlImageLoader channels "INTENSITY .png EDGE .png" -maxImages 60000 data/objects${NAME}${line}/cups/ /tmp/cache-cups-pos${NAME}${line} models/cups${NAME}${line}.dictionary.xml
echo "Second call to buildPatchResponseCache"
    ${BIN}/buildPatchResponseCache -v -dmsize 32 64 -set svlVision.svlImageLoader channels "INTENSITY .png EDGE .png" -maxImages 60000 data/objects${NAME}${line}/negative/ /tmp/cache-cups-neg${NAME}${line} models/cups${NAME}${line}.dictionary.xml

    #Train the boosted detector
echo "Training boosted detector"
    ${BIN}/trainObjectDetector -v -f -shuffle -cvn 3 -set svlML.svlBoostedClassifier boostingRounds 300 -o models/cups${NAME}${line}.model /tmp/cache-cups-pos${NAME}${line} /tmp/cache-cups-neg${NAME}${line} >& experiments/result-${NAME}${line}-train.txt
echo "Done training boosted detector"

    #Trim the dictionary
echo "Trimming the dictionary"
    #${BIN}/trimFeatureExtractor -m models/cups${NAME}${line}.model -ie models/cups${NAME}${line}.dictionary.xml -oe models/cups${NAME}${line}.dictionary-trimmed.xml
	cp models/cups${NAME}${line}.dictionary.xml models/cups${NAME}${line}.dictionary-trimmed.xml
	cp models/cups${NAME}${line}.model models/cups${NAME}${line}-trimmed.model  	
	svl/scripts/trimDictionary.pl models/cups${NAME}${line}.dictionary-trimmed.xml models/cups${NAME}${line}-trimmed.model
echo "Done trimming the dictionary"

    #Evaluate performance on the test set
echo "Evaluating performance on the test set"
    ${BIN}/classifyImages -n cups -o /tmp/detections${NAME}${line}.xml -t 0.5 -v -set svlVision.svlImageLoader channels "INTENSITY .png EDGE .png" models/cups${NAME}${line}.dictionary-trimmed.xml models/cups${NAME}${line}-trimmed.model ${TEST_SEQ}
    ${BIN}/scoreDetections -pr experiments/pr${NAME}${line} ${TEST_LAB} /tmp/detections${NAME}${line}.xml
    #not updated for willow ${BIN}/scoreDetections -allowDoubles -pr experiments/pr-allowDoubles-${NAME}${line} ${TEST_LAB} /tmp/detections${NAME}${line}.xml
    #not update for willow ${BIN}/scoreDetections -allowDoubles -noNonmax -pr experiments/pr-allowDoubles-noNonmax-${NAME}${line} ${TEST_LAB} /tmp/detections${NAME}${line}.xml

    #Evaluate performance on the train set
echo "Evaluating performance on the train set"
    ${BIN}/classifyImages -n cups -o /tmp/detections_train${NAME}${line}.xml -t 0.05 -v -baseExt .png -dmMatchImg -ch INTENSITY .png -ch EDGE .png models/cups${NAME}${line}.dictionary-trimmed.xml models/cups${NAME}${line}.model ${SEQ}
    ${BIN}/scoreDetections -pr experiments/pr-train${NAME}${line} ${LAB} /tmp/detections_train${NAME}${line}.xml
    #not updated for willow ${BIN}/scoreDetections -allowDoubles -pr experiments/pr-allowDoubles-train${NAME}${line} ${LAB} /tmp/detections_train${NAME}${line}.xml
    #not updated for willow ${BIN}/scoreDetections -noNonmax -allowDoubles -pr experiments/pr-allowDoubles-noNonmax-train${NAME}${line} ${LAB} /tmp/detections_train${NAME}${line}.xml


 #Clear the old patch response cache for test set
    echo "Building test patch response cache..."
    rm -r -f /tmp/cache-cups-pos${NAME}${line}-direct-test
    rm -r -f /tmp/cache-cups-neg${NAME}${line}-direct-test
    mkdir /tmp/cache-cups-pos${NAME}${line}-direct-test
    mkdir /tmp/cache-cups-neg${NAME}${line}-direct-test

${BIN}/buildPatchResponseCache -dmsize 32 64 -ch INTENSITY .png -ch EDGE .png -maxImages 6000000 data/objects${NAME}-direct-test/cups /tmp/cache-cups-pos${NAME}${line}-direct-test models/cups${NAME}${line}.dictionary-trimmed.xml
    ${BIN}/buildPatchResponseCache -dmsize 32 64 -ch INTENSITY .png -ch EDGE .png -maxImages 6000000 data/objects${NAME}-direct-test/negative /tmp/cache-cups-neg${NAME}${line}-direct-test models/cups${NAME}${line}.dictionary-trimmed.xml


    #Evaluate the boosted detector on a per-window basis
echo "Doing per-window evaluation"
    ${BIN}/trainObjectDetector -maxInstances 6000000 -i models/cups${NAME}${line}.dictionary-trimmed.xml models/cups${NAME}${line}.model /tmp/cache-cups-pos${NAME}${line}-direct-test /tmp/cache-cups-neg${NAME}${line}-direct-test >& experiments/result-${NAME}${line}-test.txt
done


popd
