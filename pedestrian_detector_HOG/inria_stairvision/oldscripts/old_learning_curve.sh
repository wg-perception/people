pushd ~/stair/lasik

export TEST_SEQ=/home/ia3n/2008-09-take2/sequences/2008-09-09-1.xml
export TEST_LAB=/home/ia3n/2008-09-take2/sequences/resized-2008-09-09-1-labels.xml

export NAME=-morerounds

#Make room to build the set of test windows
rm -r -f data/objects${NAME}-direct-test/person
rm -r -f data/objects${NAME}-direct-test/negative
mkdir data/objects${NAME}-direct-test
mkdir data/objects${NAME}-direct-test/person
mkdir data/objects${NAME}-direct-test/negative



#Build the test set
bin/buildTrainingImageDataset -object person -resize 32 64 -directory data/objects${NAME}-direct-test ${TEST_SEQ} ${TEST_LAB}

cat learning_curve_points.txt | while read line; do
    echo "At learning curve point $line"

    #Choose sequence to train on
    export SEQ=/home/ia3n/2008-09-take2/sequences/restricted/merge234${line}.xml
    export LAB=/home/ia3n/2008-09-take2/sequences/restricted/resized-merge234${line}-label.xml

    #Make room for new dataset
    rm -r -f data/objects${NAME}${line}/person
    rm -r -f data/objects${NAME}${line}/negative
    mkdir data/objects${NAME}${line}
    mkdir data/objects${NAME}${line}/person
    mkdir data/objects${NAME}${line}/negative

    #Build the new training set
    echo "Building training image dataset..."
    bin/buildTrainingImageDataset -object person -resize 32 64 -directory data/objects${NAME}${line} ${SEQ} ${LAB}

    #Build the patch dictionary
    echo "Building patch dictionary..."
     bin/buildPatchDictionary -ch INTENSITY .bmp -ch EDGE .bmp -n 10 -o models/person${NAME}${line}.dictionary.xml data/objects${NAME}${line}/person 32 64

    #Clear the old patch response cache
    echo "Building patch response cache..."
    rm -r -f ~/tmp_cache-person-pos${NAME}${line}
    rm -r -f ~/tmp_cache-person-neg${NAME}${line}
    mkdir ~/tmp_cache-person-pos${NAME}${line}/
    mkdir ~/tmp_cache-person-neg${NAME}${line}/

    #Build the patch response cache. I have taken out the limit on the number of images. first arg used to be -maxImages 1000
    date +%s;bin/buildPatchResponseCache -ch INTENSITY .bmp -ch EDGE .bmp -maxImages 60000 data/objects${NAME}${line}/person ~/tmp_cache-person-pos${NAME}${line} models/person${NAME}${line}.dictionary.xml; date +%s
    bin/buildPatchResponseCache -ch INTENSITY .bmp -ch EDGE .bmp -maxImages 60000 data/objects${NAME}${line}/negative ~/tmp_cache-person-neg${NAME}${line} models/person${NAME}${line}.dictionary.xml

    #Train the boosted detector
    bin/trainObjectDetector -shuffle -cvn 3 -o models/person${NAME}${line}.model ~/tmp_cache-person-pos${NAME}${line} ~/tmp_cache-person-neg${NAME}${line} 

    #Trim the dictionary
    cp models/person${NAME}${line}.dictionary.xml models/person${NAME}${line}.dictionary-trimmed.xml
    cp models/person${NAME}${line}.model models/person${NAME}${line}-trimmed.model
    svl/scripts/trimDictionary.pl models/person${NAME}${line}.dictionary-trimmed.xml models/person${NAME}${line}-trimmed.model

    #Evaluate performance on the test set
    bin/classifyImages -n person -o ~/tmp_detections${NAME}${line}.xml -t 0.01 -v -baseExt .bmp -ch INTENSITY .bmp -ch EDGE .bmp models/person${NAME}${line}.dictionary-trimmed.xml models/person${NAME}${line}-trimmed.model ${TEST_SEQ}
    bin/scoreDetections -pr experiments/pr${NAME}${line} ${TEST_LAB} ~/tmp_detections${NAME}${line}.xml
    bin/scoreDetections -allowDoubles -pr experiments/pr-allowDoubles-${NAME}${line} ${TEST_LAB} ~/tmp_detections${NAME}${line}.xml
    bin/scoreDetections -allowDoubles -noNonmax -pr experiments/pr-allowDoubles-noNonmax-${NAME}${line} ${TEST_LAB} ~/tmp_detections${NAME}${line}.xml

    #Evaluate performance on the train set
    bin/classifyImages -n person -o ~/tmp_detections_train${NAME}${line}.xml -t 0.01 -v -baseExt .bmp -ch INTENSITY .bmp -ch EDGE .bmp models/person${NAME}${line}.dictionary-trimmed.xml models/person${NAME}${line}-trimmed.model ${SEQ}
    bin/scoreDetections -pr experiments/pr-train${NAME}${line} ${LAB} ~/tmp_detections_train${NAME}${line}.xml
    bin/scoreDetections -allowDoubles -pr experiments/pr-allowDoubles-train${NAME}${line} ${LAB} ~/tmp_detections_train${NAME}${line}.xml
    bin/scoreDetections -noNonmax -allowDoubles -pr experiments/pr-allowDoubles-noNonmax-train${NAME}${line} ${LAB} ~/tmp_detections_train${NAME}${line}.xml

 #Clear the old patch response cache for training set (Make a new one with trimmed dictionary)
    echo "Building train patch response cache..."
    rm -r -f ~/tmp_cache-person-pos${NAME}${line}-direct-train
    rm -r -f ~/tmp_cache-person-neg${NAME}${line}-direct-train
    mkdir ~/tmp_cache-person-pos${NAME}${line}-direct-train
    mkdir ~/tmp_cache-person-neg${NAME}${line}-direct-train

    #Build the patch response cache. I have taken out the limit on the number of images. first arg used to be -maxImages 1000
    bin/buildPatchResponseCache -ch INTENSITY .bmp -ch EDGE .bmp -maxImages 6000000 data/objects${NAME}${line}/person ~/tmp_cache-person-pos${NAME}${line}-direct-train models/person${NAME}${line}.dictionary-trimmed.xml
    bin/buildPatchResponseCache -ch INTENSITY .bmp -ch EDGE .bmp -maxImages 6000000 data/objects${NAME}${line}/negative ~/tmp_cache-person-neg${NAME}${line}-direct-train models/person${NAME}${line}.dictionary-trimmed.xml


 #Clear the old patch response cache for test set
    echo "Building test patch response cache..."
    rm -r -f ~/tmp_cache-person-pos${NAME}${line}-direct-test
    rm -r -f ~/tmp_cache-person-neg${NAME}${line}-direct-test
    mkdir ~/tmp_cache-person-pos${NAME}${line}-direct-test
    mkdir ~/tmp_cache-person-neg${NAME}${line}-direct-test

bin/buildPatchResponseCache -ch INTENSITY .bmp -ch EDGE .bmp -maxImages 6000000 data/objects${NAME}-direct-test/person ~/tmp_cache-person-pos${NAME}${line}-direct-test models/person${NAME}${line}.dictionary-trimmed.xml
    bin/buildPatchResponseCache -ch INTENSITY .bmp -ch EDGE .bmp -maxImages 6000000 data/objects${NAME}-direct-test/negative ~/tmp_cache-person-neg${NAME}${line}-direct-test models/person${NAME}${line}.dictionary-trimmed.xml


    #Evaluate the boosted detector on a per-window basis
    bin/trainObjectDetector -maxInstances 6000000 -i models/person${NAME}${line}.dictionary-trimmed.xml models/person${NAME}${line}-trimmed.model ~/tmp_cache-person-pos${NAME}${line}-direct-train ~/tmp_cache-person-neg${NAME}${line}-direct-train >& experiments/result-${NAME}${line}-train.txt
    bin/trainObjectDetector -maxInstances 6000000 -i models/person${NAME}${line}.dictionary-trimmed.xml models/person${NAME}${line}-trimmed.model ~/tmp_cache-person-pos${NAME}${line}-direct-test ~/tmp_cache-person-neg${NAME}${line}-direct-test >& experiments/result-${NAME}${line}-test.txt
done

popd
