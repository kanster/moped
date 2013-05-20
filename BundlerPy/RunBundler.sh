#!/bin/bash
#
# RunBundler.sh
#   copyright 2008 Noah Snavely
#   Modified by Alvaro Collet
#
# A script for preparing a set of image for use with the Bundler 
# structure-from-motion system.
#
# Usage: RunBundler.sh [image_dir] [output_dir]
#
# The image_dir argument is the directory containing the input images.
# If image_dir is omitted, the current directory is used.
#
# The output_dir argument is the directory where all output data will go
#
# WARNING: BOTH IMAGE_DIR and OUTPUT_DIR need to be FULL PATHS!! (or ./,
# which is also fine)

# Set this variable to your base install path (e.g., /home/foo/bundler)
# BASE_PATH="TODO"
BASE_PATH=$(dirname $(which $0));
LIB_PATH=$BASE_PATH/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$LIB_PATH

if [ $BASE_PATH == "TODO" ]
then
    echo "Please modify this script (RunBundler.sh) with the base path of your bundler installation.";
    exit;
fi

EXTRACT_FOCAL=$BASE_PATH/bin/extract_focal.pl

OS=`uname -o`

if [ $OS == "Cygwin" ]
then
    MATCHKEYS=$BASE_PATH/bin/KeyMatchFull.exe
    BUNDLER=$BASE_PATH/bin/Bundler.exe
else
    MATCHKEYS=$BASE_PATH/bin/KeyMatchFull
    BUNDLER=$BASE_PATH/bin/bundler
fi

TO_SIFT=$BASE_PATH/bin/ToSift.sh

IMAGE_DIR="."
OUTPUT_DIR="."

if [ $# -ge 1 ]
then
    echo "Using directory '$1'"
    IMAGE_DIR=$1
fi
if [ $# -eq 2 ]
then
    echo "Using output directory '$2'"
    OUTPUT_DIR=$2
fi

# Rename ".JPG" to ".jpg"
for d in `ls -1 $IMAGE_DIR | egrep ".JPG$"`
do 
    mv $IMAGE_DIR/$d $IMAGE_DIR/`echo $d | sed 's/\.JPG/\.jpg/'`
done

# Create the list of images
find $IMAGE_DIR -maxdepth 1 | egrep ".jpg$" | sort > $OUTPUT_DIR/list_tmp.txt
$EXTRACT_FOCAL $OUTPUT_DIR/list_tmp.txt $OUTPUT_DIR

# Run the ToSift script to generate a list of SIFT commands
echo "[- Extracting keypoints -]"
rm -f $OUTPUT_DIR/sift.txt
$TO_SIFT $IMAGE_DIR > $OUTPUT_DIR/sift.txt 

# Execute the SIFT commands
sh $OUTPUT_DIR/sift.txt

# Match images (can take a while)
echo "[- Matching keypoints (this can take a while) -]"
sed -e "s:\.jpg$:\.key:" $OUTPUT_DIR/list_tmp.txt > $OUTPUT_DIR/list_keys.txt
sleep 1
echo $MATCHKEYS list_keys.txt matches.init.txt 
$MATCHKEYS $OUTPUT_DIR/list_keys.txt $OUTPUT_DIR/matches.init.txt

# Generate the options file for running bundler
rm -f $OUTPUT_DIR/options.txt

echo "--match_table $OUTPUT_DIR/matches.init.txt" >> $OUTPUT_DIR/options.txt
echo "--output bundle.out" >> $OUTPUT_DIR/options.txt
echo "--output_all bundle_" >> $OUTPUT_DIR/options.txt
echo "--output_dir $OUTPUT_DIR" >> $OUTPUT_DIR/options.txt
echo "--intrinsics $OUTPUT_DIR/cam_params.txt" >> $OUTPUT_DIR/options.txt
echo "--run_bundle" >> $OUTPUT_DIR/options.txt

# Run Bundler!
echo "[- Running Bundler -]"
rm -f constraints.txt
rm -f pairwise_scores.txt
$BUNDLER $OUTPUT_DIR/list.txt --options_file $OUTPUT_DIR/options.txt > $OUTPUT_DIR/out
mv constraints.txt $OUTPUT_DIR
mv pairwise_scores.txt $OUTPUT_DIR
mv matches*.txt $OUTPUT_DIR
mv nmatches*.txt $OUTPUT_DIR

echo "[- Done -]"
