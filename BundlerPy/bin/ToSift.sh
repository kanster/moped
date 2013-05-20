#!/bin/bash
#
# ToSift.sh [IMAGE_DIR] [OUTPUT_DIR]
# Create a script for extracting sift features from a set of images
# Modified by Alvaro Collet to handle OUTPUT_DIR

# Set this variable to your base install path (e.g., /home/foo/bundler)
BIN_PATH=$(dirname $(which $0));

IMAGE_DIR="."
OUTPUT_DIR="."

if [ $# -ge 1 ]
then
    IMAGE_DIR=$1
fi
if [ $# -eq 2 ]
then
    OUTPUT_DIR=$2
else
    OUTPUT_DIR=$IMAGE_DIR
fi

OS=`uname -o`

if [ $OS == "Cygwin" ]
then
    SIFT=$BIN_PATH/siftWin32.exe
else
    SIFT=$BIN_PATH/sift
fi

if [ -e $SIFT ]
then 
:
else
    echo "[ToSift] Error: SIFT not found.  Please install SIFT to $BIN_PATH" > /dev/stderr
fi

for d in `ls -1 $IMAGE_DIR | egrep "jpg$"`
do 
    pgm_file=$IMAGE_DIR/`echo $d | sed 's/jpg$/pgm/'`
    key_file=$OUTPUT_DIR/`echo $d | sed 's/jpg$/key/'`
    gz_key_file=`echo $key_file | sed 's/key$/key\.gz/'`
    
    # Only do this if the key.gz file is not there
    if [ -f $gz_key_file ]
    then
        echo "echo \"File $gz_key_file exists, won't do it again\""
    else
        echo "mogrify -format pgm $IMAGE_DIR/$d; $SIFT < $pgm_file > $key_file; rm $pgm_file; gzip $key_file"
    fi
done
