#!/bin/bash

key=$1
resolution=$2

MY_DIR=`dirname ${BASH_SOURCE}`
. $MY_DIR/${key}_config.sh

cd Data/$1_player/ || exit 1

# generates laser_reflectance_all.bin
#../../bin/simulateddata $map_width $map_height #142.46 54.36
# generates groundtruth/ inputstrem/
cmd="../../bin/visualize_ground_truth $map_width $map_height $resolution ."
echo $cmd
$cmd
avconv -i groundtruth/%d.png -b:v 5000k gt.mp4
avconv -i inputstream/%d.png -b:v 5000k in.mp4
cp $(ls -t groundtruth/*.png  | head -1) gt-final.png
rm -r groundtruth/
rm -r inputstream/
