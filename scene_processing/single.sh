#!/bin/bash
#cp ~/Nips2011/cleanKinectData/office/singles/allBags/*.bag ./
num=0
#for file in `dir -d demoFinalRaghu/*.bag` ; do
for file in `dir -d demo0/*.bag` ; do
#for file in `dir -d *data*.bag` ; do
num=`expr $num + 1`
#if [ $num -gt 6 ]
# then
echo "now processing $file ..."
#cp $file globalTransform.bag
#cp $file temp.bag
#echo "$file $num" >> scene_mapping.txt 
rosbag play -d 10 $file &
rosrun scene_processing live_segment_computeFeats > log.txt

mkdir $file.pred
mv *.pcd $file.pred/
mv pred.* $file.pred/
mv *data_scene* $file.pred/
mv *topHeat*.png $file.pred/
mv *frontHeat*.png $file.pred/
mv *top0riginal*.png $file.pred/
mv *front0riginal*.png $file.pred/
mv log.txt $file.pred/

#fi

done

