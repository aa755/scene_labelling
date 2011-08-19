#!/bin/bash
#cp ~/Nips2011/cleanKinectData/office/singles/allBags/*.bag ./
num=0
for file in `dir -d *.bag` ; do
num=`expr $num + 1`
cp $file globalTransform.bag
#echo "$file $num" >> scene_mapping.txt 
rosbag play -d 20 globalTransform.bag &
rosrun scene_processing live_segment_computeFeats 

mkdir $file.pred
mv *.pcd $file.pred/
mv pred.* $file.pred/
mv *data_scene* $file.pred/
mv topHeat.png $file.pred/
mv frontHeat.png $file.pred/
done

