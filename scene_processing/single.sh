#!/bin/bash
#cp ~/Nips2011/cleanKinectData/office/singles/allBags/*.bag ./
num=0
for file in `dir -d demo?/*.bag` ; do
num=`expr $num + 1`
#if [ $num -gt 6 ]
# then
#cp $file globalTransform.bag
cp $file temp.bag
#echo "$file $num" >> scene_mapping.txt 
rosbag play -d 10 temp.bag &
rosrun scene_processing live_segment_computeFeats

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

