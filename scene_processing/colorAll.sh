for file in `dir -d data_scene*.pcd` ; do
rosrun scene_processing colorImage $file label2color_office.txt
done

mkdir labeled
mv *.png labeled/


for file in `dir -d data_scene*.pcd` ; do
rosrun scene_processing colorImage $file label2color_office.txt NoLabels
done


mkdir unlabeled
mv *.png unlabeled/
