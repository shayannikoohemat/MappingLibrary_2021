
### script to run modeling tool
## add the parametrs here with a $ sign
## to make this executable run in terminal: $chmod +x modeling.sh
#$input_laser /home/shayan/MappingLibrary/Tools/indoor_geometry_reconstruction_v2/test_data/geometry_modeling/wall_fl_cl_labeled_segmented_labeled.laser
#$root_dir /home/shayan/MappingLibrary/Tools/indoor_geometry_reconstruction_v2/test_data/geometry_modeling/out/
#$planes_distances 0.40

./bin/modeling -i /home/shayan/MappingLibrary/Tools/indoor_geometry_reconstruction_v2/test_data/geometry_modeling/wall_fl_cl_labeled_segmented_labeled.laser  -root_dir /home/shayan/MappingLibrary/Tools/indoor_geometry_reconstruction_v2/test_data/geometry_modeling/out/ -MERGE_SURF 
