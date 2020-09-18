#### How to use the test data

**Run the indoor_topology function (the main function for indoor reconstruction)**

*The assumption is: the input file is already downsampled and segmented using thinlaser and surfacegrowing.*
In a terminal run: `./indoor_reocnstruction -i /data/wall_fl_cl_furniture_seg12cm.laser -root_dir /data/out/`
Then check the result `wall.laser floor.laser and ceiling.laser`.
You see some furniture points are classified as floor. To fix that you can use `floor_buffer` parameter.
So run the code again `./indoor_reocnstruction -i /data/wall_fl_cl_furniture_seg12cm.laser
 -root_dir /data/out/ -flBuffer 0.20` then the floor result should be better. 0.20 cm means points with the 
 distance of 20 cm to the main floor can be considered as a floor. For example a step-floor.
 You can see the classification in label tag per point. YOu can use pcm GUI to see the result.
 
 #### Edit the results
 Use any point cloud editor (pcm GUI, CloudCompare, Paraview) to clean up the results. 
 It means remove or change the label of misclassified points. This may take 10 to 30 mins given your skills
 in the related GUI.
 
 ### Modeling
 Modeling section includes, creating a consistent vector models which is in geometry-modeling tool. 
 
 
 
 
 
 
