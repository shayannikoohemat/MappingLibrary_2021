### Data preparation
Don't panic. Keep Calm and enjoy the algorithm. :)

#### Downsampling and making chunks of point clouds
To speed up the process dwonsampling to a factor of 1/5 or more helps a lot. However, you lose details
and point spacing increases from mm to several cm which is not good. As we are using a planar segmentation,
it is wise to do the first a small downsampling of 2 cm point spacing. Then run the surface growing. 
Then when we have the segments, run another downsampling but keeping the segments as it is.
Another way of speeding up the process is creating chunks which keep the geometry consistent.
For example, **separate floors, then in each level separate to smaller chunks by cutting alongside 
the big halls or corridor and then start the process.

### Run the indoor_reconstruction() function
Run the `indoor_reconstruction()` function. The main inputs are the input laser file (should be already segmented)
and the output directory. Other arguments are optional and set by default.
Check the output in the output directory. Start by checking `output.laser`.
 In below you see how to analysis the output.

### Steps to check or debug the results
Technically, `output.laser` is the total classified points including walls, floor, ceiling and unknown or clutter.
You can see individual files as well, for example `wall.laser` has wall candidates.
**List of labels and colors in the pcm GUI**
 *          no label                    = 0,  white, dark pink
            almost horizontal           = 1,  green
            almost vertical             = 2,  blue
            label wall-slanted-wall     = 3,  sharp yellow
            wall                        = 4, orange
            floor                       = 5, light yellow
            ceiling                     = 6  light blue
            >> Tip: press A in GUI to see labels, Press G to see segments, 
            Press L to open Appearance settings window.
 If the output was not correct, for example wall.laser or ceiling.laser 
are empty or noisy, follow below steps.

1. First look at the ceiling.laser segment in the output data, if ceiling is not detected in the process
it is highly possible that the walls can not be detected neither.
2. Check `relabeled_vert_horizon.laser` and `almost_horizontal_segments.laser` 
files to see if segments are classified correctly according to the normal direction and
 based on the **angle threshold**. Maybe you need to change the angle threshold.
3. To decide for a better angle threshold you can check: `segments_infofile.txt`. Are angles for
large segments (e.g, ceiling) correct, you can decide for this just by looking at the segment inclination?
4. Print the segments angles to decide for a better `flat_angle` and `vertical_angle` threshold. By default,
it is 10 degrees for horizontal and vertical, and 45 degrees for slanted walls and ramps and ceilings.

> Tip1: you can set `bool sort_segments_by_number=true` to true to have segments sorted by number.

> Tip2: get familiar with the labeling convention of the classes and their color in our pcm GUI.

#### Estimate the height threshold of the floor and ceiling

It can happen that the program doesn't estimate the correct floor and ceiling height. 
The height estimation is necessary to decide where to exclude the clutter from being classified as the ceiling.
Remember if a segment is misclassified as a ceiling, it is possible that attached segments are also miscalssified
as walls. To avoid false positive cases (misclassified as wall ...) check the ceiling labels.

1. `almost_horizontal_segments_infofile.txt` is to analysis almost_horizontal segments
 to estimate floor and ceiling height and pass it as `ceiling_z_userdefined` and 
 `floor_z_userdefined` arguments.
 
 ### Pair-segments operations
 Pair segments operations are very critical to create **1.wall patches** meaning merging smaller segments to 
 larger segments and **2.adjacency graph**. To create the adjacency graph, segments in a vicinity
 are checked and if connected they are added to the nodes and edges of the graph.
 
 #### Generate wallpatches
 If the flag `bool generate_wallpatches=true` is set true, then method `Buffers GenerateWallPatches()`
  from utils is activated. Note that you should set its parameters as well or you can leave it as default. 
 Wallpatches creation is not only applied on walls, 
 but all segments. We call it wallpatches as this is the final goal. 
 Generate wall patches merge segments in two steps: merge and include, first merging calculates the 
 plane after merging two segments then including adds segment points to the reference semgment but it
 doesn't recalculate the plane, because recalculating can create a skewd plane while the plane of reference
 segment should be sued.
 To set correct parameters you can have a look at `graphinfo2.txt` to compare
 the plane's angle and distances for pairs. The result is stored as `final_segmentation.laser` 
 if the flag is true. Other intermediate files are `buffer_segmentation.laser` and 
 `merged_segmentation.laser` which you can ignore. They are for low level debugging. Note that generate 
 wall patches can apply the `segment_refinement()` method from utils but the flag should be set 
 true inside the code in utils. 
**Note** verobse=true for this function will write some *.txt files in the execute directory.
> Remember after **generating wall patches** segments are renumbered, but the segment numbers in the 
> graphinfoX.txt file belong to the original segmentation to check the results before and after the process.
 
 #### Generate the adjacency graph
 For controlling the result of adj graph you can check statistics in `graphinfo.txt` file.

#### How to debug the code
If you can't find the problem in output files then you should check the code.
To do this make a small example with 4 to 5 segments, representing the wall, floor and ceiling
 and including the problem you are in doubt about and then run the code
 and check the output. When that's fixed, add several more segments and try again. In this way you can
 debug the code. 