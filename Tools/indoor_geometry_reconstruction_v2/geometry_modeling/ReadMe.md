### Modeling walls, floors and ceiling
In this section, we create vector models from segments. We use wall, floor and ceiling segments from 
**indoor_topology.cpp** for this.

! Don't use the CMakeLists.txt and `main_modeling.cpp` in the `/geometrymodeling` root. 
! The `main_modeling.cpp` is just for knowing how to use the geometry modeling altogether. 
Instead use the `cmakes` and `mains` in the three subfolders `./extend2slabs`,
 `./modelingvolumetricwalls` and `./modelingwalls`
to build three separate exe files. This is because you need to understand steps individually,
 and know how to do troubleshooting.

#### A. Preparing Ground Truth
First we need to know how the real room layout looks like, to see how is the performance of our automatic 
method. Therefore, by using pcm (GUI) or CloudCompare clean the data, and to see inside the building
remove the ceiling. These steps are recommended and shouldn't take more than 10 mins per floor:
1. remove noisy points outside the main building layout. 
2. remove the floor and ceiling. This can be done either by `surface growing segmentation` in 
pcm (can take time, so subsampling is recommended) or in CloudCompare by croping the floor and ceiling.
3. run a connected component analysis (with a point distance of 10 cm). You can do this in 
`pcm> segmentation`. Before that set the parameter in `segmentation > edit parameters`.
4. Remove large pieces of furniture and keep the room layout. If furniture are connected to walls,
remove the connection and run conn comp again.

Now you can use this as a reference.  

#### B. How to perform geometry modeling

##### B1. First interactive refinement
For following steps you need segments with `segment number` and `labels` as wall, fl, cl.

In pcm or CloudCompare:
1. correct walls, floor and ceilings by renaming the false labels to unknown 
2. modify segments by removing wrong points

##### B2. Merge surfaces 
Use `Mergesurfaces()` from **`MergeSurface.cpp`**
   to generate `planes file` and `merged_segments.laser` where wall thickness is stored in
 `residual tags`. You can use the FLAG `force_verticality` set to TRUE to make sure slightly
 inclined walls are modeled perfectly vertical. In that case,
  the algorithm generates vertical planes but point clouds remain unmodified.
  Then we use `planes` output for all next steps.
  
  ` // code example//   Mergesurfaces (lp, planes_dist, planes_angle, segments_dist, min_segSize, root, planes_dist_second,
       true, false, true, verticality_angle_threshold, false);`
  
  You can check the results by looking at `projected_merged_segments.laser` file.

  > `Merge surfaces tool` renumbers the segments, so be careful if you combine two laser files
   from other files (e.g, floors or ceilings) make sure segment numbers are not repeated.
    There is a function for this to combine files and renumber segments in `utils/Mls_preprocessing.cpp`
    is called `merge_lp_segmented()` you can use this for merging laser files.

**outputs**
- `merged_surfaces.laser`
- `merged_planes.plane`
- `projected_merged_segments.laser` >> intermediate results
- `merge_info.txt`  >> metadata
  
##### B3. Extend walls to slabs (floor and ceiling)
    
     This step is not necessary if all walls are connected
     to the ceiling and floors.
Use `Extend_Walls_Ceiling()` from **`ModelingInteriorWalls.cpp`** to extend walls to slabs. This process
makes sure walls are connected to the floors and ceilings. Use merged_walls.laser and planes
from the previous step as input for this step. Do this process separately once for
 floors and use the output to extend it once for ceilings. After extending the walls to e.g, 
 floors rename the result file to e.g., `exteded_wall_Fl.laser` and use it as input to 
 extend the walls to the ceiling. Again rename the result file to `extended_wall_Fl_Cl.laser`
  
  This step also generates `rectanglesWalls_corners.objpts` and `rectanglesWalls_edges.top`
  and the same files for slabs. You can import this files in **pcm** by
   **`File > Import > Map`** and importing `objpts` and `top` respectively. However, this 
   is just for controlling the results.

   
  > Don't forget to rename outputs after each time applying 
   the algorithm, otherwise it overwrites the previous files.

  > **Good to know:** if input planes is passed empty then the function fit planes itself.
    So don't worry if you dont have planes, just pass it empty. This can happen if you skip
    merge surface step. For very clean files with distinguishable surfaces step 2 can be skipped. 

`// code example//
Extend_Walls_Ceiling (merged_walls, wall_planes, slabs, 100, root, true)`

**outputs**
- `extendedCeil_laserpoints.laser` if applied on floors DO NOT forget to rename it to floor.
- `extendedWall_laserpoints.laser` >> main result
- `rectanglesWalls_corners.objpts` >> intermediate result
- `rectanglesWalls_edges.top`      >> >> intermediate result
 
 ##### B3.1 Merge wall, floor and ceilings

 Before next step, combine all three files wall, floor and ceiling into one laser file. 
 To do this open extended_wall_Fl-Cl.laser and use **pcm** 
 `File > Import > Additional Laser Points` to add floor and ceiling. Then rename the file to
 e.g., `wall_fl_cl_renumbered.laser`. Renumber means you should make sure the segments of 
 floors and ceiling are not repeated in walls. To avoid this use just the largest segments
 from the floor and ceiling which has the most coverage, we call it `floor_master.laser`
  and `ceiling_master.laser`. If there is not such a file, and there are many smaller pieces
  with various heights, then you can use `merge_surfaces` also on floor and ceiling. 
  Alternatively, **to combine files and renumber segments** in `utils/Mls_preprocessing.cpp`
  there is a function for this is called `merge_lp_segmented()` you can use this for merging laser files.
 
##### B4. Modeling Interior Walls (extension of surfaces and generating rectangles)

 Main goal of this function is extension of segments/surfaces to each other. 
 In the previous step (B3), we extended merged_walls to the floor and ceiling separately,
  without any extension distance, it does it based on the closest objects. 
  Then in the this step we merge them in one file and we extend them
   using `ModelingInteriorWalls`, which extends them two by two using an extension threshold.
 > This step connects also slanted ceilings or ramps in floors, for example a ramp should be connected to 
    a horizontal floor. This step makes sure this happens.

 These are steps for modeling interior walls, floor and ceilings:
 1. Input: all the walls, floors and ceilings which are segmented.
   Labels are optional and not necessary for this function (wall=4, floor=5 and ceiling=6).
 2. We assume the input walls are generalized.
 3. This function first make sure all segments are intersecting in a specific threshold by extending the
 segment to the intersection line
 4. A minimum_enclosing_rectangle_3D will be generated for all segments (in 3D e.g. slanted walls, ramps)
 5. The plane for each segment should be an input from the merge surface step 
 otherwise a FitPlane method will be used in place. 
 
 **outputs** 
 - `extended_laserpoints.laser` >> main result
 - `rectangle_corners.objpts`   >> main result
 - `rectangle_edges.top`        >> main result
 
 The outputs would be written to the disk and later would be passed as input
  to `GenerateVolumetricWalls()` to generate boxes of each object.
 > **Note** planes can be passed empty, then the function calculates
  the planes per segment using a least square FitPlane() function.

##### B5. Generating Volumetric Files and OFF Format
Technically if everything is fine with previous step, you can directly apply steps B5.1 and B5.2 
directly on the output of the `extended_laserpoints()` which is the output of B4.
   > If you want to have separate OFF files at the end, after step4 separate walls, floors 
    and ceilings (using the labels) and proceed with step B5.
##### B5.1 Generate Volumetric Walls (3D boxes)
 Modeling volumetric walls generates boxes from rectangles/polygons.
The input of this function is the output of MergSurfaces() from utils
 tool OR from ModelingInteriorWalls() from previous step (B4).
Run `Generate_offset_dist_map()` and `GenerateVolumetricWalls()` respectively. The 2nd function 
needs the output of Generate_offset_dist_map(). Important note here is the role of
`Generate_offset_dist_map()`, it uses `wall thicknes` stored in `residual_tag` during 
`mergesurface()`. So leave the `lower_wall_thickness` and `upper_wall_thickness` values as default=0, to use 
calculated `wallthickness`. But if you want to normalize all the walls to a lower and upper
thickness to force all the walls look the same then set here the two parameters which represent the min and
max of wall thickness. 

Similarly, walls without two sides, hence with a zero `wallthickness` attribute, can have an offset as wall thickness,
then the argument `fix_offset_dist` for `GenerateVolumetricWalls()` multiplied by 2 is the thickness
  of the wall for such cases.
  
  **outptus**
  
   - `walls_vertices.objpts` >> vertices of walls 3D boxes
   - `walls_faces.top`       >> faces of walls 3D boxes

##### B5.2 Create OFF files for other 3D software (meshlab, cloudcompare)
Run `LineTopologies_to_OFFBoxes()` right after step B5.1. The inputs for LineTopologies_to_OFFBoxes
  are the outputs of `GenerateVolumetricWalls()`. 
 
 After this step the results can be imported into CloudCompare and exported as e.g., OBJ or PLy mesh.
   
   **outptu**
   
   - `off_output.off` >> OFF files of 3D boxes
 
#### C. How to visualise results in each step
A good practice to see the performance of the algorithm and check if the output is suitable for 
next steps is to visualize them. For example, draw minimum rectangles and see how the surfaces 
look like. In the `../visualization_tools` directory you can find tools to do this:

- `VisualizePlanes()` to draw a minimum oriented rectangle (any arbitrary orientation)
**Note**: there are two implementations for `void VisualizePlanes()`. The one which accepts 
input directory draws an oriented rectangle. Look at the following examples.
     
- `Min3DRectangle_to_3DBox()` draw a minimum oriented bounding box 

- `Min3DBox_OFF()` generate OFF files to import in 3D softwares (Meshalb, cloudcompare)

- `LineTopologies_to_OFF()` generate OFF file from linetopologies

**Method Examples**
    
    /// to generate slanted and vertical rectnagles
    VisualizePlanes (lp, min_seg_size, corners, lines, rootDir, false);
    
    /// to generate horizontal rectangles
    VisualizePlanes (lp, min_seg_size, max_edge_dist, corners, lines, false);
    
    /// to generate a vertical NOT Oriented BBox and convert it to OFF file from
    Min3DBox_OFF(corners, lines, lp, min_seg_size, offFile_out, min_z, max_z, height); 
    
    /// to generate an arbitrary oriented box
    Min3DRectangle_to_3DBox (corners, offset_dist, threeDbox_vertices, threeDbox_edges); /// with vertices and edges
    
    /// to generate an oriented box with faces instead of edges
    Min3DRectangle_to_3DFaces (corners, offset_dist, threeDbox_vertices, threeDbox_faces); /// with vertices and faces
    
    /// This function converts the linetopologies and vertices of objects to vertices and 
    faces that are readable by meshlab software and cloudcompare. 
    Note: For a polygon it may fails as meshes are not created.
    LineTopologies_to_OFF (const ObjectPoints &vertices, const LineTopologies &faces, char *root, bool verbose)

