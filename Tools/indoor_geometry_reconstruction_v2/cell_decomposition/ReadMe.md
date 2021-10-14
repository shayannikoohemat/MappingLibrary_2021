### Description of generated files
read the data structure.

### How to use the outputs
You can open *.top with an associated *.objpts to see the cell decomposition or faces.
You can use ITC PCM GUI to open the *.laser files or use laser2ascii tool from the ITC Library/Tools 
to convert them to ASCII formats.

**Labels and Tags:**
In `*.top` files which represent faces `LineNumberTag` (code 16 -- 100/101) shows the valid/invalid faces faces (wihtout points).
In `*.top` files which represent faces `LineLabelTagTag` (code 0 -- segment-num) shows the parent segment which the face belongs to.

In laserpoints `SemgmentNumberTag` is used for planar segmentation. 
`LabelTag` is used for sampled points labels valid=100/invalid=101. 
`LabelTag` is also used for object classes: 
**ceiling=0, floor=1, wall=2, beam=3, column=4, door=5, clutter=6**

In laserpoints `ScanLineNumberTag` is used for face numbers association. PlaneNumberTag is used for plane number, 
for example a segment can have the same plane number but can be decomposed to different faces (ScanLineNumberTag).

### Labeling Process
We label points and associated faces (if possible) for training data.
We autoamtically assign invalid label to faces which have less than a threshold supporting points(=500). Then we sample
points on these faces and give them label invalid. Rest of the (original) points get label valid. So if a face doesn't belong to
permanent structure and has points on it and these points are sparse or very little (from a noise or clutter) then they are invalid=101.
You can use `foo_original_pnts_plus_invalid_sampled_pnts_facenum.laser` to check if the labeling (valid=100/invalid=101) is correct.
In this file you see sampled points on invalid faces with label 101 and original points with label 100. You can select points
asscoiated with a face using the ScaLineNumberTag. And you can select points associated to a segment using the SegmentNumber.
If you wish to use sample points for all faces, replace the original points with `foo_valid_sampled_pnts_facenum.laser`.

#### Where automatic labeling goes wrong?
If a face is created for a permanent structure but doesn't have supporting points from the original scan, then it gets invalid label autoamtically,
this should be corrected during the manual checkup.
Similarly, if a face is created but doesn't belong to any structure and has some supporting points and those points belong to a clutter or noise then that face
 may still get a valid label (and its associated points) so the operator should change it to invalid.

### in/out data structure
```
DATA
├── dump                    # temp files
│   ├── 00_wNormals.xyz
│   └── lp_segmented.laser
|
├── faces_mesh
│   ├── Area_1_conferenceRoom_1_faces_invalid.top   # invalid faces. The LineNUmberTag (code 16 -- 101) shows the invalid faces (without points)
│   ├── Area_1_conferenceRoom_1_faces.objpts        # cell decomposition mesh (vertices)  
│   ├── Area_1_conferenceRoom_1_faces.top           # cell decomposition all faces. LineLabelTag (code 0 -- segment-num) shows the parent segment which the face belongs to.
│   ├── Area_1_conferenceRoom_1_faces_valid.top     # valid faces. LineNUmber Tag (code 16 -- 100) shows the valid faces (with points)
│   ├── ... next file
|
├── input_data
│   └── Area_1_conferenceRoom_2.txt                 # input txt ASCII file (point clouds of a room)
│   ├── ... next file
|
├── laserpoints
│   ├── ascii                                       # dir of ascii files output
│   └── laser                                       # dir laser files output
│       ├── Area_1_conferenceRoom_1_invalid_sampled_pnts_facenum.laser                          # the sampled points on invalid faces. Check ScanLineNumberTag for face number
│       ├── Area_1_conferenceRoom_1_original_pnts_plus_invalid_sampled_pnts_facenum.laser       # original segmented points plus the sampled points on invalid faces. Check ScanLineNumberTag for face number
│       ├── Area_1_conferenceRoom_1_original_points_facenum.laser                               # original segmented points with face numbers. Check ScanLineNumberTag for face number
│       ├── Area_1_conferenceRoom_1_seg.laser                                                   # original segmented points. Cehck SegmentNumber for segmentation
│       ├── Area_1_conferenceRoom_1_valid_sampled_pnts_facenum.laser                            # the sampled points on valid faces. Check ScanLineNumberTag for face number 
|
├── off
│   └── Area_1_conferenceRoom_1_decomposed.off      # cell decomposition mesh in OFF format
|   ├── ... next file
|
└── threed_boxes
    ├── Area_1_conferenceRoom_1_3dbox.objpts        # 3D Box of the room mesh (vertices)
    ├── Area_1_conferenceRoom_1_3dbox.off           # 3D Box of the room in OFF format
    ├── Area_1_conferenceRoom_1_3dbox.top           # 3D Box of the room faces
    ├── ... next file
```

### Cell decomposition steps
&nbsp;
<p float="left">
    <img src="image/celldecomposition_steps.png"/>
</p>

&nbsp;

![Figure1: the cell decomposition steps](https://github.com/shayannikoohemat/MappingLibrary_2021/blob/master/Tools/indoor_geometry_reconstruction_v2/cell_decomposition/images/celldecomposition_steps.png)
