### Description of generated files
#### How to use the outputs

### in/out data structure
```
DATA
├── dump                    # temp files
│   ├── 00_wNormals.xyz
│   └── lp_segmented.laser
|
├── faces_mesh
│   ├── Area_1_conferenceRoom_1_faces_invalid.top   # invalid faces
│   ├── Area_1_conferenceRoom_1_faces.objpts        # cell decomposition mesh (vertices)  
│   ├── Area_1_conferenceRoom_1_faces.top           # cell decomposition all faces
│   ├── Area_1_conferenceRoom_1_faces_valid.top     # valid faces
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
