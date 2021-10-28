/****************************************************************************
** Meta object code from reading C++ file 'PointCloudMapper.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/PointCloudMapper.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PointCloudMapper.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_PointCloudMapper_t {
    QByteArrayData data[215];
    char stringdata0[3948];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PointCloudMapper_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PointCloudMapper_t qt_meta_stringdata_PointCloudMapper = {
    {
QT_MOC_LITERAL(0, 0, 16), // "PointCloudMapper"
QT_MOC_LITERAL(1, 17, 13), // "SelectProject"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 11), // "SaveProject"
QT_MOC_LITERAL(4, 44, 13), // "SaveProjectAs"
QT_MOC_LITERAL(5, 58, 11), // "SaveMapData"
QT_MOC_LITERAL(6, 70, 13), // "SaveMapDataAs"
QT_MOC_LITERAL(7, 84, 13), // "SaveModelData"
QT_MOC_LITERAL(8, 98, 15), // "SaveModelDataAs"
QT_MOC_LITERAL(9, 114, 15), // "SaveLaserDataAs"
QT_MOC_LITERAL(10, 130, 23), // "SaveSelectedLaserDataAs"
QT_MOC_LITERAL(11, 154, 12), // "SaveSettings"
QT_MOC_LITERAL(12, 167, 14), // "SaveSettingsAs"
QT_MOC_LITERAL(13, 182, 12), // "LoadSettings"
QT_MOC_LITERAL(14, 195, 15), // "SaveRoofRecords"
QT_MOC_LITERAL(15, 211, 17), // "SaveRoofRecordsAs"
QT_MOC_LITERAL(16, 229, 26), // "SelectRoofVerificationFile"
QT_MOC_LITERAL(17, 256, 13), // "ImportMapData"
QT_MOC_LITERAL(18, 270, 16), // "ImportPCMMapData"
QT_MOC_LITERAL(19, 287, 18), // "ImportPCMModelData"
QT_MOC_LITERAL(20, 306, 18), // "ImportLaserPyramid"
QT_MOC_LITERAL(21, 325, 16), // "ImportLaserBlock"
QT_MOC_LITERAL(22, 342, 20), // "ImportNewLaserPoints"
QT_MOC_LITERAL(23, 363, 27), // "ImportAdditionalLaserPoints"
QT_MOC_LITERAL(24, 391, 30), // "RemoveLoadedLaserPointsFromSet"
QT_MOC_LITERAL(25, 422, 7), // "QuitPCM"
QT_MOC_LITERAL(26, 430, 8), // "aboutPCM"
QT_MOC_LITERAL(27, 439, 14), // "PrintShortCuts"
QT_MOC_LITERAL(28, 454, 7), // "aboutQt"
QT_MOC_LITERAL(29, 462, 7), // "SetMode"
QT_MOC_LITERAL(30, 470, 9), // "MouseMode"
QT_MOC_LITERAL(31, 480, 8), // "new_mode"
QT_MOC_LITERAL(32, 489, 17), // "SetPoseChangeMode"
QT_MOC_LITERAL(33, 507, 28), // "SetPoseChangePerspectiveMode"
QT_MOC_LITERAL(34, 536, 13), // "SetBrowseMode"
QT_MOC_LITERAL(35, 550, 16), // "SetSelectMapMode"
QT_MOC_LITERAL(36, 567, 25), // "SetSelectMapPartitionMode"
QT_MOC_LITERAL(37, 593, 18), // "SetSelectModelMode"
QT_MOC_LITERAL(38, 612, 22), // "SetSelectModelPartMode"
QT_MOC_LITERAL(39, 635, 22), // "SetSelectModelFaceMode"
QT_MOC_LITERAL(40, 658, 23), // "SetSelectLaserPointMode"
QT_MOC_LITERAL(41, 682, 25), // "SetSelectLaserSegmentMode"
QT_MOC_LITERAL(42, 708, 22), // "SetSelectRectangleMode"
QT_MOC_LITERAL(43, 731, 15), // "SetSplitMapMode"
QT_MOC_LITERAL(44, 747, 24), // "SetSplitMapPartitionMode"
QT_MOC_LITERAL(45, 772, 17), // "SetExtendLineMode"
QT_MOC_LITERAL(46, 790, 15), // "SetMoveNodeMode"
QT_MOC_LITERAL(47, 806, 11), // "ShowMapData"
QT_MOC_LITERAL(48, 818, 20), // "ShowMapPartitionData"
QT_MOC_LITERAL(49, 839, 13), // "ShowModelData"
QT_MOC_LITERAL(50, 853, 17), // "ShowLastModelData"
QT_MOC_LITERAL(51, 871, 13), // "ShowLaserData"
QT_MOC_LITERAL(52, 885, 19), // "ShowSelectedMapData"
QT_MOC_LITERAL(53, 905, 28), // "ShowSelectedMapPartitionData"
QT_MOC_LITERAL(54, 934, 21), // "ShowSelectedModelData"
QT_MOC_LITERAL(55, 956, 25), // "ShowSelectedModelPartData"
QT_MOC_LITERAL(56, 982, 25), // "ShowSelectedModelFaceData"
QT_MOC_LITERAL(57, 1008, 21), // "ShowSelectedLaserData"
QT_MOC_LITERAL(58, 1030, 23), // "ToggleShowSelectedPoint"
QT_MOC_LITERAL(59, 1054, 24), // "ToggleShowTileBoundaries"
QT_MOC_LITERAL(60, 1079, 27), // "DisplayLaserDataAfterImport"
QT_MOC_LITERAL(61, 1107, 11), // "DisplayData"
QT_MOC_LITERAL(62, 1119, 8), // "DataType"
QT_MOC_LITERAL(63, 1128, 4), // "type"
QT_MOC_LITERAL(64, 1133, 10), // "PCMWindow*"
QT_MOC_LITERAL(65, 1144, 6), // "window"
QT_MOC_LITERAL(66, 1151, 12), // "data_changed"
QT_MOC_LITERAL(67, 1164, 25), // "DisplayMessageAfterExport"
QT_MOC_LITERAL(68, 1190, 18), // "UpdateShowDataMenu"
QT_MOC_LITERAL(69, 1209, 16), // "SelectObjectData"
QT_MOC_LITERAL(70, 1226, 11), // "PointNumber"
QT_MOC_LITERAL(71, 1238, 6), // "number"
QT_MOC_LITERAL(72, 1245, 10), // "Position3D"
QT_MOC_LITERAL(73, 1256, 7), // "map_pos"
QT_MOC_LITERAL(74, 1264, 14), // "selection_type"
QT_MOC_LITERAL(75, 1279, 9), // "base_type"
QT_MOC_LITERAL(76, 1289, 19), // "ToggleSelectionData"
QT_MOC_LITERAL(77, 1309, 15), // "SelectLaserData"
QT_MOC_LITERAL(78, 1325, 25), // "ChangeLaserPointSelection"
QT_MOC_LITERAL(79, 1351, 12), // "LaserPoints*"
QT_MOC_LITERAL(80, 1364, 14), // "selected_point"
QT_MOC_LITERAL(81, 1379, 3), // "add"
QT_MOC_LITERAL(82, 1383, 10), // "LaserPoint"
QT_MOC_LITERAL(83, 1394, 23), // "DisplayPointInformation"
QT_MOC_LITERAL(84, 1418, 5), // "point"
QT_MOC_LITERAL(85, 1424, 23), // "DeleteSelectedLaserData"
QT_MOC_LITERAL(86, 1448, 23), // "CropToSelectedLaserData"
QT_MOC_LITERAL(87, 1472, 20), // "SelectLaserDataByBox"
QT_MOC_LITERAL(88, 1493, 10), // "from_block"
QT_MOC_LITERAL(89, 1504, 6), // "inside"
QT_MOC_LITERAL(90, 1511, 9), // "show_data"
QT_MOC_LITERAL(91, 1521, 20), // "SelectLaserDataByMap"
QT_MOC_LITERAL(92, 1542, 19), // "ProcessShortCutKeys"
QT_MOC_LITERAL(93, 1562, 10), // "QKeyEvent*"
QT_MOC_LITERAL(94, 1573, 5), // "event"
QT_MOC_LITERAL(95, 1579, 12), // "SplitOutline"
QT_MOC_LITERAL(96, 1592, 4), // "mode"
QT_MOC_LITERAL(97, 1597, 13), // "LineSegment2D"
QT_MOC_LITERAL(98, 1611, 10), // "split_line"
QT_MOC_LITERAL(99, 1622, 13), // "MergeMapLines"
QT_MOC_LITERAL(100, 1636, 8), // "ReadView"
QT_MOC_LITERAL(101, 1645, 8), // "SaveView"
QT_MOC_LITERAL(102, 1654, 21), // "FitViewToSelectedData"
QT_MOC_LITERAL(103, 1676, 26), // "FitViewToSelectedLaserData"
QT_MOC_LITERAL(104, 1703, 20), // "ToggleResetOnLoading"
QT_MOC_LITERAL(105, 1724, 15), // "ReconstructRoof"
QT_MOC_LITERAL(106, 1740, 11), // "FitFlatRoof"
QT_MOC_LITERAL(107, 1752, 11), // "FitShedRoof"
QT_MOC_LITERAL(108, 1764, 12), // "FitGableRoof"
QT_MOC_LITERAL(109, 1777, 19), // "FitRotatedGableRoof"
QT_MOC_LITERAL(110, 1797, 10), // "FitHipRoof"
QT_MOC_LITERAL(111, 1808, 17), // "FitRotatedHipRoof"
QT_MOC_LITERAL(112, 1826, 22), // "FitDoubleSlopedHipRoof"
QT_MOC_LITERAL(113, 1849, 29), // "FitRotatedDoubleSlopedHipRoof"
QT_MOC_LITERAL(114, 1879, 14), // "FitGambrelRoof"
QT_MOC_LITERAL(115, 1894, 21), // "FitRotatedGambrelRoof"
QT_MOC_LITERAL(116, 1916, 13), // "FitSphereRoof"
QT_MOC_LITERAL(117, 1930, 11), // "FitConeRoof"
QT_MOC_LITERAL(118, 1942, 15), // "FitCylinderRoof"
QT_MOC_LITERAL(119, 1958, 22), // "FitRotatedCylinderRoof"
QT_MOC_LITERAL(120, 1981, 21), // "FitRectangularMapLine"
QT_MOC_LITERAL(121, 2003, 19), // "FitPolygonalMapLine"
QT_MOC_LITERAL(122, 2023, 18), // "FitCircularMapLine"
QT_MOC_LITERAL(123, 2042, 35), // "SetLowestPointForWallReconstr..."
QT_MOC_LITERAL(124, 2078, 22), // "SaveReconstructedModel"
QT_MOC_LITERAL(125, 2101, 13), // "CopyAllPoints"
QT_MOC_LITERAL(126, 2115, 18), // "CopySelectedPoints"
QT_MOC_LITERAL(127, 2134, 19), // "CopySegmentedPoints"
QT_MOC_LITERAL(128, 2154, 21), // "CopyUnsegmentedPoints"
QT_MOC_LITERAL(129, 2176, 15), // "DeleteAllPoints"
QT_MOC_LITERAL(130, 2192, 20), // "DeleteSelectedPoints"
QT_MOC_LITERAL(131, 2213, 21), // "DeleteSegmentedPoints"
QT_MOC_LITERAL(132, 2235, 23), // "DeleteUnsegmentedPoints"
QT_MOC_LITERAL(133, 2259, 16), // "SegmentLaserData"
QT_MOC_LITERAL(134, 2276, 12), // "GrowSurfaces"
QT_MOC_LITERAL(135, 2289, 35), // "UnifySegmentNumberSelectedLas..."
QT_MOC_LITERAL(136, 2325, 19), // "RemoveSmallSegments"
QT_MOC_LITERAL(137, 2345, 20), // "UnlabelSmallSegments"
QT_MOC_LITERAL(138, 2366, 21), // "MeanShiftSegmentation"
QT_MOC_LITERAL(139, 2388, 14), // "SegmentGrowing"
QT_MOC_LITERAL(140, 2403, 26), // "MajorityFilterSegmentation"
QT_MOC_LITERAL(141, 2430, 13), // "MergeSurfaces"
QT_MOC_LITERAL(142, 2444, 26), // "RemoveNonLastPulseSegments"
QT_MOC_LITERAL(143, 2471, 26), // "RemoveMultipleEchoSegments"
QT_MOC_LITERAL(144, 2498, 10), // "DeleteLine"
QT_MOC_LITERAL(145, 2509, 9), // "CropLines"
QT_MOC_LITERAL(146, 2519, 14), // "DeleteLastEdge"
QT_MOC_LITERAL(147, 2534, 11), // "ReverseLine"
QT_MOC_LITERAL(148, 2546, 13), // "CreateNewLine"
QT_MOC_LITERAL(149, 2560, 9), // "CloseLine"
QT_MOC_LITERAL(150, 2570, 15), // "SetNewStartNode"
QT_MOC_LITERAL(151, 2586, 23), // "SplitLineAtSelectedNode"
QT_MOC_LITERAL(152, 2610, 17), // "PartitionBuilding"
QT_MOC_LITERAL(153, 2628, 17), // "DeleteLoosePoints"
QT_MOC_LITERAL(154, 2646, 11), // "ObjectPoint"
QT_MOC_LITERAL(155, 2658, 22), // "ChangeBackGroundColour"
QT_MOC_LITERAL(156, 2681, 18), // "SetBackGroundImage"
QT_MOC_LITERAL(157, 2700, 14), // "BackGroundType"
QT_MOC_LITERAL(158, 2715, 13), // "selected_type"
QT_MOC_LITERAL(159, 2729, 20), // "SetNoBackGroundImage"
QT_MOC_LITERAL(160, 2750, 24), // "SetHeightBackGroundImage"
QT_MOC_LITERAL(161, 2775, 24), // "SetShadedBackGroundImage"
QT_MOC_LITERAL(162, 2800, 23), // "SetPhotoBackGroundImage"
QT_MOC_LITERAL(163, 2824, 19), // "LoadBackGroundImage"
QT_MOC_LITERAL(164, 2844, 15), // "RemovePCMWindow"
QT_MOC_LITERAL(165, 2860, 21), // "SpawnCopiedDataWindow"
QT_MOC_LITERAL(166, 2882, 19), // "SpawnSameDataWindow"
QT_MOC_LITERAL(167, 2902, 25), // "TransferDataFromSubWindow"
QT_MOC_LITERAL(168, 2928, 20), // "AddDataFromSubWindow"
QT_MOC_LITERAL(169, 2949, 13), // "SetNoScaleBar"
QT_MOC_LITERAL(170, 2963, 21), // "SetLeftBottomScaleBar"
QT_MOC_LITERAL(171, 2985, 17), // "SetCentreScaleBar"
QT_MOC_LITERAL(172, 3003, 15), // "SetScaleTick1mm"
QT_MOC_LITERAL(173, 3019, 15), // "SetScaleTick1cm"
QT_MOC_LITERAL(174, 3035, 16), // "SetScaleTick10cm"
QT_MOC_LITERAL(175, 3052, 14), // "SetScaleTick1m"
QT_MOC_LITERAL(176, 3067, 15), // "SetScaleTick10m"
QT_MOC_LITERAL(177, 3083, 16), // "SetScaleTick100m"
QT_MOC_LITERAL(178, 3100, 15), // "SetScaleTick1km"
QT_MOC_LITERAL(179, 3116, 25), // "SetLaserSelectionTagLabel"
QT_MOC_LITERAL(180, 3142, 33), // "SetLaserSelectionTagSegmentNu..."
QT_MOC_LITERAL(181, 3176, 31), // "SetLaserSelectionTagPlaneNumber"
QT_MOC_LITERAL(182, 3208, 30), // "SetLaserSelectionTagScanNumber"
QT_MOC_LITERAL(183, 3239, 40), // "SetLaserSelectionTagScanNumbe..."
QT_MOC_LITERAL(184, 3280, 27), // "SetLaserSelectionTagAFNCode"
QT_MOC_LITERAL(185, 3308, 30), // "SetLaserSelectionTagIsFiltered"
QT_MOC_LITERAL(186, 3339, 30), // "SetLaserSelectionTagPulseCount"
QT_MOC_LITERAL(187, 3370, 35), // "SetLaserSelectionTagComponent..."
QT_MOC_LITERAL(188, 3406, 40), // "SetLaserSelectionTagSegmentAn..."
QT_MOC_LITERAL(189, 3447, 34), // "SetLaserSelectionTagScanLineN..."
QT_MOC_LITERAL(190, 3482, 20), // "ModifyAttributeLabel"
QT_MOC_LITERAL(191, 3503, 25), // "ModifyAttributeIsFiltered"
QT_MOC_LITERAL(192, 3529, 26), // "ModifyAttributePlaneNumber"
QT_MOC_LITERAL(193, 3556, 28), // "ModifyAttributeSegmentNumber"
QT_MOC_LITERAL(194, 3585, 25), // "ModifyAttributeScanNumber"
QT_MOC_LITERAL(195, 3611, 22), // "UpdatePyramidSelection"
QT_MOC_LITERAL(196, 3634, 12), // "DataBounds2D"
QT_MOC_LITERAL(197, 3647, 6), // "bounds"
QT_MOC_LITERAL(198, 3654, 13), // "WriteDefaults"
QT_MOC_LITERAL(199, 3668, 11), // "const char*"
QT_MOC_LITERAL(200, 3680, 8), // "filename"
QT_MOC_LITERAL(201, 3689, 12), // "ReadDefaults"
QT_MOC_LITERAL(202, 3702, 21), // "ReconstructRoofCorner"
QT_MOC_LITERAL(203, 3724, 24), // "Reconstruct3DLineSegment"
QT_MOC_LITERAL(204, 3749, 16), // "ReconstructWalls"
QT_MOC_LITERAL(205, 3766, 24), // "LoadUpdatedPyramidPoints"
QT_MOC_LITERAL(206, 3791, 7), // "num_pts"
QT_MOC_LITERAL(207, 3799, 17), // "EditBuildingModel"
QT_MOC_LITERAL(208, 3817, 33), // "UpdateLastModelDataInResultWi..."
QT_MOC_LITERAL(209, 3851, 22), // "UpdateLaserDataDisplay"
QT_MOC_LITERAL(210, 3874, 15), // "FilterLaserData"
QT_MOC_LITERAL(211, 3890, 12), // "RoofAccepted"
QT_MOC_LITERAL(212, 3903, 13), // "RoofUndecided"
QT_MOC_LITERAL(213, 3917, 12), // "RoofRejected"
QT_MOC_LITERAL(214, 3930, 17) // "NextUndecidedRoof"

    },
    "PointCloudMapper\0SelectProject\0\0"
    "SaveProject\0SaveProjectAs\0SaveMapData\0"
    "SaveMapDataAs\0SaveModelData\0SaveModelDataAs\0"
    "SaveLaserDataAs\0SaveSelectedLaserDataAs\0"
    "SaveSettings\0SaveSettingsAs\0LoadSettings\0"
    "SaveRoofRecords\0SaveRoofRecordsAs\0"
    "SelectRoofVerificationFile\0ImportMapData\0"
    "ImportPCMMapData\0ImportPCMModelData\0"
    "ImportLaserPyramid\0ImportLaserBlock\0"
    "ImportNewLaserPoints\0ImportAdditionalLaserPoints\0"
    "RemoveLoadedLaserPointsFromSet\0QuitPCM\0"
    "aboutPCM\0PrintShortCuts\0aboutQt\0SetMode\0"
    "MouseMode\0new_mode\0SetPoseChangeMode\0"
    "SetPoseChangePerspectiveMode\0SetBrowseMode\0"
    "SetSelectMapMode\0SetSelectMapPartitionMode\0"
    "SetSelectModelMode\0SetSelectModelPartMode\0"
    "SetSelectModelFaceMode\0SetSelectLaserPointMode\0"
    "SetSelectLaserSegmentMode\0"
    "SetSelectRectangleMode\0SetSplitMapMode\0"
    "SetSplitMapPartitionMode\0SetExtendLineMode\0"
    "SetMoveNodeMode\0ShowMapData\0"
    "ShowMapPartitionData\0ShowModelData\0"
    "ShowLastModelData\0ShowLaserData\0"
    "ShowSelectedMapData\0ShowSelectedMapPartitionData\0"
    "ShowSelectedModelData\0ShowSelectedModelPartData\0"
    "ShowSelectedModelFaceData\0"
    "ShowSelectedLaserData\0ToggleShowSelectedPoint\0"
    "ToggleShowTileBoundaries\0"
    "DisplayLaserDataAfterImport\0DisplayData\0"
    "DataType\0type\0PCMWindow*\0window\0"
    "data_changed\0DisplayMessageAfterExport\0"
    "UpdateShowDataMenu\0SelectObjectData\0"
    "PointNumber\0number\0Position3D\0map_pos\0"
    "selection_type\0base_type\0ToggleSelectionData\0"
    "SelectLaserData\0ChangeLaserPointSelection\0"
    "LaserPoints*\0selected_point\0add\0"
    "LaserPoint\0DisplayPointInformation\0"
    "point\0DeleteSelectedLaserData\0"
    "CropToSelectedLaserData\0SelectLaserDataByBox\0"
    "from_block\0inside\0show_data\0"
    "SelectLaserDataByMap\0ProcessShortCutKeys\0"
    "QKeyEvent*\0event\0SplitOutline\0mode\0"
    "LineSegment2D\0split_line\0MergeMapLines\0"
    "ReadView\0SaveView\0FitViewToSelectedData\0"
    "FitViewToSelectedLaserData\0"
    "ToggleResetOnLoading\0ReconstructRoof\0"
    "FitFlatRoof\0FitShedRoof\0FitGableRoof\0"
    "FitRotatedGableRoof\0FitHipRoof\0"
    "FitRotatedHipRoof\0FitDoubleSlopedHipRoof\0"
    "FitRotatedDoubleSlopedHipRoof\0"
    "FitGambrelRoof\0FitRotatedGambrelRoof\0"
    "FitSphereRoof\0FitConeRoof\0FitCylinderRoof\0"
    "FitRotatedCylinderRoof\0FitRectangularMapLine\0"
    "FitPolygonalMapLine\0FitCircularMapLine\0"
    "SetLowestPointForWallReconstruction\0"
    "SaveReconstructedModel\0CopyAllPoints\0"
    "CopySelectedPoints\0CopySegmentedPoints\0"
    "CopyUnsegmentedPoints\0DeleteAllPoints\0"
    "DeleteSelectedPoints\0DeleteSegmentedPoints\0"
    "DeleteUnsegmentedPoints\0SegmentLaserData\0"
    "GrowSurfaces\0UnifySegmentNumberSelectedLaserData\0"
    "RemoveSmallSegments\0UnlabelSmallSegments\0"
    "MeanShiftSegmentation\0SegmentGrowing\0"
    "MajorityFilterSegmentation\0MergeSurfaces\0"
    "RemoveNonLastPulseSegments\0"
    "RemoveMultipleEchoSegments\0DeleteLine\0"
    "CropLines\0DeleteLastEdge\0ReverseLine\0"
    "CreateNewLine\0CloseLine\0SetNewStartNode\0"
    "SplitLineAtSelectedNode\0PartitionBuilding\0"
    "DeleteLoosePoints\0ObjectPoint\0"
    "ChangeBackGroundColour\0SetBackGroundImage\0"
    "BackGroundType\0selected_type\0"
    "SetNoBackGroundImage\0SetHeightBackGroundImage\0"
    "SetShadedBackGroundImage\0"
    "SetPhotoBackGroundImage\0LoadBackGroundImage\0"
    "RemovePCMWindow\0SpawnCopiedDataWindow\0"
    "SpawnSameDataWindow\0TransferDataFromSubWindow\0"
    "AddDataFromSubWindow\0SetNoScaleBar\0"
    "SetLeftBottomScaleBar\0SetCentreScaleBar\0"
    "SetScaleTick1mm\0SetScaleTick1cm\0"
    "SetScaleTick10cm\0SetScaleTick1m\0"
    "SetScaleTick10m\0SetScaleTick100m\0"
    "SetScaleTick1km\0SetLaserSelectionTagLabel\0"
    "SetLaserSelectionTagSegmentNumber\0"
    "SetLaserSelectionTagPlaneNumber\0"
    "SetLaserSelectionTagScanNumber\0"
    "SetLaserSelectionTagScanNumberWithoutAFN\0"
    "SetLaserSelectionTagAFNCode\0"
    "SetLaserSelectionTagIsFiltered\0"
    "SetLaserSelectionTagPulseCount\0"
    "SetLaserSelectionTagComponentNumber\0"
    "SetLaserSelectionTagSegmentAndTileNumber\0"
    "SetLaserSelectionTagScanLineNumber\0"
    "ModifyAttributeLabel\0ModifyAttributeIsFiltered\0"
    "ModifyAttributePlaneNumber\0"
    "ModifyAttributeSegmentNumber\0"
    "ModifyAttributeScanNumber\0"
    "UpdatePyramidSelection\0DataBounds2D\0"
    "bounds\0WriteDefaults\0const char*\0"
    "filename\0ReadDefaults\0ReconstructRoofCorner\0"
    "Reconstruct3DLineSegment\0ReconstructWalls\0"
    "LoadUpdatedPyramidPoints\0num_pts\0"
    "EditBuildingModel\0UpdateLastModelDataInResultWindow\0"
    "UpdateLaserDataDisplay\0FilterLaserData\0"
    "RoofAccepted\0RoofUndecided\0RoofRejected\0"
    "NextUndecidedRoof"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PointCloudMapper[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
     183,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  929,    2, 0x08 /* Private */,
       3,    0,  930,    2, 0x08 /* Private */,
       4,    0,  931,    2, 0x08 /* Private */,
       5,    0,  932,    2, 0x08 /* Private */,
       6,    0,  933,    2, 0x08 /* Private */,
       7,    0,  934,    2, 0x08 /* Private */,
       8,    0,  935,    2, 0x08 /* Private */,
       9,    0,  936,    2, 0x08 /* Private */,
      10,    0,  937,    2, 0x08 /* Private */,
      11,    0,  938,    2, 0x08 /* Private */,
      12,    0,  939,    2, 0x08 /* Private */,
      13,    0,  940,    2, 0x08 /* Private */,
      14,    0,  941,    2, 0x08 /* Private */,
      15,    0,  942,    2, 0x08 /* Private */,
      16,    0,  943,    2, 0x08 /* Private */,
      17,    0,  944,    2, 0x08 /* Private */,
      18,    0,  945,    2, 0x08 /* Private */,
      19,    0,  946,    2, 0x08 /* Private */,
      20,    0,  947,    2, 0x08 /* Private */,
      21,    0,  948,    2, 0x08 /* Private */,
      22,    0,  949,    2, 0x08 /* Private */,
      23,    0,  950,    2, 0x08 /* Private */,
      24,    0,  951,    2, 0x08 /* Private */,
      25,    0,  952,    2, 0x08 /* Private */,
      26,    0,  953,    2, 0x08 /* Private */,
      27,    0,  954,    2, 0x08 /* Private */,
      28,    0,  955,    2, 0x08 /* Private */,
      29,    1,  956,    2, 0x08 /* Private */,
      32,    0,  959,    2, 0x08 /* Private */,
      33,    0,  960,    2, 0x08 /* Private */,
      34,    0,  961,    2, 0x08 /* Private */,
      35,    0,  962,    2, 0x08 /* Private */,
      36,    0,  963,    2, 0x08 /* Private */,
      37,    0,  964,    2, 0x08 /* Private */,
      38,    0,  965,    2, 0x08 /* Private */,
      39,    0,  966,    2, 0x08 /* Private */,
      40,    0,  967,    2, 0x08 /* Private */,
      41,    0,  968,    2, 0x08 /* Private */,
      42,    0,  969,    2, 0x08 /* Private */,
      43,    0,  970,    2, 0x08 /* Private */,
      44,    0,  971,    2, 0x08 /* Private */,
      45,    0,  972,    2, 0x08 /* Private */,
      46,    0,  973,    2, 0x08 /* Private */,
      47,    0,  974,    2, 0x08 /* Private */,
      48,    0,  975,    2, 0x08 /* Private */,
      49,    0,  976,    2, 0x08 /* Private */,
      50,    0,  977,    2, 0x08 /* Private */,
      51,    0,  978,    2, 0x08 /* Private */,
      52,    0,  979,    2, 0x08 /* Private */,
      53,    0,  980,    2, 0x08 /* Private */,
      54,    0,  981,    2, 0x08 /* Private */,
      55,    0,  982,    2, 0x08 /* Private */,
      56,    0,  983,    2, 0x08 /* Private */,
      57,    0,  984,    2, 0x08 /* Private */,
      58,    0,  985,    2, 0x08 /* Private */,
      59,    0,  986,    2, 0x08 /* Private */,
      60,    0,  987,    2, 0x08 /* Private */,
      61,    3,  988,    2, 0x08 /* Private */,
      61,    2,  995,    2, 0x28 /* Private | MethodCloned */,
      67,    0, 1000,    2, 0x08 /* Private */,
      68,    1, 1001,    2, 0x08 /* Private */,
      69,    4, 1004,    2, 0x08 /* Private */,
      76,    1, 1013,    2, 0x08 /* Private */,
      77,    0, 1016,    2, 0x08 /* Private */,
      78,    2, 1017,    2, 0x08 /* Private */,
      77,    2, 1022,    2, 0x08 /* Private */,
      83,    1, 1027,    2, 0x08 /* Private */,
      85,    0, 1030,    2, 0x08 /* Private */,
      86,    0, 1031,    2, 0x08 /* Private */,
      87,    3, 1032,    2, 0x08 /* Private */,
      91,    3, 1039,    2, 0x08 /* Private */,
      92,    1, 1046,    2, 0x08 /* Private */,
      95,    2, 1049,    2, 0x08 /* Private */,
      99,    0, 1054,    2, 0x08 /* Private */,
     100,    0, 1055,    2, 0x08 /* Private */,
     101,    0, 1056,    2, 0x08 /* Private */,
     102,    0, 1057,    2, 0x08 /* Private */,
     103,    0, 1058,    2, 0x08 /* Private */,
     104,    0, 1059,    2, 0x08 /* Private */,
     105,    0, 1060,    2, 0x08 /* Private */,
     106,    0, 1061,    2, 0x08 /* Private */,
     107,    0, 1062,    2, 0x08 /* Private */,
     108,    0, 1063,    2, 0x08 /* Private */,
     109,    0, 1064,    2, 0x08 /* Private */,
     110,    0, 1065,    2, 0x08 /* Private */,
     111,    0, 1066,    2, 0x08 /* Private */,
     112,    0, 1067,    2, 0x08 /* Private */,
     113,    0, 1068,    2, 0x08 /* Private */,
     114,    0, 1069,    2, 0x08 /* Private */,
     115,    0, 1070,    2, 0x08 /* Private */,
     116,    0, 1071,    2, 0x08 /* Private */,
     117,    0, 1072,    2, 0x08 /* Private */,
     118,    0, 1073,    2, 0x08 /* Private */,
     119,    0, 1074,    2, 0x08 /* Private */,
     120,    0, 1075,    2, 0x08 /* Private */,
     121,    0, 1076,    2, 0x08 /* Private */,
     122,    0, 1077,    2, 0x08 /* Private */,
     123,    0, 1078,    2, 0x08 /* Private */,
     124,    1, 1079,    2, 0x08 /* Private */,
     125,    0, 1082,    2, 0x08 /* Private */,
     126,    0, 1083,    2, 0x08 /* Private */,
     127,    0, 1084,    2, 0x08 /* Private */,
     128,    0, 1085,    2, 0x08 /* Private */,
     129,    0, 1086,    2, 0x08 /* Private */,
     130,    0, 1087,    2, 0x08 /* Private */,
     131,    0, 1088,    2, 0x08 /* Private */,
     132,    0, 1089,    2, 0x08 /* Private */,
     133,    0, 1090,    2, 0x08 /* Private */,
     134,    0, 1091,    2, 0x08 /* Private */,
     135,    0, 1092,    2, 0x08 /* Private */,
     136,    0, 1093,    2, 0x08 /* Private */,
     137,    0, 1094,    2, 0x08 /* Private */,
     138,    0, 1095,    2, 0x08 /* Private */,
     139,    0, 1096,    2, 0x08 /* Private */,
     140,    0, 1097,    2, 0x08 /* Private */,
     141,    0, 1098,    2, 0x08 /* Private */,
     142,    0, 1099,    2, 0x08 /* Private */,
     143,    0, 1100,    2, 0x08 /* Private */,
     144,    0, 1101,    2, 0x08 /* Private */,
     145,    0, 1102,    2, 0x08 /* Private */,
     146,    0, 1103,    2, 0x08 /* Private */,
     147,    0, 1104,    2, 0x08 /* Private */,
     148,    0, 1105,    2, 0x08 /* Private */,
     149,    0, 1106,    2, 0x08 /* Private */,
     150,    0, 1107,    2, 0x08 /* Private */,
     151,    0, 1108,    2, 0x08 /* Private */,
     152,    0, 1109,    2, 0x08 /* Private */,
     153,    0, 1110,    2, 0x08 /* Private */,
      83,    2, 1111,    2, 0x08 /* Private */,
     155,    0, 1116,    2, 0x08 /* Private */,
     156,    2, 1117,    2, 0x08 /* Private */,
     159,    0, 1122,    2, 0x08 /* Private */,
     160,    0, 1123,    2, 0x08 /* Private */,
     161,    0, 1124,    2, 0x08 /* Private */,
     162,    0, 1125,    2, 0x08 /* Private */,
     163,    0, 1126,    2, 0x08 /* Private */,
     164,    1, 1127,    2, 0x08 /* Private */,
     165,    0, 1130,    2, 0x08 /* Private */,
     166,    0, 1131,    2, 0x08 /* Private */,
     167,    1, 1132,    2, 0x08 /* Private */,
     168,    1, 1135,    2, 0x08 /* Private */,
     169,    0, 1138,    2, 0x08 /* Private */,
     170,    0, 1139,    2, 0x08 /* Private */,
     171,    0, 1140,    2, 0x08 /* Private */,
     172,    0, 1141,    2, 0x08 /* Private */,
     173,    0, 1142,    2, 0x08 /* Private */,
     174,    0, 1143,    2, 0x08 /* Private */,
     175,    0, 1144,    2, 0x08 /* Private */,
     176,    0, 1145,    2, 0x08 /* Private */,
     177,    0, 1146,    2, 0x08 /* Private */,
     178,    0, 1147,    2, 0x08 /* Private */,
     179,    0, 1148,    2, 0x08 /* Private */,
     180,    0, 1149,    2, 0x08 /* Private */,
     181,    0, 1150,    2, 0x08 /* Private */,
     182,    0, 1151,    2, 0x08 /* Private */,
     183,    0, 1152,    2, 0x08 /* Private */,
     184,    0, 1153,    2, 0x08 /* Private */,
     185,    0, 1154,    2, 0x08 /* Private */,
     186,    0, 1155,    2, 0x08 /* Private */,
     187,    0, 1156,    2, 0x08 /* Private */,
     188,    0, 1157,    2, 0x08 /* Private */,
     189,    0, 1158,    2, 0x08 /* Private */,
     190,    0, 1159,    2, 0x08 /* Private */,
     191,    0, 1160,    2, 0x08 /* Private */,
     192,    0, 1161,    2, 0x08 /* Private */,
     193,    0, 1162,    2, 0x08 /* Private */,
     194,    0, 1163,    2, 0x08 /* Private */,
     195,    1, 1164,    2, 0x08 /* Private */,
     198,    1, 1167,    2, 0x08 /* Private */,
     201,    1, 1170,    2, 0x08 /* Private */,
     202,    0, 1173,    2, 0x08 /* Private */,
     203,    0, 1174,    2, 0x08 /* Private */,
     204,    0, 1175,    2, 0x08 /* Private */,
     205,    1, 1176,    2, 0x08 /* Private */,
     207,    0, 1179,    2, 0x08 /* Private */,
     207,    1, 1180,    2, 0x08 /* Private */,
     208,    0, 1183,    2, 0x08 /* Private */,
     209,    0, 1184,    2, 0x08 /* Private */,
     210,    0, 1185,    2, 0x08 /* Private */,
     211,    0, 1186,    2, 0x08 /* Private */,
     212,    0, 1187,    2, 0x08 /* Private */,
     213,    0, 1188,    2, 0x08 /* Private */,
     214,    0, 1189,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 30,   31,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 62, 0x80000000 | 64, QMetaType::Bool,   63,   65,   66,
    QMetaType::Void, 0x80000000 | 62, 0x80000000 | 64,   63,   65,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 62,   63,
    QMetaType::Void, 0x80000000 | 70, 0x80000000 | 72, 0x80000000 | 62, 0x80000000 | 62,   71,   73,   74,   75,
    QMetaType::Void, 0x80000000 | 62,   74,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 79, QMetaType::Bool,   80,   81,
    QMetaType::Void, 0x80000000 | 82, 0x80000000 | 62,   80,   63,
    QMetaType::Void, 0x80000000 | 82,   84,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool, QMetaType::Bool, QMetaType::Bool,   88,   89,   90,
    QMetaType::Void, QMetaType::Bool, QMetaType::Bool, QMetaType::Bool,   88,   89,   90,
    QMetaType::Void, 0x80000000 | 93,   94,
    QMetaType::Void, 0x80000000 | 30, 0x80000000 | 97,   96,   98,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 64,   65,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 154, 0x80000000 | 62,   84,   63,
    QMetaType::Void,
    QMetaType::Bool, 0x80000000 | 157, 0x80000000 | 64,  158,   65,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 64,   65,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 64,    2,
    QMetaType::Void, 0x80000000 | 64,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 196,  197,
    QMetaType::Void, 0x80000000 | 199,  200,
    QMetaType::Void, 0x80000000 | 199,  200,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,  206,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 64,   65,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void PointCloudMapper::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointCloudMapper *_t = static_cast<PointCloudMapper *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SelectProject(); break;
        case 1: _t->SaveProject(); break;
        case 2: _t->SaveProjectAs(); break;
        case 3: _t->SaveMapData(); break;
        case 4: _t->SaveMapDataAs(); break;
        case 5: _t->SaveModelData(); break;
        case 6: _t->SaveModelDataAs(); break;
        case 7: _t->SaveLaserDataAs(); break;
        case 8: _t->SaveSelectedLaserDataAs(); break;
        case 9: _t->SaveSettings(); break;
        case 10: _t->SaveSettingsAs(); break;
        case 11: _t->LoadSettings(); break;
        case 12: _t->SaveRoofRecords(); break;
        case 13: _t->SaveRoofRecordsAs(); break;
        case 14: _t->SelectRoofVerificationFile(); break;
        case 15: _t->ImportMapData(); break;
        case 16: _t->ImportPCMMapData(); break;
        case 17: _t->ImportPCMModelData(); break;
        case 18: _t->ImportLaserPyramid(); break;
        case 19: _t->ImportLaserBlock(); break;
        case 20: _t->ImportNewLaserPoints(); break;
        case 21: _t->ImportAdditionalLaserPoints(); break;
        case 22: _t->RemoveLoadedLaserPointsFromSet(); break;
        case 23: _t->QuitPCM(); break;
        case 24: _t->aboutPCM(); break;
        case 25: _t->PrintShortCuts(); break;
        case 26: _t->aboutQt(); break;
        case 27: _t->SetMode((*reinterpret_cast< MouseMode(*)>(_a[1]))); break;
        case 28: _t->SetPoseChangeMode(); break;
        case 29: _t->SetPoseChangePerspectiveMode(); break;
        case 30: _t->SetBrowseMode(); break;
        case 31: _t->SetSelectMapMode(); break;
        case 32: _t->SetSelectMapPartitionMode(); break;
        case 33: _t->SetSelectModelMode(); break;
        case 34: _t->SetSelectModelPartMode(); break;
        case 35: _t->SetSelectModelFaceMode(); break;
        case 36: _t->SetSelectLaserPointMode(); break;
        case 37: _t->SetSelectLaserSegmentMode(); break;
        case 38: _t->SetSelectRectangleMode(); break;
        case 39: _t->SetSplitMapMode(); break;
        case 40: _t->SetSplitMapPartitionMode(); break;
        case 41: _t->SetExtendLineMode(); break;
        case 42: _t->SetMoveNodeMode(); break;
        case 43: _t->ShowMapData(); break;
        case 44: _t->ShowMapPartitionData(); break;
        case 45: _t->ShowModelData(); break;
        case 46: _t->ShowLastModelData(); break;
        case 47: _t->ShowLaserData(); break;
        case 48: _t->ShowSelectedMapData(); break;
        case 49: _t->ShowSelectedMapPartitionData(); break;
        case 50: _t->ShowSelectedModelData(); break;
        case 51: _t->ShowSelectedModelPartData(); break;
        case 52: _t->ShowSelectedModelFaceData(); break;
        case 53: _t->ShowSelectedLaserData(); break;
        case 54: _t->ToggleShowSelectedPoint(); break;
        case 55: _t->ToggleShowTileBoundaries(); break;
        case 56: _t->DisplayLaserDataAfterImport(); break;
        case 57: _t->DisplayData((*reinterpret_cast< DataType(*)>(_a[1])),(*reinterpret_cast< PCMWindow*(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 58: _t->DisplayData((*reinterpret_cast< DataType(*)>(_a[1])),(*reinterpret_cast< PCMWindow*(*)>(_a[2]))); break;
        case 59: _t->DisplayMessageAfterExport(); break;
        case 60: _t->UpdateShowDataMenu((*reinterpret_cast< DataType(*)>(_a[1]))); break;
        case 61: _t->SelectObjectData((*reinterpret_cast< const PointNumber(*)>(_a[1])),(*reinterpret_cast< const Position3D(*)>(_a[2])),(*reinterpret_cast< DataType(*)>(_a[3])),(*reinterpret_cast< DataType(*)>(_a[4]))); break;
        case 62: _t->ToggleSelectionData((*reinterpret_cast< DataType(*)>(_a[1]))); break;
        case 63: _t->SelectLaserData(); break;
        case 64: _t->ChangeLaserPointSelection((*reinterpret_cast< LaserPoints*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 65: _t->SelectLaserData((*reinterpret_cast< const LaserPoint(*)>(_a[1])),(*reinterpret_cast< DataType(*)>(_a[2]))); break;
        case 66: _t->DisplayPointInformation((*reinterpret_cast< const LaserPoint(*)>(_a[1]))); break;
        case 67: _t->DeleteSelectedLaserData(); break;
        case 68: _t->CropToSelectedLaserData(); break;
        case 69: _t->SelectLaserDataByBox((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 70: _t->SelectLaserDataByMap((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 71: _t->ProcessShortCutKeys((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 72: _t->SplitOutline((*reinterpret_cast< MouseMode(*)>(_a[1])),(*reinterpret_cast< const LineSegment2D(*)>(_a[2]))); break;
        case 73: _t->MergeMapLines(); break;
        case 74: _t->ReadView(); break;
        case 75: _t->SaveView(); break;
        case 76: _t->FitViewToSelectedData(); break;
        case 77: _t->FitViewToSelectedLaserData(); break;
        case 78: _t->ToggleResetOnLoading(); break;
        case 79: _t->ReconstructRoof(); break;
        case 80: _t->FitFlatRoof(); break;
        case 81: _t->FitShedRoof(); break;
        case 82: _t->FitGableRoof(); break;
        case 83: _t->FitRotatedGableRoof(); break;
        case 84: _t->FitHipRoof(); break;
        case 85: _t->FitRotatedHipRoof(); break;
        case 86: _t->FitDoubleSlopedHipRoof(); break;
        case 87: _t->FitRotatedDoubleSlopedHipRoof(); break;
        case 88: _t->FitGambrelRoof(); break;
        case 89: _t->FitRotatedGambrelRoof(); break;
        case 90: _t->FitSphereRoof(); break;
        case 91: _t->FitConeRoof(); break;
        case 92: _t->FitCylinderRoof(); break;
        case 93: _t->FitRotatedCylinderRoof(); break;
        case 94: _t->FitRectangularMapLine(); break;
        case 95: _t->FitPolygonalMapLine(); break;
        case 96: _t->FitCircularMapLine(); break;
        case 97: _t->SetLowestPointForWallReconstruction(); break;
        case 98: _t->SaveReconstructedModel((*reinterpret_cast< PCMWindow*(*)>(_a[1]))); break;
        case 99: _t->CopyAllPoints(); break;
        case 100: _t->CopySelectedPoints(); break;
        case 101: _t->CopySegmentedPoints(); break;
        case 102: _t->CopyUnsegmentedPoints(); break;
        case 103: _t->DeleteAllPoints(); break;
        case 104: _t->DeleteSelectedPoints(); break;
        case 105: _t->DeleteSegmentedPoints(); break;
        case 106: _t->DeleteUnsegmentedPoints(); break;
        case 107: _t->SegmentLaserData(); break;
        case 108: _t->GrowSurfaces(); break;
        case 109: _t->UnifySegmentNumberSelectedLaserData(); break;
        case 110: _t->RemoveSmallSegments(); break;
        case 111: _t->UnlabelSmallSegments(); break;
        case 112: _t->MeanShiftSegmentation(); break;
        case 113: _t->SegmentGrowing(); break;
        case 114: _t->MajorityFilterSegmentation(); break;
        case 115: _t->MergeSurfaces(); break;
        case 116: _t->RemoveNonLastPulseSegments(); break;
        case 117: _t->RemoveMultipleEchoSegments(); break;
        case 118: _t->DeleteLine(); break;
        case 119: _t->CropLines(); break;
        case 120: _t->DeleteLastEdge(); break;
        case 121: _t->ReverseLine(); break;
        case 122: _t->CreateNewLine(); break;
        case 123: _t->CloseLine(); break;
        case 124: _t->SetNewStartNode(); break;
        case 125: _t->SplitLineAtSelectedNode(); break;
        case 126: _t->PartitionBuilding(); break;
        case 127: _t->DeleteLoosePoints(); break;
        case 128: _t->DisplayPointInformation((*reinterpret_cast< const ObjectPoint(*)>(_a[1])),(*reinterpret_cast< DataType(*)>(_a[2]))); break;
        case 129: _t->ChangeBackGroundColour(); break;
        case 130: { bool _r = _t->SetBackGroundImage((*reinterpret_cast< BackGroundType(*)>(_a[1])),(*reinterpret_cast< PCMWindow*(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 131: _t->SetNoBackGroundImage(); break;
        case 132: _t->SetHeightBackGroundImage(); break;
        case 133: _t->SetShadedBackGroundImage(); break;
        case 134: _t->SetPhotoBackGroundImage(); break;
        case 135: _t->LoadBackGroundImage(); break;
        case 136: _t->RemovePCMWindow((*reinterpret_cast< PCMWindow*(*)>(_a[1]))); break;
        case 137: _t->SpawnCopiedDataWindow(); break;
        case 138: _t->SpawnSameDataWindow(); break;
        case 139: _t->TransferDataFromSubWindow((*reinterpret_cast< PCMWindow*(*)>(_a[1]))); break;
        case 140: _t->AddDataFromSubWindow((*reinterpret_cast< PCMWindow*(*)>(_a[1]))); break;
        case 141: _t->SetNoScaleBar(); break;
        case 142: _t->SetLeftBottomScaleBar(); break;
        case 143: _t->SetCentreScaleBar(); break;
        case 144: _t->SetScaleTick1mm(); break;
        case 145: _t->SetScaleTick1cm(); break;
        case 146: _t->SetScaleTick10cm(); break;
        case 147: _t->SetScaleTick1m(); break;
        case 148: _t->SetScaleTick10m(); break;
        case 149: _t->SetScaleTick100m(); break;
        case 150: _t->SetScaleTick1km(); break;
        case 151: _t->SetLaserSelectionTagLabel(); break;
        case 152: _t->SetLaserSelectionTagSegmentNumber(); break;
        case 153: _t->SetLaserSelectionTagPlaneNumber(); break;
        case 154: _t->SetLaserSelectionTagScanNumber(); break;
        case 155: _t->SetLaserSelectionTagScanNumberWithoutAFN(); break;
        case 156: _t->SetLaserSelectionTagAFNCode(); break;
        case 157: _t->SetLaserSelectionTagIsFiltered(); break;
        case 158: _t->SetLaserSelectionTagPulseCount(); break;
        case 159: _t->SetLaserSelectionTagComponentNumber(); break;
        case 160: _t->SetLaserSelectionTagSegmentAndTileNumber(); break;
        case 161: _t->SetLaserSelectionTagScanLineNumber(); break;
        case 162: _t->ModifyAttributeLabel(); break;
        case 163: _t->ModifyAttributeIsFiltered(); break;
        case 164: _t->ModifyAttributePlaneNumber(); break;
        case 165: _t->ModifyAttributeSegmentNumber(); break;
        case 166: _t->ModifyAttributeScanNumber(); break;
        case 167: _t->UpdatePyramidSelection((*reinterpret_cast< const DataBounds2D(*)>(_a[1]))); break;
        case 168: _t->WriteDefaults((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 169: _t->ReadDefaults((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 170: _t->ReconstructRoofCorner(); break;
        case 171: _t->Reconstruct3DLineSegment(); break;
        case 172: _t->ReconstructWalls(); break;
        case 173: _t->LoadUpdatedPyramidPoints((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 174: _t->EditBuildingModel(); break;
        case 175: _t->EditBuildingModel((*reinterpret_cast< PCMWindow*(*)>(_a[1]))); break;
        case 176: _t->UpdateLastModelDataInResultWindow(); break;
        case 177: _t->UpdateLaserDataDisplay(); break;
        case 178: _t->FilterLaserData(); break;
        case 179: _t->RoofAccepted(); break;
        case 180: _t->RoofUndecided(); break;
        case 181: _t->RoofRejected(); break;
        case 182: _t->NextUndecidedRoof(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 57:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 58:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 98:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 130:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 136:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 139:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 140:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        case 175:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PCMWindow* >(); break;
            }
            break;
        }
    }
}

const QMetaObject PointCloudMapper::staticMetaObject = {
    { &PCMWindow::staticMetaObject, qt_meta_stringdata_PointCloudMapper.data,
      qt_meta_data_PointCloudMapper,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *PointCloudMapper::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PointCloudMapper::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PointCloudMapper.stringdata0))
        return static_cast<void*>(this);
    return PCMWindow::qt_metacast(_clname);
}

int PointCloudMapper::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PCMWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 183)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 183;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 183)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 183;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
