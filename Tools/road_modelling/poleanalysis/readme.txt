
# ***************************************************************************
# *   copyright            : (C) 2010 by University of Twente               *
# ***************************************************************************
#
# ***************************************************************************
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU General Public License as published by  *
# *   the Free Software Foundation; either version 2 of the License, or     *
# *   (at your option) any later version.                                   *
# *                                                                         *
# ***************************************************************************



(c) Martin Rutzinger, ITC
last update: 20.4.2010

This is the help file for the program "poles". It contains the usage of the program and a further detailed description of the paramters.

poles -usage
Usage: poles [-i <file name> OR -f <file filter>] : input files

Parameters for poles extraction
        [-per (def: 4)]                      : number of percentiles
        [-nperpole (def: 3)]                 : number of percentile for pole extraction (min. enclosing rectangle)
        [-parts (def: 0.1)]                  : height of parts to select pointsfor min. encl. rect. in nperpole
        [-maxpts (def: 5000)]                : maximum number of points in parts for min. encl. rect.
        [-numparts (def: 2)]                 : class: min number of rect. partspassing class criteria
        [-maxdiagpart (def: 0.3)]            : rect: maximum diagonal of rect.
        [-diffpos (def: 0.05)]               : rect: diff of position of rect. center between two adjacent parts
        [-diffdiag (def: 0.2)]               : rect: diff of diagonal between two adjacent rect. parts
        [-tag (def: 1)]                      : tag used for labeling: 1 -> LabelTag, 2 -> ComponentNumberTag
		

Detailed parameter description:

-i
The input is a segmented .laser point cloud. A unique id for each segment (or connected component) as SegmentNumberTag is required.

-per
Defines the number of percentiles, in which the segment splitted along the z-axis. If -per is set to 1 the whole segment is analysed.

-nperpole
Defines the percentile which is selected for analysing the pole. The percentiles are counted starting from the one with the lowest z-value. If -per is set 1 inorder to analyse the whole segment -nperpole has to be set to 1 as well.

-parts
The laser points within the selected percentile are sliced into parts along the x-axis. -parts gives the width of these slices.

-maxpts
A part is only analysed if the number of points is smaller than -maxpts. This avoids too long processing times of parts with alot of points (e.g. in tree crowns), which most likely do not belong to poles anyway.

-maxdiagpart
For each part a enclosing rectangle is fitted to the selected points. For this rectangle the diagonals and the centre point is calculated. -maxdiagpart defines the maximum allowed length of both diagonales in order to do further analysis. If the diagonale is too large the part is skipped.

-diffdiag
Is the maximum difference between the length of digagonales in a first part and a subsequent part. If the difference between diagonales of 2 subsequent parts is too large the parts are not considered for classification.

-diffpos
Is the maximum difference between the distance of two centre points in a first part and a subsequent part. If the difference between centre points of 2 subsequent parts is too large the parts are not considered for classification.

- numparts
This is the classification criteria. It defines how often the criteria of maxpts,maxdiagpart,-diffdiag,-diffpos must be fullfilled in order to label a segment as pole.

-tag
Defines with label tag is used for labeling poles in the laser file. -tag 1 uses the LabelTag and -tag 2 tag ComponentNumberTag