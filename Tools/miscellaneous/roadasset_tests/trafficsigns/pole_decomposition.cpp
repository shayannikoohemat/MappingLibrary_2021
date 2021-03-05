//
// Created by shayan on 10/27/20.
//

#include <LaserPoints.h>
#include "../utils/MLS_preprocessing.h"
#include "pole_decomposition.h"
#include <numeric>

bool sameDOUBLE(double a, double b, double EPSILON)
{
    return fabs(a - b) < EPSILON;
}

void Visualize2DRectangles(vector< LaserPoints> &laserpoints_vec, ObjectPoints &rect_corners,
                           LineTopologies &rect_edges, LaserPointTag tag){
    int next_number;
    int line_number=0;
    for (auto &s : laserpoints_vec){ // s is a slice or a segment of laserpoints
        if (s.size() > 5){
            int seg_num = s[0].Attribute(tag);
            DataBoundsLaser db;
            db = s.DeriveDataBounds(0);
            /// Find the minimum enclosing rectangle for the segment in 2D
            s.DeriveTIN();
            ObjectPoints corners_per_rectangle;
            LineTopology edges_per_rectnagle;
            double max_edge_dist=0.10;
            s.EnclosingRectangle(max_edge_dist, corners_per_rectangle, edges_per_rectnagle);
            // Store points and topology in one file
            if (rect_corners.empty()){
                next_number = 0;
            } else {
                next_number = (rect_corners.back ()).Number () + 1;
            }
            /// update the numbering for objectpoints of corners
            PointNumber pnumber;
            LineTopology rectangle_edges_renumebr;
            for (int i=0; i < corners_per_rectangle.size() ; i++){
                pnumber = PointNumber(next_number + i);
                //corner = ObjectPoint(corners[i], pnumber, cov3d);
                corners_per_rectangle[i].NumberRef () = pnumber;
                /// set slice center z-value to the rectangle corners
                corners_per_rectangle[i].Z() = s.CentreOfGravity(s.TaggedPointNumberList(tag, seg_num)).GetZ();
                rect_corners.push_back(corners_per_rectangle[i]);
                rectangle_edges_renumebr.push_back(pnumber);  // making the linetopology
            }
            rectangle_edges_renumebr.push_back(PointNumber(next_number)); // Close the polygon
            if (rectangle_edges_renumebr.IsClockWise(rect_corners)){
                rectangle_edges_renumebr.MakeCounterClockWise(rect_corners);
            }else{
                rectangle_edges_renumebr.MakeClockWise(rect_corners);
            }

            rectangle_edges_renumebr.Number () = line_number++; //seg_num
            rect_edges.push_back(rectangle_edges_renumebr);
        }
    } // end of slices
}

Line3D RANSAC3DLine_customized(LaserPoints lp, double max_dist_to_line, int min_num_hits,
                               int max_num_tries, int &num_hits,
                               double max_dist_between_points,
                               LineSegments3D &segments)
{

    Line3D                      line;
    LaserPoints::const_iterator point, point1, point2;
    int                         num_pts, pt_no, random_number1,
            random_number2, num_tries;
    long                        random_long;
    bool                        found, changed;
    double                      scalar;
    LineSegments3D::iterator    segment1, segment2;

    num_hits = 0;

    // Count number of points with the specified attribute value
    num_pts = lp.size();
    if (num_pts < min_num_hits) return line; // Can never succeed

    srand(1); // Initialise random number generator

    // Generate hypotheses
    num_tries = 0;
    do {
        num_tries++;
        // Delete old line data
        line.Erase();
        // Get two random points
        random_long = rand();
        random_number1 = random_long - (random_long / num_pts) * num_pts;
        point1 = lp.begin() + random_number1;
        random_number2 = random_number1;
        while (random_number2 == random_number1) {
            random_long = rand();
            random_number2 = random_long - (random_long / num_pts) * num_pts;
        }
        point2 = lp.begin() + random_number2;
        // Construct the 3D line
        line = Line3D(point1->Position3DRef(), point2->Position3DRef());
        // Count number of points near the line and collect data for recalculation
        for (point=lp.begin(), num_hits=0; point!=lp.end(); point++) {
            if (line.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
                num_hits++;
                line.AddPoint(point->Position3DRef(), false);
            }
        }
        if (num_hits > min_num_hits) {
            // Fit line to all selected points
            line.Recalculate();

            // Make a segment from every point near the line, store number of points in segment number
            segments.erase(segments.begin(), segments.end());
            for (point=lp.begin(), num_hits=0; point!=lp.end(); point++) {
/*                if (select_tag != UndefinedTag) {
                    if (point->HasAttribute(select_tag)) {
                        if (point->Attribute(select_tag) != select_value) found = false;
                    }
                    else found = false;
                }*/

                if (line.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
                    num_hits++;
                    scalar = line.Scalar(point->Position3DRef());
                    segments.push_back(LineSegment3D(line, scalar, scalar));
                    (segments.end()-1)->Number() = 1;
                }
            }

            // Merge segments
            segments.SortOnStartScalar();
            do {
                changed = false;
                for (segment1=segments.begin(); segment1<segments.end()-1; segment1++) {
                    segment2 = segment1 + 1;
                    if (segment2->ScalarBegin() - segment1->ScalarEnd() <=
                        max_dist_between_points) {
                        changed = true;
                        segment1->ScalarEnd() = segment2->ScalarEnd();
                        segment1->Number() = segment1->Number() + segment2->Number();
                        segments.erase(segment2);
                        segment1--;
                    }
                }
            } while (changed);

            // Sort segments after the number of points
            segments.SortOnNumber();
            if (segments.empty()) num_hits = 0;
            else num_hits = segments.begin()->Number();
        }
    } while (num_tries < max_num_tries && num_hits < min_num_hits);

    if (num_hits < min_num_hits) segments.erase(segments.begin(), segments.end());
    return line;
}

int pole_decomposition_fittingLineRANSAC(LaserPoints &predicted_poles, char *root, double max_dist_toLine, bool verbose) {

    char str_root[500];
    vector<LaserPoints> poleComponent_vec;
    poleComponent_vec = PartitionLpByTag(predicted_poles, SegmentNumberTag, root);
    // fit line to each pole and extract the vertical lines
    LineSegments3D linesegments;
    LaserPoints inlierComponents; // not used
    ObjectPoints ransacLine_points;
    LineTopologies ransacLine_topo;
    LaserPoints inlierpoints, remaining_points;
    for (auto &poleComp : poleComponent_vec) {
        if(poleComp.size() < 5) continue;
        int segmentNum = poleComp[0].Attribute(SegmentNumberTag);
        printf("\n -------------------------------- \n");
        printf("Segment Number: %d \n", segmentNum);
        int numhits;
        /// (50.0 * poleComp.size() / 100) is half of the number of points in the segment
        int min_hits = int(40.0 * poleComp.size() / 100);
        if(verbose) printf("minimum num of hits: %d \n", min_hits);
        Line3D fittedline = poleComp.RANSAC3DLine(max_dist_toLine, min_hits,
                            500, numhits, 1.0, linesegments);
        if (numhits >= min_hits) {
            printf("\nnum of hits: %d \n", numhits);
            printf("num of laserpoints: %d", poleComp.size());
            for (auto &point : poleComp) {
                if (fittedline.DistanceToPoint(point.Position3DRef()) <= max_dist_toLine) {
                    inlierpoints.push_back(point);
                } else remaining_points.push_back(point);
            }
            for (auto &lineSeg:linesegments) {
                //TODO: later we can skip small line segments
                printf("\n line length: %.2f \n", lineSeg.Length() );
                //Angle
                //Angle2Lines(lineSeg, Vector3D(0,0,1));
            }
            ObjectPoints fittedline_objpoints;
            LineTopologies fittedline_top;
            linesegments.PointsWithTopology(fittedline_objpoints, fittedline_top, 1);
            fittedline_top.SetAttribute(LineLabelTag, segmentNum);
//           printf("num of lines: %d \n", fittedline_top.size());
            if (!ransacLine_points.empty())
                fittedline_top.ReNumber(fittedline_objpoints, (ransacLine_points.end() - 1)->Number() + 1,
                                        (ransacLine_topo.end() - 1)->Number() + 1);
            ransacLine_topo.insert(ransacLine_topo.end(), fittedline_top.begin(), fittedline_top.end());
            ransacLine_points.insert(ransacLine_points.end(), fittedline_objpoints.begin(), fittedline_objpoints.end());
        } else {
            printf("\nNOTENOUGH!!! num of hits: %d \n", numhits);
            printf("NOTENOUGH!!! num of laserpoints: %d \n", poleComp.size());
        }
    }

    printf("Averaging double points ...\n");

    ransacLine_points.AverageDoublePoints(ransacLine_topo, 0.25, 0.1);
    printf("Removing double points from %d to ", ransacLine_points.size());
    ransacLine_points.RemoveDoublePoints(ransacLine_topo, 0.15);
    printf("%d.\n", ransacLine_points.size());
    strcpy(str_root, root);
    ransacLine_topo.Write(strcat(str_root, "/ransacLine_topo.top"), false);
    strcpy(str_root, root);
    ransacLine_points.Write(strcat(str_root, "/ransacLine_points.objpts"));
    strcpy(str_root, root);
    inlierpoints.Write(strcat(str_root, "/inlierpoints.laser"), false);
    strcpy(str_root, root);
    remaining_points.Write(strcat(str_root, "/remaining_points.laser"), false);
}

LineSegment3D extend_Line3D(DataBoundsLaser &db, const LineSegment3D& linesegment){

    Position3D max_pos, min_pos, begin_pos, end_pos;
    /// we get the begin and end point of the linesegment and extend it to the Z-value of the laserpoints bounds
    begin_pos = linesegment.BeginPoint();
    end_pos = linesegment.EndPoint();
    /// modify the Z-value of the begin and end
    begin_pos.Z() = db.Minimum().GetZ();
    end_pos.Z()   = db.Maximum().GetZ();

    /// now we create a new line segment
    LineSegment3D extended_line;
    extended_line = LineSegment3D(begin_pos, end_pos);

    return extended_line;
}

double calculate_laserpoints_2Ddiameter(LaserPoints &lpoints){
    DataBoundsLaser db;
    db = lpoints.DeriveDataBounds(0);
    /// Find the minimum enclosing rectangle for the segment in 2D
    lpoints.DeriveTIN();
    ObjectPoints rect_vertices;
    LineTopology rect_edges;
    double max_edge_dist = 0.10;
    lpoints.EnclosingRectangle(max_edge_dist, rect_vertices, rect_edges);
    /// we assume points of the min rect are ordered as: 0 1 2 3 0 OR 0 3 2 1 0  then 0 and 2 should be the opposite of each other.
    double diameter =fabs(rect_vertices[0].Distance2D(rect_vertices[2])) ;
    return diameter;
}

/// we are not worry about the overflow here because the vector size is small
std::pair<double, double> get_mean_and_stdev(vector<double> v ){
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);

    std::pair<double, double> mean_stdev;
    mean_stdev.first  = mean;
    mean_stdev.second = stdev;

    return mean_stdev;
}


void pole_decomposition_slicing(LaserPoints &lpoints, char *root, double slice_height, double width_diff_tolerance,
                                LaserPointTag tag, bool RANSAC_linefitting, double max_dist_toLine, bool verbose) {

    char str_root[500];
    vector<LaserPoints> poleComponent_vec, accepted_slices_lpVec;
    poleComponent_vec = PartitionLpByTag(lpoints, tag, root);
    LaserPoints lpoints_sliced,  // slice number is reserved in ComponentTag
    slices_centers,             // slices center of gravity --> just for evaluation, not necessary for final results
    accepted_slicePoints,       // accepted slices based on the min-rectangle regularization --> necessary output
    inlierpoints,               // inlier points to the line which is fitted to the extracted carrier points
    remaining_points;   // should show the pole attachments. remaining points after removing carriers. This is used for decomposition.
    LineTopologies poles_sekeleton_lines, // connected center points of slices
    rects_edges,                // edges of min rectangle of slices
    fittedPoleLines_topo;       // this is fitted line to the pole carrier
    ObjectPoints poles_sekeleton_points,
    rects_corners,              // vertices of min rectangles of slices
    fittedPoleLines_points;     // this is the vertices of fitted line to the pole carrier
    LineSegments3D linesegments;
    int next_number;
    /// all the operations in this for loop is per segment. Each segment is assumed a pole object
    for (auto &poleComp : poleComponent_vec) {
        if (poleComp.size() < 5) continue;
        int seg_num = poleComp[0].Attribute(tag);
        printf("\nSegment Number: %d \n", seg_num);
        LaserPoints sliced_points; // output of slicePoints_by_height
        vector <LaserPoints> slices_vec;
        //slicePoints_Z_axis(lpoints, sliced_points, 4);
        /// sliceTag is stored in ComponentNumberTag
        slices_vec = slicePoints_by_height(poleComp, sliced_points, slice_height, verbose);
        printf("num of slices: %d \n", slices_vec.size()); //debug
        lpoints_sliced.AddPoints(sliced_points);
        /// extract the center of gravity of slices
        LaserPoints centers_lp, this_pole_lp;
        int cnt=1;
        double width_previous;
        vector<double> accepted_widths_v;
        FILE *dimensions_infofile;
        if(verbose){
            strcpy (str_root, root);
            dimensions_infofile = fopen(strcat(str_root, "dimensions_infofile.txt"),"w");
            fprintf(dimensions_infofile, "width, diameter\n");
        }

        for (auto &s : slices_vec){
            if (s.size() < 4) continue;
            Position3D pos = s.CentreOfGravity(s.TaggedPointNumberList(tag, seg_num));
            centers_lp.push_back(pos.Position3DRef());
            /********************** compare slices and accept those without a drastic width change ********************/
            DataBoundsLaser db;
            db = s.DeriveDataBounds(0);
            /// Find the minimum enclosing rectangle for the segment in 2D
            s.DeriveTIN();
            ObjectPoints rect_vertices;
            LineTopology rect_edges;
            double max_edge_dist = 0.10;
            s.EnclosingRectangle(max_edge_dist, rect_vertices, rect_edges);
            /// get the width of the rectangle and compare to the previous width
            double len1 =fabs(rect_vertices[0].Distance2D(rect_vertices[1])) ;
            double len2 =fabs(rect_vertices[1].Distance2D(rect_vertices[2])) ;
            double diameter=NULL;
            if(verbose){
                diameter =fabs(rect_vertices[0].Distance2D(rect_vertices[2])) ; // we dont need this
                printf("diameter: %.2f & ", diameter);
            }

            double width = std::max(len1, len2);
            if(verbose) fprintf(dimensions_infofile, "%.2f,%.2f \n", width, diameter);
            if(verbose) printf("width: %.2f", width);  //DEBUG // \n is added later
            /// if current width is almost equal to the previous one then we update the width and add it to the accepted width
            // initialize the first width
            if(cnt==1) width_previous =width; // this is a sloppy assumption if the first width is not acceptable
            //if (sameDOUBLE(width, width_previous, width_diff_tolerance)){ // smaeDouble uses fabs
            if ((width - width_previous) < width_diff_tolerance){   // we want to accept the change if it is negative no matter of the threshold
                width_previous=width; // updating the previouse width
                accepted_widths_v.push_back(width);
                //accepted_slices.push_back(s);
                accepted_slicePoints.AddPoints(s); /// this keeps all the accepted poles in the laserpoints
                this_pole_lp.AddPoints(s); // this is refreshed in each iteration of poles
                if(verbose) printf(" ... accepted \n"); ////DEBUG
            } else if(verbose) printf("\n"); //DEBUG
            cnt++;
            /********************************************/
        }
        slices_centers.AddPoints(centers_lp);
        /// connect the centers to create the object skeleton or contour
        Points2PolyLine(centers_lp, 1, poles_sekeleton_lines, poles_sekeleton_points);
        /// add the corners and lines to one file to visualize in pcm
        Visualize2DRectangles(slices_vec, rects_corners, rects_edges, tag);
        /* compare slices and exclude large slices with large width changes
         //accepted_slicePoints = compare_slices(slices_vec, width_diff_tolerance, accepted_slices_lpVec); // there is a bug when I use this
         fit a RANSAC3D or a leastsquare line to the accepted points of this_pole
         and extend it to the bounds of the original pole component*/

        if(this_pole_lp.size() < 4) printf("ERROR: the extracted pole carrier is empty!");
/*        /// calculate the max_dist_to_line parameter from the pole carrier diameter instead of a fixed size
        double pole_diameter = calculate_laserpoints_2Ddiameter(this_pole_lp);
        if(pole_diameter < 0 || pole_diameter > 999.0 ||
                sameDOUBLE(pole_diameter, 0.0, width_diff_tolerance)){ // just to make sure the diameter is valid
            max_dist_toLine = max_dist_toLine; // if calculated diameter is not accepted technically do nothing
            if(verbose) printf("max_dist_toLine: %.2f \n ", max_dist_toLine);
        } else
        {  // else use the diameter instead of a fixed value
            max_dist_toLine = pole_diameter - width_diff_tolerance;
            if(verbose) printf("diameter <> max_dist_toLine: %.2f \n ", max_dist_toLine);
        }*/
        /// calculate the max_dist_to_line parameter from the accepted slices width
        std::pair <double, double> mean_stdev;
        mean_stdev = get_mean_and_stdev(accepted_widths_v);
        max_dist_toLine = mean_stdev.first + mean_stdev.second; // add the stdev to mean value as a max_dist threshold
        if(verbose) printf("mean: %.2f & stdev: %.2f of accepted slices. \n ", mean_stdev.first, mean_stdev.second);
        printf("max_dist_toLine: %.2f \n ", max_dist_toLine);

        /* TODO There is a problem in RANSAC3DLine() function that for poles that part of it is separated from the rest,-
         * for example a slice is on top of the pole and the rest on lower part, then the line fitting stuck in a loop and
         * it fits a bad line or it takes much longer. It multiples the calculation process by a lot.
         * */
        Line3D line;
        int min_hits = int(40.0 * this_pole_lp.size() / 100);
        int numhits;
        if(RANSAC_linefitting){
            LineSegments3D ransac_linesegments;
            if(verbose) printf("\nminimum num of RANSAC hits: %d \n", min_hits);
            // we fit the line to the accepted slices
            line = this_pole_lp.RANSAC3DLine(max_dist_toLine, min_hits,
                                             500, numhits, 1.0, ransac_linesegments);
            printf("num of RANSAC hits: %d \n", numhits);
        } else
            line = this_pole_lp.FitLine(this_pole_lp.begin(), this_pole_lp.end()); // a least square fitting is applied

        //if (numhits >= min_hits) { // we can use this condition to make sure lines are more robust, but the assumption is that the pole is already refined.
        /// extract the inliers for the fitted line and store the remaining points which are pole attachments
        for (auto &point : poleComp) {
            if (line.DistanceToPoint(point.Position3DRef()) <= max_dist_toLine) {
                inlierpoints.push_back(point);
            } else remaining_points.push_back(point);
        }
        //}

        /*********** This section is not necessary, it is just to visualize the correct line segment ******************/
        //TODO why not instead of creating a linesegment from the line, use the linesegments output from RANSAC3DLine()?? and then extend it
        DataBoundsLaser this_dbounds = this_pole_lp.DeriveDataBounds(0);
        double scalar_begin = line.Scalar(this_dbounds.Minimum()); // using these scalars from dbound is error prone, for inclined poles should be checked
        double scalar_end = line.Scalar(this_dbounds.Maximum());
        double sbegin, send;
        /// this is a better solution to extract scalars of the line to create a linesegment
        if(this_pole_lp.FaceNearLine(this_pole_lp.TaggedPointNumberList(
                SegmentNumberTag, seg_num), line, max_dist_toLine, sbegin, send)){
            scalar_begin = sbegin;
            scalar_end = send;
            //printf("line scalars updated!!! \n"); //DEBUG
        }
        LineSegment3D extended_linesegment;
        DataBoundsLaser originalpole_dbounds = poleComp.DeriveDataBounds(0);
        extended_linesegment = extend_Line3D(originalpole_dbounds,
                LineSegment3D(line, scalar_begin, scalar_end));
        extended_linesegment.Number()=seg_num;
        linesegments.push_back(extended_linesegment);
        ObjectPoints fittedline_objpoints;
        LineTopologies fittedline_top;
        linesegments.PointsWithTopology(fittedline_objpoints, fittedline_top, 1);
        fittedline_top.SetAttribute(LineLabelTag, seg_num);
//           printf("num of lines: %d \n", fittedline_top.size());
        if (!fittedPoleLines_points.empty())
            fittedline_top.ReNumber(fittedline_objpoints, (fittedPoleLines_points.end() - 1)->Number() + 1,
                                    (fittedPoleLines_topo.end() - 1)->Number() + 1);
        fittedPoleLines_topo.insert(fittedPoleLines_topo.end(), fittedline_top.begin(), fittedline_top.end());
        fittedPoleLines_points.insert(fittedPoleLines_points.end(), fittedline_objpoints.begin(), fittedline_objpoints.end());
        /**************************************************************************************************************/
    } // end of a pole processing

    /// write slices laserpoints to the output
    strcpy(str_root, root);
    lpoints_sliced.Write(strcat(str_root, "/sliced.laser"),false);
    strcpy(str_root, root);
    accepted_slicePoints.Write(strcat(str_root, "/accepted_slicePoints.laser"),false);

    /// write all centers of slices of poles
    strcpy(str_root, root);
    slices_centers.Write(strcat(str_root, "/slices_centers.laser"), false);

    /// write extracted centerline of poles
    strcpy(str_root, root);
    poles_sekeleton_points.Write(strcat(str_root, "/center_points.objpts"));
    strcpy(str_root, root);
    poles_sekeleton_lines.Write(strcat(str_root, "/center_lines.top"), false);

    /// write all rectangles of slices
    strcpy(str_root, root);
    rects_corners.Write(strcat(str_root, "/rects_corners.objpts"));
    strcpy(str_root, root);
    rects_edges.Write(strcat(str_root, "/rects_edges.top"), false);

    /// write lines of accepted pole carriers
    strcpy(str_root, root);
    fittedPoleLines_points.Write(strcat(str_root, "/polecarrier_points.objpts"));
    strcpy(str_root, root);
    fittedPoleLines_topo.Write(strcat(str_root, "/polecarrier_line.top"), false);

    /// write carrier points and attachment points in separate files
    strcpy(str_root, root);
    inlierpoints.Write(strcat(str_root, "/inlierpoints.laser"), false);
    strcpy(str_root, root);
    remaining_points.Write(strcat(str_root, "/remaining_points.laser"), false);

}

// Store points and topology in one file
// this functions connects all the points in the objpoints_in to create lines_topo_out
void Renumber_objpoints(ObjectPoints &objpoints_in, LineTopologies &lines_topo_out,
        ObjectPoints &objpoints_out, int seg_num, bool close_polygon) {
    int next_number;
    if (objpoints_out.empty()){
        next_number = 0;
    } else {
        next_number = (objpoints_out.back ()).Number () + 1;
    }
    /// update the numbering for objectpoints of objpoints_out
    PointNumber pnumber;
    LineTopology lines_top;
    for (int i=0; i < objpoints_in.size() ; i++){
        pnumber = PointNumber(next_number + i);
        //objpoints_out = ObjectPoint(corners[i], pnumber, cov3d);
        objpoints_in[i].NumberRef () = pnumber;
        objpoints_out.push_back(objpoints_in[i]);
        lines_top.push_back(pnumber);  // making the linetopology
    }
    if (close_polygon)
        lines_top.push_back(PointNumber(next_number)); // Close the polygon


    lines_top.Number () = seg_num; //++line_number;
    lines_topo_out.push_back(lines_top);
}

// Store points and topology in one file
// this functions creates list of lines or polygons in linestopo_in, finds the corresponding points in objpoints_in
/// then gets the last number in objpoints_out and renumber vertices and lines and adds them to the output
/* This function needs to be tested it is not finalized */
void Renumber_objpointsANDlinestopo(ObjectPoints &objpoints_in, LineTopologies &linestopo_in,
                    ObjectPoints &objpoints_out, LineTopologies &lines_topo_out, int seg_num, bool close_polygon) {
    int next_number;
    if (objpoints_out.empty()){
        next_number = 0;
    } else {
        next_number = (objpoints_out.back ()).Number () + 1;
    }
    /// update the numbering for objectpoints of objpoints_out
    PointNumber next_pointNumber;
    LineTopology line_top;
    for (int l=0; l < linestopo_in.size(); l++){
        PointNumberList pnumlist;
        pnumlist = linestopo_in[l].PointNumberListReference();
        for (int i=0; i < pnumlist.size() ; i++){
            ObjectPoint objpoint = objpoints_in.GetPoint(pnumlist[i].NumberRef())->ObjectPointRef();
            next_pointNumber = PointNumber(next_number + i);
            objpoint.NumberRef() = next_pointNumber;
            objpoints_out.push_back(objpoint);
            line_top.push_back(next_pointNumber);  // making the linetopology
        }
        if (close_polygon)
            line_top.push_back(PointNumber(next_number)); // Close the polygon
    }

    line_top.Number () = seg_num; //++line_number;
    lines_topo_out.push_back(line_top);
}

/// not a finished function as we change to laserpoints slices to do the same, see compare_slices()
void compare_rectangles (LineTopologies &rects_topo, ObjectPoints &rects_corners, double tolerance){

    vector <double> width_accepted;
    double width_previous;
    vector <LineTopologies> accepted_rects_edges;
    vector <ObjectPoints> accepted_rects_vertices;
    for (auto &rect : rects_topo){
        PointNumberList vertices_pnumlist;
        vertices_pnumlist = rect.PointNumberListReference();
        ObjectPoints rect_vertices;
        /// for each rectangle get the area, width and center and compare them to normalize the width.
        for(int i =0; i < vertices_pnumlist.size(); i++){
            ObjectPoint vertex;
            vertex = rects_corners.GetPoint(vertices_pnumlist[i].NumberRef())->ObjectPointRef();
            rect_vertices.push_back(vertex);
        }
        //double rect_area = rect.CalculateArea(rect_vertices);
        double len1 =fabs(rect_vertices[0].Distance2D(rect_vertices[1])) ;
        double len2 =fabs(rect_vertices[1].Distance2D(rect_vertices[2])) ;
        double width = std::max(len1, len2);
        /// if current width is almost equal to the previous one then we update the width and add it to the accepted width
        // also we accept the corresponding rectangle and vertices
        if (sameDOUBLE(width, width_previous, tolerance)){
            width_previous=width;
            width_accepted.push_back(width);
        }
        /// collect a vector of rects and vertices
    }
}

/// note this function should start with an acceptable width, if the first width is rejected all the rest are rejected too
LaserPoints compare_slices (vector <LaserPoints> slices_vec, double tolerance, vector<LaserPoints> &accepted_slices){
    /*1. read each slice, 2. calcualte the min_rectangle, 3. compare the width of rectangles,
     * 4. exclude rects with big changes in the width*/
    LaserPoints accepted_pointslp;
    LaserPointTag tag;
    int cnt=1;
    vector <double> width_accepted;
    double width_previous;
    for (auto &s : slices_vec){
        if (s.size() > 5) {
            DataBoundsLaser db;
            db = s.DeriveDataBounds(0);
            /// Find the minimum enclosing rectangle for the segment in 2D
            s.DeriveTIN();
            ObjectPoints rect_vertices;
            LineTopology rect_edges;
            double max_edge_dist = 0.10;
            s.EnclosingRectangle(max_edge_dist, rect_vertices, rect_edges);
            /// get the width of the rectangle and compare to the previous width
            double len1 =fabs(rect_vertices[0].Distance2D(rect_vertices[1])) ;
            double len2 =fabs(rect_vertices[1].Distance2D(rect_vertices[2])) ;
            double width = std::max(len1, len2);
            printf("width: %.2f", width);  //DEBUG // \n is added later
            /// if current width is almost equal to the previous one then we update the width and add it to the accepted width
            // initialize the first width
            if(cnt==1) width_previous =width; // this is a sloppy assumption if the first width is not acceptable
            if (sameDOUBLE(width, width_previous, tolerance)){
                width_previous=width;
                width_accepted.push_back(width);
                accepted_slices.push_back(s);
                accepted_pointslp.AddPoints(s);
                printf(" ... accepted \n"); ////DEBUG
            } else printf("\n"); //DEBUG
        }
        cnt++;
        if (cnt==7) //DEBUG
            printf("DEBUG \n");
    } // end of slices
    return accepted_pointslp;
}

