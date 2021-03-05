//
// Created by NikoohematS on 2-3-2017.
//

#include <algorithm>
#include <fstream>
#include <ctime>
#include <iterator>
#include "LaserPoints.h"
#include "Buffer.h"
#include "Buffers.h"
#include "utils.h"
#include "LineTopology.h"


bool comparesize (const LaserPoints &lp1, const LaserPoints &lp2)
{ return lp1.size() > lp2.size(); }

bool compareweight (const Buffer &b1, const Buffer &b2)
{ return b1.Weight() > b2.Weight(); }

bool OverlapXY1(const DataBoundsLaser &bounds1, const DataBoundsLaser &bounds2)
{
    if (bounds1.MinimumXIsSet() && bounds2.MaximumXIsSet() &&
        bounds1.Minimum().X() > bounds2.Maximum().X()) return false;    // 1 is left of 2
    if (bounds1.MaximumXIsSet() && bounds2.MinimumXIsSet() &&
        bounds1.Maximum().X() < bounds2.Minimum().X()) return false;    // 1 is right of 2
    if (bounds1.MinimumYIsSet() && bounds2.MaximumYIsSet() &&
        bounds1.Minimum().Y() > bounds2.Maximum().Y())  return false;    // 1 is above 2
    if (bounds1.MaximumYIsSet() && bounds2.MinimumYIsSet() &&
        bounds1.Maximum().Y() < bounds2.Minimum().Y())   return false;   // 1 is below 2

    return true;
}

DataBoundsLaser EnlargeDBounds(const DataBoundsLaser &bounds, double enlargement_size){

    DataBoundsLaser enlargedbounds;
    enlargedbounds = bounds;

    /// enlarge min and max X
    if (bounds.MinimumXIsSet()) {
        enlargedbounds.Minimum().SetX(bounds.Minimum().GetX() - enlargement_size);
    }
    if (bounds.MaximumXIsSet()) {
        enlargedbounds.Maximum().SetX(bounds.Maximum().GetX() + enlargement_size);
    }

    /// enlarge min and max Y
    if (bounds.MinimumYIsSet()) {
        enlargedbounds.Minimum().SetY(bounds.Minimum().GetY() - enlargement_size);
    }
    if (bounds.MaximumYIsSet()) {
        enlargedbounds.Maximum().SetY(bounds.Maximum().GetY() + enlargement_size);
    }

    /// set Z without change
    if (bounds.MinimumZIsSet()) {
        enlargedbounds.Minimum().SetZ(bounds.Minimum().GetZ());
    }
    if (bounds.MaximumZIsSet()) {
        enlargedbounds.Maximum().SetZ(bounds.Maximum().GetZ());
    }

    return enlargedbounds;
}

DataBoundsLaser ShrinkDBounds(const DataBoundsLaser &bounds, double shrinking_size){

    DataBoundsLaser shrinkedbounds;
    shrinkedbounds = bounds;

    /// enlarge min and max X
    if (bounds.MinimumXIsSet()) {
        shrinkedbounds.Minimum().SetX(bounds.Minimum().GetX() + shrinking_size);
    }
    if (bounds.MaximumXIsSet()) {
        shrinkedbounds.Maximum().SetX(bounds.Maximum().GetX() - shrinking_size);
    }

    /// enlarge min and max Y
    if (bounds.MinimumYIsSet()) {
        shrinkedbounds.Minimum().SetY(bounds.Minimum().GetY() + shrinking_size);
    }
    if (bounds.MaximumYIsSet()) {
        shrinkedbounds.Maximum().SetY(bounds.Maximum().GetY() - shrinking_size);
    }

    /// set Z without change
    if (bounds.MinimumZIsSet()) {
        shrinkedbounds.Minimum().SetZ(bounds.Minimum().GetZ());
    }
    if (bounds.MaximumZIsSet()) {
        shrinkedbounds.Maximum().SetZ(bounds.Maximum().GetZ());
    }

    return shrinkedbounds;
}

/// shrink or enlarge the minimum enclosing rectangle or any other polygon
/// the scalefactor is the percentage of original size: e.g. 0.98, e.g. 1.02
ObjectPoints ScaleRectangle (ObjectPoints &corners, LineTopology &edges, double scalefactor){

    //if (!edges.IsClockWise (corners)) edges.MakeClockWise (corners);
    /// calcualte the centroid of the rectangle
    Position2D centroid2D;
    centroid2D = edges.CalculateCentroid (corners);
    /// convert it to 3D point
    Position3D centroid3D;
    centroid3D.X () = centroid2D.X ();
    centroid3D.Y () = centroid2D.Y ();
    centroid3D.Z () = corners.begin ()->Z (); // just set the Z of one of the corners to the center

    /// rescale the rectangle about the centroid -> affine transformation
    ObjectPoints new_corners;
    for (auto &corner : corners){
        ObjectPoint new_corner;
        new_corner.X () = scalefactor * (corner.X () - centroid3D.X()) + centroid3D.X ();
        new_corner.Y () = scalefactor * (corner.Y () - centroid3D.Y()) + centroid3D.Y ();
        new_corner.Z () = corner.Z (); /// we don't scale the Z value
        new_corner.Number() = corner.Number (); /// number sequence of the corners
        new_corners.push_back (new_corner);
    }

    /// new_edges are the same as edges becasue the order and topology of vertices are not changed
    return new_corners;
}


LaserPoints segment_refinement(LaserPoints lp, int min_segment_size, double maxdistincomponent);

LaserPoints Project_points_to_Plane(LaserPoints lp, Plane plane)
{
    // generate laserpoints of projected points on the plane
    LaserPoints projected_points;
    for (int i=0; i<lp.size(); i++){
    Position3D projected_pos;
    LaserPoint projected_point;
    projected_pos = plane.Project(lp[i].Position3DRef());
    projected_point.SetX(projected_pos.GetX());
    projected_point.SetY(projected_pos.GetY());
    projected_point.SetZ(projected_pos.GetZ());
        projected_points.push_back(projected_point);
    }
    return projected_points;
}

Buffers GenerateWallPatches(LaserPoints lp, double dist, double angle, double dist_along_twosegments,
        char* root, bool verbose) {

    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    //root = (char*) "D://test//indoor_reconstruction//";
    strcpy (str_root,root); // initialize the str_root with root string

    //verbose = true; // override verbose
    bool project_segments_to_plane = false;
    bool do_segment_refinement = false;
    bool default_input = false;


    if (default_input){
        dist = 0.30;
        angle = 10.0; //in degrees
        dist_along_twosegments = 0.30;

        char *laserFile;
        laserFile = (char *)  "D://test//indoor_reconstruction//test_samples//walls_2ndIter_seg8cm.laser"; //crop_merge_seg8cm.laser"
        lp.Read(laserFile);
    }

    /// collect points without segment nr
/*    LaserPoints notsegmented_points;
    LaserPoints::iterator point_it;
    for (point_it = lp.begin(); point_it != lp.end(); point_it++){
        if (!point_it->HasAttribute(SegmentNumberTag)){
            notsegmented_points.push_back(*point_it);
        }
    }*/

    if (do_segment_refinement){
        /// resegmenting laserpoints by removing longedges in the TIN and removing deformed segments
        LaserPoints refined_lp;
        refined_lp = segment_refinement(lp, 2, dist);
        /// renew laserpoints with the result of segment_refinement
        lp = refined_lp;

        strcpy (str_root,root);
        //lp.Write("D://test//buffer_Kada//segment_refined.laser", false);
        lp.Write(strcat(str_root, "segment_refined.laser"), false);
    }

    vector<int> segment_numbers;

    if (lp.HasAttribute(SegmentNumberTag)) {
        segment_numbers = lp.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        printf ("Number of laser segments: %d\n", (int) segment_numbers.size());
    } else {
        printf("ERROR: laserPoints don't have segment number or are not segmented, Please use segmented laserpoints. \n");
        EXIT_FAILURE;
    }

    LaserPoints projected_segments;
    vector<Buffer> buffers;
    vector<int>::iterator segmentNr_it;
    int buffnr = 0;
    printf(" \n Constructing buffers ... \n");
    for (segmentNr_it = segment_numbers.begin(); segmentNr_it != segment_numbers.end(); segmentNr_it++) {
        buffnr++;
        LaserPoints segment_lpoints;
        segment_lpoints = lp.SelectTagValue(SegmentNumberTag, *segmentNr_it);

        if (project_segments_to_plane){
            LaserPoints seg_projectedpoints;
            Plane seg_plane;
            seg_plane = lp.FitPlane(*segmentNr_it, *segmentNr_it, SegmentNumberTag);
            seg_projectedpoints = Project_points_to_Plane(segment_lpoints, seg_plane);
            //seg_projectedpoints.Write("D://test//buffer_Kada//result//seg_projected_temp.laser", false);
            projected_segments.AddPoints(seg_projectedpoints);
            projected_segments.SetAttribute(LabelTag, 0);
        }
        //segments_lp.push_back(segment_lpoints);
        Buffer buffer(segment_lpoints, *segmentNr_it, buffnr);
        buffers.push_back(buffer);
    }

    if (project_segments_to_plane) strcpy (str_root,root),
                projected_segments.Write(strcat(str_root, "projected_segments.laser"), false);


     /** Check for merging buffers
     * */
    printf(" \n Merging process ... \n");
    std::sort(buffers.begin(), buffers.end(), compareweight);

    /// print sorted buffers' weight for check
    if(verbose){
        for (int i=0; i <buffers.size(); i++){
            printf("%d ", buffers[i].Weight());
        }
        printf ("\n");
    }

    vector <Buffer>::iterator buffers_it;
    vector <int> mergedbuffers_nr;
    vector <Buffer> mergedbuffers;
    int newmerge_buffnr=0;
    bool this_is_merged=false;
    for (buffers_it = buffers.begin(); buffers_it != buffers.end(); buffers_it++){
        if (verbose) printf("Buffer weight/seg: %d / %d \n",
                buffers_it->Weight(), buffers_it->BuffSegmentNumber());
        this_is_merged   = false;

/*        if (verbose){
            if(buffers_it->BuffSegmentNumber()==107){
                printf("halt \n");
            }
        }*/

        //check if current buffer is not already merged;
       if (std::find(mergedbuffers_nr.begin(), mergedbuffers_nr.end(), buffers_it->BuffNumber()) != mergedbuffers_nr.end()) {
           continue;
        }

        DataBoundsLaser bounds1;
        bounds1 = EnlargeDBounds(buffers_it->BuffBounds(), dist_along_twosegments/2);

        Buffers::iterator nextbuffer_it;
        Buffer new_buffer;
        newmerge_buffnr++;
        for (nextbuffer_it = buffers_it +1 ; nextbuffer_it < buffers.end(); nextbuffer_it++) {
            // check if nextbuffer is not already merged
            if (std::find(mergedbuffers_nr.begin(), mergedbuffers_nr.end(), nextbuffer_it->BuffNumber()) !=mergedbuffers_nr.end()){
                continue;
            }
            // if is not merged, prints the buffer info
            if (verbose){
                printf("Next buffer: weight: %d  / segment: %d \n",
                       nextbuffer_it->Weight(), nextbuffer_it ->BuffSegmentNumber());
            }


            if (verbose){
                if(nextbuffer_it->BuffSegmentNumber()==14){
                    printf("halt \n");
                }
            }

            DataBoundsLaser bounds2;
            bounds2 = EnlargeDBounds(nextbuffer_it->BuffBounds(), dist_along_twosegments/2);

            if(OverlapXY1(bounds1, bounds2)){ // MinIsSet and MaxISSet should be set to 1.0 for both bounds
                if(buffers_it->Merge(*nextbuffer_it, dist, angle, new_buffer, newmerge_buffnr)){
                    if (verbose){
                        strcpy (str_root,root);
                        new_buffer.BuffLaserPoints().Write(strcat(str_root, "ew_mergedbuffer_temp.laser"), false); //debug
                    }
                    mergedbuffers_nr.push_back(nextbuffer_it->BuffNumber());
                    *buffers_it = new_buffer;
                    this_is_merged  = true;
                }
            }
        } // 2nd for

        if(new_buffer.Weight()!=0){
            mergedbuffers.push_back(new_buffer);
        }
        if(!this_is_merged){ // if nothing is not merged into this_buffer,renumber and then we add it anyway to final_buffers
            buffers_it->SetBufferNr(newmerge_buffnr);
            mergedbuffers.push_back(*buffers_it);
        }
    }

    /* renumber segmentation of merged buffers
     **/
    int new_segmentnr =0;
    LaserPoints new_merged_segmentation;
    printf("\n Renumber merged segmentation... wait ...\n");
    std::sort(mergedbuffers.begin(), mergedbuffers.end(), compareweight);
    for (int i=0; i< mergedbuffers.size(); i++){
        new_segmentnr++;
        mergedbuffers[i].SetSegmentNr(new_segmentnr);
        LaserPoints mergebufflp;
        mergebufflp = mergedbuffers[i].BuffLaserPoints();
        new_merged_segmentation.AddPoints(mergebufflp);
    }
    strcpy (str_root,root);
    new_merged_segmentation.Write(strcat(str_root, "merged_segmentation.laser"), false);

    /* Check for including buffers in merged buffers
     * NOTE: include doesnt update the current buffer's plane and weight, just number of laserpoints and bounds will change
     * */
    printf(" \n Including process ... \n");
    vector <Buffer>::iterator mergedbuffers_it;
    vector <int> includebuffers_nr;
    vector <int> not_includedsegments;
    vector <Buffer> final_buffers, not_included;
    bool this_is_included=false;
    int newinclude_buffnr=0;

    for (mergedbuffers_it = mergedbuffers.begin(); mergedbuffers_it != mergedbuffers.end(); mergedbuffers_it++){

        if (verbose){
            printf("Buffer nr %d : weight/seg: %d / %d \n",
                    mergedbuffers_it->BuffNumber(), mergedbuffers_it->Weight(), mergedbuffers_it->BuffSegmentNumber());
        }

        this_is_included = false;

        //check if current buffer is not already included;
        if (std::find(includebuffers_nr.begin(), includebuffers_nr.end(),
                      mergedbuffers_it->BuffNumber()) != includebuffers_nr.end()) {
            continue;
        }

        DataBoundsLaser bounds1;
        bounds1 = EnlargeDBounds(mergedbuffers_it->BuffBounds(), dist_along_twosegments/2);

        Buffers::iterator nextinclude_it;
        Buffer newinclude_buffer;
        newinclude_buffnr++;
        for (nextinclude_it = mergedbuffers_it +1 ; nextinclude_it < mergedbuffers.end(); nextinclude_it++) {
            mergedbuffers_it->BuffLaserPoints().size();
            /// check if nextinclude buffer is not already included
            if (std::find(includebuffers_nr.begin(), includebuffers_nr.end(),
                          nextinclude_it->BuffNumber()) != includebuffers_nr.end()) {
                continue;
            }

            /// if is not included, prints the buffer
            if (verbose){
                printf("Next buffer nr %d : weight: %d  / segment: %d \n",
                       nextinclude_it->BuffNumber() , nextinclude_it->Weight(), nextinclude_it ->BuffSegmentNumber());
            }


            if (verbose){
                if(nextinclude_it->BuffSegmentNumber()==107){
                    printf("halt \n");
                }
            }

            DataBoundsLaser bounds2;
            bounds2 = EnlargeDBounds(nextinclude_it->BuffBounds(), dist_along_twosegments/2);

            if (OverlapXY1(bounds1, bounds2)){
                /// NOTE: include doesnt update the current buffer plane and weight, just number of laserpoints and bounds
                if(mergedbuffers_it->Include(*nextinclude_it, dist, angle, newinclude_buffer, newinclude_buffnr)){
                    if (verbose){
                        strcpy (str_root,root);
                        newinclude_buffer.BuffLaserPoints().Write(strcat(str_root, "new_includebuffer_temp.laser"), false); //debug
                    }
                    includebuffers_nr.push_back(nextinclude_it->BuffNumber());
                    *mergedbuffers_it = newinclude_buffer; // just buffer width and laserpoints will be updated
                    this_is_included  = true;
                }
            }
        } // end 2nd for

        if(newinclude_buffer.Weight()!=0){
            final_buffers.push_back(newinclude_buffer);
        }
        if(!this_is_included){ // if nothing is not included into this_buffer,renumber and then add it anyway to final_buffers
            mergedbuffers_it->SetBufferNr(newinclude_buffnr);
            //not_included.push_back(*mergedbuffers_it);
            //vector <int> not_includedsegments_this;
            //not_includedsegments_this = mergedbuffers_it->IncludedSegmentNumbers();
            //not_includedsegments.insert(not_includedsegments.begin(), not_includedsegments_this.begin(), not_includedsegments_this.end());
            final_buffers.push_back(*mergedbuffers_it);
        }
    } // end for


    vector <int> mergedsegments_vec;
    int new_finalsegmentnr =0;
    LaserPoints new_segmentation, projected_buffers;
    vector <int> mergedsegments, merged_dominanat, includedsegments, include_dominanat;
    printf("\n Renumber final segmentation... wait ...\n");

    for (int i=0; i< final_buffers.size(); i++){
        new_finalsegmentnr++;

        final_buffers[i].SetSegmentNr(new_finalsegmentnr);
        final_buffers[i].SetBuffPlaneNumber(new_finalsegmentnr);
        LaserPoints bufflp;
        //bufflp.Initialise();
        bufflp = final_buffers[i].BuffLaserPoints();
        //printf("pointNr: %d , size: %d\n", bufflp[i].GetPointNumber(), bufflp.size()); // bug, points donst have PointNumber
        new_segmentation.AddPoints(bufflp);

        if (verbose){
            vector <int> mergedsegments_this;
            mergedsegments_this = final_buffers[i].MergedSegmentNumbers();
            if (mergedsegments_this.size() > 1){
                mergedsegments.insert(mergedsegments.end(), mergedsegments_this.begin()+1,
                                      mergedsegments_this.end());

                merged_dominanat.push_back(mergedsegments_this[0]);
            }

            vector <int> includesegments_this;
            includesegments_this = final_buffers[i].IncludedSegmentNumbers();
            if (includesegments_this.size() > 1){
                includedsegments.insert(includedsegments.end(), includesegments_this.begin()+1,
                                        includesegments_this.end());
                include_dominanat.push_back(includesegments_this[0]);
            }
        }

        if (project_segments_to_plane){
            LaserPoints buff_projectedpoints;
            Plane buff_plane;
            buff_plane = final_buffers[i].BuffPlane();
            buff_projectedpoints = Project_points_to_Plane(bufflp, buff_plane);
            buff_projectedpoints.SetAttribute(SegmentNumberTag, new_finalsegmentnr+10); // +10 to avoid be the same as segment numbers
            projected_buffers.AddPoints(buff_projectedpoints);
            projected_buffers.SetAttribute(LabelTag, 1);
        }
    }
    //new_segmentation.AddPoints(notsegmented_points);
    strcpy (str_root,root);
    new_segmentation.Write(strcat(str_root, "final_segmentation.laser"), false);

    vector <int> new_segment_nr;
    new_segment_nr = new_segmentation.AttributeValues(SegmentNumberTag);
    printf ("Number of new laser segments: %d\n", new_segment_nr.size());

    if (verbose){
        sort(new_segment_nr.begin(), new_segment_nr.end());
        sort(segment_numbers.begin(), segment_numbers.end());
        sort(mergedsegments.begin(), mergedsegments.end());
        sort(includedsegments.begin(), includedsegments.end());

        std::ofstream               new_segments_file("..//new_segments_file.txt");
        std::ostream_iterator<int>  new_segments_iterator(new_segments_file, ",");
        std::copy(new_segment_nr.begin(), new_segment_nr.end(), new_segments_iterator);

        std::ofstream               segments_file("..//segmentnumbers.txt");
        std::ostream_iterator<int>  segments_iterator(segments_file, ",");
        std::copy(segment_numbers.begin(), segment_numbers.end(), segments_iterator);

        std::ofstream               merge_file("..//result//mergedsegments.txt");
        std::ostream_iterator<int>  merge_iterator(merge_file, ",");
        std::copy(mergedsegments.begin(), mergedsegments.end(), merge_iterator);

        std::ofstream              include_file("..//includedsegments.txt");
        std::ostream_iterator<int> include_iterator(include_file, ",");
        std::copy(includedsegments.begin(), includedsegments.end(), include_iterator);

        sort(not_includedsegments.begin(), not_includedsegments.end());
        std::ofstream              notinclude_file("..//NOTincludedsegments.txt");
        std::ostream_iterator<int> notinclude_iterator(notinclude_file, ",");
        std::copy(not_includedsegments.begin(), not_includedsegments.end(), notinclude_iterator);

        sort(merged_dominanat.begin(), merged_dominanat.end());
        sort(include_dominanat.begin(), include_dominanat.end());

        std::ofstream              mergeDOM_file("..//mergeDOM_filedsegments.txt");
        std::ostream_iterator<int> mergeDOM_file_iterator(mergeDOM_file, ",");
        std::copy(merged_dominanat.begin(), merged_dominanat.end(), mergeDOM_file_iterator);


        std::ofstream              includeDOM_file("..//includeDOMdsegments.txt");
        std::ostream_iterator<int> includeDOM_iterator(includeDOM_file, ",");
        std::copy(include_dominanat.begin(), include_dominanat.end(), includeDOM_iterator);
    }


    if (verbose){
        // debug
        LaserPoints notincluded;
        for (int i=0; i< not_included.size(); i++){

            LaserPoints bufflp;
            bufflp = not_included[i].BuffLaserPoints();
            notincluded.AddPoints(bufflp);
        }
        strcpy (str_root,root);
        notincluded.Write(strcat(str_root, "notincluded.laser"), false);
    }

    if (project_segments_to_plane){
        strcpy (str_root,root);
        projected_buffers.Write(strcat(str_root, "projected_buffers.laser"), false);
        //projected_buffers.Write("..//projected_buffers.laser", false);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total buffers processing time: "<< duration/60 << "m" << '\n';

    /// convert vector <buffers> to Buffers object
    Buffers segment_buffers;
    segment_buffers.insert(segment_buffers.end(), final_buffers.begin(), final_buffers.end());

    return segment_buffers;

}
