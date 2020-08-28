//
// Created by NikoohematS on 1-3-2017.
//

#include "Buffer.h"

bool comparison (const ObjectPoint &p1, const ObjectPoint &p2)
{
    return p1.GetX() < p2.GetX();
}

// Constructor
Buffer::Buffer(LaserPoints &segment_lp, int segment_nr, int buffer_number)
{
    ComputeBuffer(segment_lp, segment_nr, buffer_number);
    MergedSegNr.push_back(segment_nr);
    IncludeSegNr.push_back(segment_nr);
}

Buffer::Buffer()
{
    // Construct empty Buffer
    normal=Vector3D();
    plane.Initialise();
    weight=0;
    buffer_width=0.0;
    buffer_nr=0;
    buffer_segnr=0;
    buffbounds.Initialise();
    bufflabel=0;
    laserpoints.Initialise();
    MergedSegNr={0};
    IncludeSegNr={0};
}

void Buffer::ComputeBuffer (LaserPoints &segment_lp, int segment_nr, int buffernumber)
{
    laserpoints = segment_lp;

    //plane = segment_lp.FitPlane(pointslist, buffernumber)
    plane = BuffFitPlane(segment_lp, segment_nr);
    normal = plane.Normal();
    weight = segment_lp.size();
    buffer_nr = buffernumber;
    buffer_segnr = segment_nr;
    buffbounds = segment_lp.DeriveDataBounds(0);
    bufflabel=0;
    buffer_width = SegmentWidth(segment_lp, plane);
}

// calculate buffer with includable buffer and updates buffer_width and buffbounds, other attributes remain unchanged
void Buffer::ComputeIncludeBuffer (LaserPoints segment_lp, LaserPoints &include_lp, int segment_nr, int buffernumber)
{
    include_lp.size();
    segment_lp.size();
    laserpoints.AddPoints(include_lp);
    laserpoints.size();

    plane = BuffFitPlane(segment_lp, segment_nr);  // calculate plane of segment not all laserpoints
    normal = plane.Normal();
    weight = segment_lp.size(); /// ??? weight should be for biggest segment or all laserpoints in buffer
    buffer_nr = buffernumber;
    buffer_segnr = segment_nr;
    buffbounds = laserpoints.DeriveDataBounds(0);
    bufflabel=0;
    buffer_width = SegmentWidth(laserpoints, plane);
}

void Buffer::SetLabel(int label) {bufflabel = label; }

void Buffer::SetSegmentNr(int segmentnr)
{
    buffer_segnr = segmentnr;
    laserpoints.SetAttribute(SegmentNumberTag, segmentnr);
}

void Buffer::SetBufferNr(int buffernr) { buffer_nr = buffernr; }

void Buffer::SetBuffPlaneNumber(int planenr) {plane.Number() = planenr; }


/// NOTE: include doesnt update the current buffer's plane and weight, just number of laserpoints and bounds will change
bool Buffer::Include(Buffer other_buffer, double dist_th, double angle_th, Buffer &new_buff
                        , int new_buffer_nr)
{
    double anglebetween;
    if(normal.DotProduct(other_buffer.BuffNormal()) < 0){ // two normals are in opposite directions
        anglebetween = Angle(other_buffer.BuffNormal(), normal * -1);  /// ???? *-1  not sure is correct way to do it
    }else {
        anglebetween = Angle(other_buffer.BuffNormal(), normal);
    }

    double distancebetween;

    distancebetween = plane.Distance (other_buffer.BuffPlane().CentreOfGravity());
    Buffer this_buffer;
    this_buffer = *this; /// we reserve *this before getting updated
    if ((anglebetween * 180 / PI > angle_th) && (fabs(distancebetween) <= dist_th)){

        /// then include and calculate includebuffer
        ComputeIncludeBuffer(laserpoints, other_buffer.BuffLaserPoints(), other_buffer.buffer_segnr, new_buffer_nr);
        /// *this is updated now

        /// check if updated buffer width is not more than dist_th
        if (BufferWidth() > dist_th){
            *this = this_buffer; // if include is rejected return *this to previous values
            return false;
        } else{
            IncludeSegNr.insert(IncludeSegNr.end(), other_buffer.IncludeSegNr.begin(), other_buffer.IncludeSegNr.end());
            new_buff = *this;
            return true;
        }
    }

    return false;
}

bool Buffer::Merge(Buffer other_buffer, double dist_th, double angle_th, Buffer &new_buff
                    , int new_buffer_nr)
{

    double anglebetween;
    if(normal.DotProduct(other_buffer.BuffNormal()) < 0){ // two normals are in opposite directions
        anglebetween = Angle(other_buffer.BuffNormal(), normal * -1);  /// ???? *-1  not sure is correct way to do it
    }else {
        anglebetween = Angle(other_buffer.BuffNormal(), normal);
    }

    double distancebetween;

    distancebetween = plane.Distance (other_buffer.BuffPlane().CentreOfGravity());
    //distancediffTOorigin = abs(plane.Distance()) - abs(other_buffer.BuffPlane().Distance()); // not appropriate for comparing distance of two segments
    Buffer this_buffer;
    this_buffer = *this;
    if ((anglebetween * 180 / PI <= angle_th) && (fabs(distancebetween) <= dist_th)){
        /// updating laserpoints of current buffer
        laserpoints.AddPoints(other_buffer.BuffLaserPoints());
        /// then merge and calculate new buffer
        ComputeBuffer(laserpoints, other_buffer.buffer_segnr, new_buffer_nr);

        /// check if updated buffer width is not more than dist_th
        if (BufferWidth() > dist_th){
            *this = this_buffer;
            return false;
        } else{
            MergedSegNr.insert(MergedSegNr.end(), other_buffer.MergedSegNr.begin(), other_buffer.MergedSegNr.end());
            new_buff = *this;
            return true;
        }
    }
    return false;
}


int Buffer::DominantSegment()
{
    int dominant_segmentnr;
    dominant_segmentnr = MergedSegNr[0];
    return dominant_segmentnr ;
}

Buffer &Buffer::operator=(const Buffer &bf) {
    // Check for self assignment
    if (this == &bf) return *this;

    // Clear old data
    if (laserpoints.size() != 0){
        //laserpoints.ErasePoints();
        // Remove all attributes
        laserpoints.RemoveAttributes();

        // Delete all points
        if (!laserpoints.empty()) laserpoints.erase(laserpoints.begin(), laserpoints.end());
        // Delete TIN
        if (laserpoints.GetTIN()) laserpoints.EraseTIN();
    }

    // Copy new data
    normal = bf.normal;
    plane = bf.plane;
    weight = bf.weight;
    buffer_width = bf.buffer_width;
    buffer_nr = bf.buffer_nr;
    buffer_segnr = bf.buffer_segnr;
    buffbounds = bf.buffbounds;
    bufflabel = bf.bufflabel;
    laserpoints = bf.laserpoints;
    MergedSegNr = bf.MergedSegNr;
    IncludeSegNr = bf.IncludeSegNr;

    return *this;
}


