//
// Created by NikoohematS on 1-3-2017.
//

#ifndef INDOOR_TOPOLOGY_BUFFER_H
#define INDOOR_TOPOLOGY_BUFFER_H

#include "LaserPoints.h"

const double PI = 3.14159;

class Buffer {

protected:
    Vector3D normal;
    Plane plane;
    int weight;
    double buffer_width;
    int buffer_nr;
    int buffer_segnr;
    DataBoundsLaser buffbounds;
    int bufflabel;
    LaserPoints laserpoints;
    vector<int> MergedSegNr;
    vector<int> IncludeSegNr;

private:

    Plane BuffFitPlane(LaserPoints lpoints, int planenumber) {
        LaserPoints::iterator point;
        Plane                 pl;

        for (point=lpoints.begin(); point!=lpoints.end(); point++) {
            pl.AddPoint(*point, false);
        }
        pl.Recalculate();
        pl.Number() = planenumber;

        return pl;
    }

    /* function to check distance of points on both side of the segment's plane and get the range
     * of distances. Then get the maximum distances of points and sum up two maximum distances from both side
     * as the segment thickness
     * */
double SegmentWidth(LaserPoints seg_lpoints, Plane seg_plane)
    {
        if (seg_lpoints.size() == 0) return EXIT_FAILURE;

        LaserPoints::iterator p_it;
        vector <double> distances_unsigend, distances_signed; // positive, negative doubles
        for (p_it = seg_lpoints.begin(); p_it != seg_lpoints.end(); p_it++){
            double pdistplane;
            pdistplane = p_it->Distance(seg_plane);
            if (pdistplane < 0.0){
                distances_signed.push_back(pdistplane);
            }else {
                distances_unsigend.push_back(pdistplane);
            }
        }
        // TODO: change it to signed values and get the front and back value of the distances
        // we dont need two different vectors for negative and positive
        std::sort(distances_signed.begin(), distances_signed.end()); // negative values
        std::sort(distances_unsigend.begin(), distances_unsigend.end()); // positive values

        double dist_max_usigned, dist_max_signed;
        dist_max_usigned = dist_max_signed = 0;
        if(distances_unsigend.size()!= 0)   dist_max_usigned = distances_unsigend.back();
        if(distances_signed.size() != 0)    dist_max_signed  = distances_signed.front();

        double segment_width=0.0;
        if (dist_max_signed!=0 && dist_max_usigned!=0){
            segment_width = fabs(dist_max_signed) + dist_max_usigned;
            //printf("segment width: %0.2f \n", segment_thickness);
        }
        return segment_width;
    }

public:

    // Default Constructor
    Buffer();

    // Construct buffer with new laserpoints
    Buffer(LaserPoints &segment_lp, int segment_nr, int buffernumber);

    /// Copy assignment operator
    Buffer& operator=(const Buffer& bf);

    // return Buffer Normalvector
    const Vector3D &BuffNormal() const
    { return normal;}

    // return Buffer plane
    const Plane &BuffPlane() const
    {return plane;}

    // return Buffer weight
    const int &Weight() const
    {return weight;}

    // return Buffer thickness (width)
    double BufferWidth()
    {return buffer_width; }

    // return Buffer number
    int BuffNumber()
    {return buffer_nr; }

    // return Buffer number
    int BuffSegmentNumber()
    {return buffer_segnr; }

    // return Buufer bounds, bbox axis aligned
    DataBoundsLaser BuffBounds()
    { return buffbounds;}

    // set label
    void SetLabel(int label);

    // set segment number
    void SetSegmentNr(int segmentnr);

    // set buffer number
    void SetBufferNr (int buffernr);

    // set buffer plane number
    void SetBuffPlaneNumber(int planenr);

    // return label
    int Label()
    {return bufflabel;}

    // return lp
    LaserPoints &BuffLaserPoints()
    {return laserpoints; }

    // list of included segments in the buffer
    vector<int> IncludedSegmentNumbers()
    { return IncludeSegNr;}

    // list of merged segmentsin the buffer
    vector<int> MergedSegmentNumbers()
    { return MergedSegNr;}

    // segment number of dominant segment
    int  DominantSegment();

    void ComputeBuffer (LaserPoints &segment_lp, int segment_nr, int buffnr);

    void ComputeIncludeBuffer (LaserPoints segment_lp, LaserPoints &include_lp, int segment_nr, int buffernumber);

    bool Merge(Buffer other_buffer, double dist_th, double angle_th, Buffer &new_buff
            , int new_buffer_nr);

    bool Include(Buffer other_buffer, double dist_th, double angle_th, Buffer &new_buff
            , int new_buffer_nr);

};


#endif //INDOOR_TOPOLOGY_BUFFER_H
