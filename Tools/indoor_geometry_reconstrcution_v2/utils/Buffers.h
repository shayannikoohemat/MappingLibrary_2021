//
// Created by NikoohematS on 6-3-2017.
//

#ifndef INDOOR_TOPOLOGY_BUFFERS_H
#define INDOOR_TOPOLOGY_BUFFERS_H

#include <vector>
#include "Buffer.h"
#include "LaserPoints.h"


class Buffers : public std::vector<Buffer>{

    /*
--------------------------------------------------------------------------------
                             Copy assignment
--------------------------------------------------------------------------------
*/
public:
    Buffers & operator = (const Buffers &buffers)
    {
        // Check for self assignment
        if (this == &buffers) return *this;
        if (!empty()) erase(begin(), end());
        if (!buffers.empty()) insert(begin(), buffers.begin(), buffers.end());
        return(*this);
    }


    LaserPoints Buffers_laserpoints(){
        LaserPoints new_segmentation;
        new_segmentation.Initialise();
        Buffers::iterator buffer_it;
        int new_finalsegmentnr=0;
        for (buffer_it = begin(); buffer_it != end(); buffer_it++){
            new_finalsegmentnr++;
            buffer_it->SetSegmentNr(new_finalsegmentnr);
            LaserPoints bufflp;
            bufflp = buffer_it->BuffLaserPoints();
            new_segmentation.AddPoints(bufflp);
        }
        return new_segmentation;
    }


/*
--------------------------------------------------------------------------------
                             Erase all lists
--------------------------------------------------------------------------------
*/

/*    void Erase()
    {
        for (Buffers::iterator buffer=begin(); buffer!=end(); buffer++) buffer->Erase();
        erase(begin(), end());
    }*/


};


#endif //INDOOR_TOPOLOGY_BUFFERS_H
