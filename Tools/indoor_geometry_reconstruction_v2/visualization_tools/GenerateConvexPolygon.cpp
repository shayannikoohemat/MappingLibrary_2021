//
// Created by NikoohematS on 11-10-2017.
//

#include <iostream>
#include <vector>
#include "LaserPoints.h"
#include "LaserPoint.h"

/// derived from LaserPoints class
void FindHull(LaserPatches &lps_tmp,LaserPatch &sk, Position3D P, Position3D Q, int num, int flag, int vertical)
{
    if(sk.size()==0)
        return;
    vector<Position3D>::iterator iter_tmp,iter_tmp2;
    LaserPatch s1,s2;
    Position3D C;
    Vector2D vec,p1,q1,c1;

    Line2D l;
    if(vertical)  //if vertical, use XZ plane
        l=Line2D(Position2D(P.GetX(),P.GetZ()),Position2D(Q.GetX(),Q.GetZ()));
    else
        l=Line2D(Position2D(P.GetX(),P.GetY()),Position2D(Q.GetX(),Q.GetY()));

    double maxdistance=-32576,distance=0;
    vector<Position3D>::iterator iter, iter1;


    for(iter=sk.begin();iter!=sk.end();iter++)
    {
        if(vertical)
            distance=l.DistanceToPoint(Position2D((*iter).GetX(),(*iter).GetZ()));
        else
            distance=l.DistanceToPoint(Position2D((*iter).GetX(),(*iter).GetY()));
        if(distance>maxdistance)
        {
            maxdistance=distance;
            iter_tmp=iter;
        }

    }

    C=*iter_tmp;

    for(iter_tmp2=lps_tmp[num].begin();iter_tmp2!=lps_tmp[num].end();iter_tmp2++)
    {
        if(*iter_tmp2==P)
        {iter=iter_tmp2;iter++;break;}
    }


    lps_tmp[num].insert(iter,C);

    for(iter=sk.begin();iter!=sk.end();iter++)
    {
        if(iter!=iter_tmp)
        {
            if(vertical)
            {
                vec=Vector2D((*iter).GetX(),(*iter).GetZ());
                p1=Vector2D(P.GetX(),P.GetZ());
                c1=Vector2D(C.GetX(),C.GetZ());
                q1=Vector2D(Q.GetX(),Q.GetZ());
            }
            else
            {
                vec=(Vector2D)(*iter);
                p1=(Vector2D)P;
                c1=(Vector2D)C;
                q1=(Vector2D)Q;
            }
            if(vec.AreaSign(p1,c1)==-1)
                s1.push_back(*iter);

            if(vec.AreaSign(c1,q1)==-1)
                s2.push_back(*iter);


        }
    }

    FindHull(lps_tmp,s1,P,C,num,flag,vertical);
    FindHull(lps_tmp,s2,C,Q,num,flag,vertical);
}


/// derived and modified from LaserPoints class: void ConvexHull(ObjectPoints &objpts, LineTopologies &tops, int max) const;
void ConvexHull(ObjectPoints &objpts, LineTopologies &tops, int max, LaserPoints lp, const LaserPointTag &tag)

{
    Position3D p3d,a,b;
    LaserPatches lps_tmp;
    LaserPoints testpoints;
    double max_x=-65535, min_x=65535;
    double max_i=0, min_i=0;
    double tmp_min=0, tmp_max=0;
    Plane myplane;
    LineTopology Piece_out;
    ObjectPoint vertex_out;
    int obj_num=0;
    LineTopologies::const_iterator line_in, line_out;
    LineTopologies::const_iterator temp;
    int t=0;
    int i,vertical,j=0;


    lps_tmp=LaserPatches(max+1);

    cout<<"The max segmenttag value is "<<max<<endl;

    //start loop for all Pieces
    for(i=0;i<=max;i++){
        max_x=-32576999;
        min_x=33576999;
        max_i=0, min_i=0;
        tmp_min=0, tmp_max=0;

        testpoints=lp.SelectTagValue(tag,i);

        if(testpoints.size()<3)
        {
            testpoints.ErasePoints(true);
            continue;
        }

        printf("No. %d \r",i);
        myplane=testpoints.FitPlane(i,i,tag);

        if(myplane.SmallestEigenvalue () ==0 )
            continue;


        if(myplane.IsVertical(0.1))
        {
            vertical=1;
        }
        else vertical=0;

        lps_tmp[i].setPlane(myplane);

        for(j=0;j<testpoints.size();j++)  //make a segment more flat by projecting all points to plane
        {
            testpoints[j]=myplane.Project(testpoints[j]);
        }

        //cout<<"size: "<<testpoints.size()<<endl;
        for(j=0;j<testpoints.size();j++)   //Determine most left and right points
        {
            if((tmp_min=testpoints[j].GetX())<=min_x)
            {
                min_x=tmp_min;
                min_i=j;
            }
            if((tmp_max=testpoints[j].GetX())>=max_x)
            {
                max_x=tmp_max;
                max_i=j;
            }
        }

        a= Position3D(testpoints[min_i].GetX(),testpoints[min_i].GetY(),testpoints[min_i].GetZ());
        b= Position3D(testpoints[max_i].GetX(),testpoints[max_i].GetY(),testpoints[max_i].GetZ());

        lps_tmp[i].push_back(a);
        lps_tmp[i].push_back(b);
        LaserPatch s1,s2;


        for(j=0;j<testpoints.size();j++)
        {
            if((j!=min_i)&&(j!=max_i))
            {
                p3d=Position3D(testpoints[j].GetX(),testpoints[j].GetY(),testpoints[j].GetZ());
                if(((Vector2D)p3d).AreaSign(a,b)==-1)
                    s1.push_back(p3d);
                else
                    s2.push_back(p3d);
            }
        }
        FindHull(lps_tmp,s1,a,b,i,1,vertical);
        FindHull(lps_tmp,s2,b,a,i,0,vertical);

    }

    testpoints.ErasePoints(true);
    cout<<"Find Hull complete"<<endl;

    objpts.clear();
    tops.clear();


    for(t=0;t<=max;t++)
    {
        if(lps_tmp[t].size()==0)
            continue;

        Piece_out.clear();
        Piece_out.Label()=t;

        for(int t1=0;t1<lps_tmp[t].size();t1++)
        {

            vertex_out=ObjectPoint(lps_tmp[t][t1].GetX(),lps_tmp[t][t1].GetY(),lps_tmp[t][t1].GetZ(),obj_num,0,0,0,0,0,0);
            Piece_out.push_back(PointNumber(obj_num));
            objpts.push_back(vertex_out);
            obj_num++;
        }

        Piece_out.push_back(Piece_out[0]);

        tops.push_back(Piece_out);

    }
    // building_patches.clear();

    for(i=0;i<tops.size();i++)
        tops[i].push_back(tops[i][0]);

    lps_tmp.Release();
}

void GenerateConvexPolygon(ObjectPoints &corners, LineTopologies &polygons,
                           LaserPoints segmented_lp, const LaserPointTag &tag){

    vector<int> segments_no_v;
    segments_no_v = segmented_lp.AttributeValues(tag);  // tag: e.g. SegmentNumberTag
    sort(segments_no_v.begin(), segments_no_v.end());
    int max_seg_no=0;
    if(segments_no_v.size()) max_seg_no = segments_no_v.back();
    ConvexHull(corners, polygons, max_seg_no, segmented_lp, tag);
}





