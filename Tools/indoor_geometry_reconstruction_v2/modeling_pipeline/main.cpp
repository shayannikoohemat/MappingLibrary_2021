#include <iostream>
#include <cstring>
#include "modeling_pipeline.h"
#include "LaserPoints.h"

using namespace std;


//void test(char *input_file, char* out_dir)
//{
//    /// write segmented ascii file
//    char char_arr[500];
//    FILE *lp_seg_ascii;
//    //std::string lp_seg_path2 = laser_dir + "/" + filename + "_seg.txt" ;
//    lp_seg_ascii = fopen(strcpy(char_arr, out_dir),"w");
//    fprintf(lp_seg_ascii, "x, y, z, segment_num \n");
//    LaserPoints lp_segmented;
//    lp_segmented.Read(input_file);
//    for(auto &p : lp_segmented){
//        if (p.HasAttribute(SegmentNumberTag))
//            fprintf(lp_seg_ascii, "%.2f, %.2f, %.2f", p.X(), p.Y(), p.Z());
//        else{
//            cerr << "points don't have segment number to convert to ascii!!!" << endl;
//        }
//    }
//}


int main()
{
    cout << "Hello World!" << endl;
    char * input_file = (char*) "/mnt/DataPartition/REscan/data/dump/lp_segmented.laser";
    threedmodeling(input_file, "/mnt/DataPartition/REscan/data", false);

    return 0;
}
