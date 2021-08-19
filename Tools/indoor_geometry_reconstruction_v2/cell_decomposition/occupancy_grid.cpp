#include "LaserPoints.h"
#include "Laservoxel.h"

void occup_grid(LaserPoints lp, double vox_size, char *out_dir){

    char str_root[500];
    ///initialize the LaserVoxel
    LaserVoxel vox(lp, vox_size);
    LaserPoints seg_vox_centers;

    /// generating vox_centers and generating a vector to relate voxel ijk and centers
    LaserPoints vox_centers, vox_centers_occupied;
    vector< vector < vector < int > > > vec_ijk;
    vec_ijk = vox.export_vox_centres(1, vox_centers);
    cout << "num of vox_centers: " << vox_centers.size() << endl;
    strcpy (str_root, out_dir);
    vox_centers.Write(strcat(str_root, "vox_centers.laser"), false);

    /// generating occupied voxels
    vox_centers_occupied = vox_centers.SelectTagValue(LabelTag, 1);
    cout << "num of occupied vox centers: " << vox_centers_occupied.size() << endl;
    strcpy (str_root, out_dir);
    vox_centers_occupied.Write(strcat(str_root, "vox_centers_occupied.laser"), false);
}


//int main(){
//    char *out_dir = (char*) "./out_dir";
//    LaserPoints lp;
//    lp.Read("input.laser");
//    occup_grid(lp, 0.10, out_dir);
//}
