//for histogram
static float subClass=0.01;
static int myClass=1;
static int nClass=200;
static int Sta[200];
static float center=0.00f;
static float Centers[160];
//for file path
static char *link1=(char *) "D:\\data2008_1\\";
static char *link2=(char *) "D:\\dataAHN_1\\";
static char *link3=(char *) "D:\\datamix_1\\";
static char *link4=(char *) "D:\\image_1\\";
static char *origintile=(char *) "D:\\Laserdata\\Rotterdam\\lastp2008\\";
static char *filter=(char *) ".laser";
static char *tilefilter=(char *) ".tile";
static char *imagefilter=(char *) ".jpg";

struct Change
{
       float dist;
       int drct_z;
       int drct_xy;
       };


       
