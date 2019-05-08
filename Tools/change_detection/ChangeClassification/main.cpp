#include "class_par.h"
#include <time.h>

extern void timer_start(clock_t *time1);
extern void timer_end(clock_t time1, char *string);

using namespace std;

#define PI 3.1415926
double b0a0ck_offset=3.0;
vector<double>v1;
     
void OutputFile(char *output, char *path, char *filename, char *myFilter) 
{
       strcpy(output,path);
       strcat(output,filename);
       strcat(output,myFilter);    
     }

int main(int argc, char *argv[])
{ 
  LaserPoints Classify(LaserPoints &, LaserPoints &, fstream &);
  LaserPoints Lpts,lpts_input_above,lpts_input_dtm,lpts_output;
  LaserBlock block;
  LaserBlock::iterator unitpt;
  LaserUnit::iterator subunit;
  char *filename;
  char *link= (char *) "d:\\feature\\";
  char *filter= (char *) ".laser";
  fstream fp;
  clock_t time_start;  
  
  fp.open("d:\\class.txt",ios::out);
  fp<<"tile,seg_num,seg_size,normal,ps,pss,dist2dtm,residual,pre-label\n";
   if(block.ReadMetaData(argv[1]))
  {
    unitpt=block.begin(); 
   //loop over all tiles
   for(subunit=unitpt->begin();subunit!=unitpt->end();subunit++)
   {
     char *myoutput;
     filename=subunit->Name();      
     myoutput=(char*)malloc(strlen(filename) + 30);
     OutputFile(myoutput,link,filename,filter);    
     Lpts.ErasePoints();     
     if(!Lpts.Read(myoutput,false))
     {        
        cout<<filename<<"\n";
        timer_start(&time_start);
        lpts_input_above.Read(subunit->PointFile());
            
        //ready, read points and filter points
        lpts_input_dtm.ErasePoints();   
        lpts_input_above.RemoveAttribute(LabelTag);
        lpts_input_above.RemoveAttribute(SegmentNumberTag);
        lpts_input_above.RemoveAttribute(PointNumberTag); 
        lpts_input_above.RemoveAttribute(PlaneNumberTag);
        
        lpts_input_dtm=lpts_input_above;
        lpts_input_dtm.RemoveFilteredPoints();
        lpts_input_above.RemoveGroundPoints();
        //-----------------for unfiltered data----------------------------
        //filter using morphology
        //--------------------------------------------------------------      
        //filter
        /*   Image kernel;    
        FilteringParameters filtering_parameters;
  
        // Create a filter kernel based on the specified parameters
        kernel.CreateFilterKernel(filtering_parameters.HeightDifAtSameLocation(),
                            -1.0, 0.0, -1.0, 0.0, -1.0, 0.0,
                            -1.0, 0.0, -1.0, 0.0,
                            filtering_parameters.FilterRadius(),
                            filtering_parameters.HeightDifAtFilterRadius(),
                            filtering_parameters.FilterRadius() / 1000.0);

        // Verify TIN and recreate edges
        lpts.VerifyTIN();
        lpts.EraseNeighbourhoodEdges();
        edges.Derive(lpts.TINReference());
        lpts.SetNeighbourhoodEdges(&edges);
  
        // Filter
        lpts.FilterOnMorphology(kernel, filtering_parameters.FilterRadius(),0.0, 0.0, edges);
        lpts.Write("abc.laser",0,0);
        lpts.EraseTIN();*/
        lpts_output=Classify(lpts_input_above,lpts_input_dtm,fp); 
        lpts_output.Write(myoutput,0,0);
        timer_end(time_start,myoutput);        
     }     
     Lpts.ErasePoints();
   }
  }  
   else printf("Error reading meta data of blocks!"); 
   //for ISPRS Test data
/*  char *myoutput;
   char *filename="test";
   myoutput=(char*)malloc(strlen(filename) + 30);
   OutputFile(myoutput,link,filename,filter);
   Classify(argv[1],argv[2],myoutput,fp);*/
   fp.close();
   return EXIT_SUCCESS;
}
