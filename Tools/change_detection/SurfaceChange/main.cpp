#include <cstdlib>
#include <iostream>
#include <fstream>
#include "mystruct.h"
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "LaserBlock.h" 
#include "LaserUnit.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "LineTopologies.h"
#include "Positions3D.h"
#include <stdio.h>
#include <windows.h>
using namespace std;
#define PI 3.1415926
double b0a0ck_offset=3.0;
vector<double>v1;

//get sub-part of a string
char *Substring(char *input, int location)
{
     return input+location;
     } 
//use the pointer to get the full name
string Getfullname(char *index)
{
       string name="";
       for(int i=0;i<strlen(index);i++)
        { name=name+*(index+i); } 
        return name;  
       }
       
void OutputFile(char *output, char *path, char *filename, char *myFilter) 
{
       strcpy(output,path);
       strcat(output,filename);
       strcat(output,myFilter);    
     }

int main(int argc, char *argv[])
{
void CompareDatasets(char *, char *, char *, char *, char *,char *);
//fstream fp; 
LaserPoints Lpts;
LaserBlock block_2008, block_AHN;
LaserBlock::iterator unitpt_2008,unitpt_AHN;
LaserUnit::iterator subunit_2008,subunit_AHN;
string a="",b="";
bool equal;
char *name2008,*nameAHN;
char *fixname2008,*fixnameAHN; 
//fp.open("2.txt",ios::out); 
//int time=0;
int cen=0;
  if(block_2008.ReadMetaData(argv[1]) && block_AHN.ReadMetaData(argv[2]))
  {
    unitpt_2008=block_2008.begin();    
    unitpt_AHN=block_AHN.begin();
    
   //loop over all tiles in epoch2008
   for(subunit_2008=unitpt_2008->begin();subunit_2008!=unitpt_2008->end();subunit_2008++)
   {
     name2008=subunit_2008->Name();
     fixname2008=name2008+9;
    // fixname2008=Substring(name2008,8);     
     a=Getfullname(fixname2008);
     equal=false;  
     //set Sta to zero           
   //  for(int p=0;p<nClass;p++){Sta[p]=0;} 
     //loop over all tiles in epochAHN
      for(subunit_AHN=unitpt_AHN->begin();subunit_AHN!=unitpt_AHN->end()&&!equal;subunit_AHN++)
     {
       //set center to zero
      // center=0.0;             
       nameAHN=subunit_AHN->Name();
       fixnameAHN=nameAHN+9;
       b=Getfullname(fixnameAHN);  
       //check the name of each tile     
       if(a==b)
       {
          // time=time+1;
           //fp<<b<<" "<<time<<"\n";           
           //generate the names of outputfiles           
           char *outputname2008,*outputnameAHN, *outputmix, *tilepath,*imagepath;
           outputname2008=(char*)malloc(strlen(name2008) + 50);
           outputnameAHN=(char*)malloc(strlen(nameAHN)+50);
           outputmix=(char*)malloc(strlen(fixname2008)+50);
           tilepath=(char*)malloc(strlen(fixname2008)+50);
           imagepath=(char*)malloc(strlen(name2008)+50);

           //generate output paths for data
           OutputFile(outputname2008,link1,name2008,filter);
           cout<<outputname2008<<"\n"; 
           OutputFile(outputnameAHN,link2,nameAHN,filter);   
           OutputFile(outputmix,link3,fixname2008,filter);
           OutputFile(tilepath,origintile,name2008,tilefilter);  
           OutputFile(imagepath,link4,name2008,imagefilter); 
           
           //compare 2 datasets and output them
           CompareDatasets(subunit_2008->PointFile(),subunit_AHN->PointFile(),outputname2008,outputnameAHN,outputmix,imagepath);
           //outputtile
          // CopyFile(tilepath,link3,true);
          // cout<<tilepath<<" "<<outputmix<<"\n";
          // Centers[cen]=center;
          // cen=cen+1;          
          // fp<<center<<"\n";
           free(outputname2008);
           free(outputnameAHN);
           free(outputmix);
           free(tilepath);
           free(imagepath);
           equal=true;
           a=b="";
        }         
       }     
     }
   }
   else printf("Error reading meta data of blocks!");
                                         
    system("PAUSE");
    return EXIT_SUCCESS;
}
