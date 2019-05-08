#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"
#include "InlineArguments.h"
#include "ImageLines.h"
#include <map>

#include "DataBounds3D.h"
#include "KNNFinder.h"
#include "cv.h"
#include "highgui.h"

//new, April 2015: for the ISPRS 3D labeling benchmark we need to use files where the class is not defined via color, but simply an id.
//the data comes in a txt file x y z class.
//>>> original software as in 2013/14 for own work still in the old mapping lib structure (3D supervised classification assessment)

//the first step is to use ascii2laser and interprete the 4th col as REFLECTANCE, eg ascii2laser -i output_RTrees.pts -r 4 -root output_RTrees_id_reflectance
//in this programm now there is an automation which checks if the color is given (default) or not. If not it interprets the reflectance and sets the color accordingly, via labelinfo. 
//Then the normal process is continued,
//eg, using the color as class indicator and for visualisation 


//Strategy for evaluation in the 3D labeling benchmark. Side constraints/facts
//-a participant might not deliver a) all classes and b) not a label for each point from the given dataset. Both issues need to be considered.
//--> to best cope with a) it is the best if during reading the input laserfile a list is maintained with actual available classes (eg vector<bool> where the index is the class number) ==> note that in the software anyhow a numbering from 0 to n-1 is assumed). Then in all processing steps (first of all for the measures and output generation) the availability of each class is checked.]//

//concerning b): a special class "not_classified" is defined. The evaluation goes through all ref. points. If a ref. point is from a class which is actually provided in the testdata set, BUT the point is not in the set, it is a not_classified for the respective class. If the class is anyhow not provided by the participant, BUT  if this point has another class, it counts of course also as false!



using namespace std;

bool check_value_in_vec(vector<uint> &v, uint val)
{
	for (uint i=0; i<v.size();i++)
		if (v[i]==val) return 1;

	return 0;
}

void confusion_matrix(vector<int> &ref, vector<int> &extr, ::map<uint, string> &label_name, char * name_template, vector <bool> & class_is_available, vector<int> & missing_in_validation, double *accuracy=NULL)
{
vector < vector < int > > confusion_abs;
vector < vector < double > > confusion_rel;

vector<int> TP_per_class,FN_per_class,FP_per_class;
vector<double> completeness_per_class,correctness_per_class,Fone_per_class;


//first fill the confusion matrix, irrespectively if a class was considered or not. This comes later in the statistis. However, we extend the matrix to be able to add the information for missed points in the refrenced
//to this end we leave the variable totallables, but add 1 for the memory allocation. By this means we always have the additional information separate and do not confuse it

vector<int> class_count_v; //index: labelid, number: number of reference labels
int totallabels=label_name.size();
printf("%d different labels\n", totallabels);

class_count_v.resize(totallabels+1,0);
confusion_abs.resize(totallabels+1);
confusion_rel.resize(totallabels+1);

TP_per_class.resize(totallabels,0);
FN_per_class.resize(totallabels,0);
FP_per_class.resize(totallabels,0);

completeness_per_class.resize(totallabels,0);
correctness_per_class.resize(totallabels,0);
Fone_per_class.resize(totallabels,0);


for (uint l=0;l<totallabels+1;l++)
	{
	confusion_abs[l].resize(totallabels+1,0);
	confusion_rel[l].resize(totallabels+1,0);
	}

int nsamples_all=ref.size();
if (ref.size() != extr.size()) {printf("error in confusion matrix computation: ref and extr do not have the same size\n");exit(0);}
//iterate over the points and if both reference AND extr have label != -1 : update the absolute statistics
for( int i = 0; i < nsamples_all; i++ ) //this is only for the ones looking from the validation set. Hereafter add all the ones from the missing_in_validationLS as an additional row in the confusionmatrx
    {
      int pref=ref[i];
      int pextr=extr[i];
        
        if ((pref == -1) || (pextr==-1)) continue;
        
	class_count_v[(uint)(pref)]++;
	confusion_abs[(uint)(pref)][(uint)(pextr)]++;
    }
 //now add the info from the missed refrence points
 for (int i=0; i< missing_in_validation.size(); i++)
  {
  //it basically adds a new entry in the respective row of the reference label and the last col (independent form the other validation classes)
  //the missing_in_validation[i] is the ref class label!
  int pref=missing_in_validation[i];
  class_count_v[(uint)(pref)]++;
  confusion_abs[(uint)(pref)][(uint)(totallabels)]++; //so totallabels is the last col!!!
   
  }
  
 //for old output format (eg used in ISPRJ J paper 2014, see old mapping lib gcc342 folder ). This is now taken from the 2D assessment code for the benchmark
 //statistics: vector < vector < int > > confusionabs , where first index (along rows): label referenc, second index (along cols): label predicted
//iterate over the segment ids. and for each add one counter in the respective map address (only if label in ref is != -1
//write the confusion matrix also to a textfile
char output_confusion[500];
strcpy(output_confusion,name_template);
strcat(output_confusion,"_confusion.txt");

char output_confusion_cvmat[500];
strcpy(output_confusion_cvmat,name_template);
strcat(output_confusion_cvmat,"_confusion.CVMAT");

char output_html[500];
strcpy(output_html,name_template);
strcat(output_html,"_confusion.html");

char output_json[500];
strcpy(output_json,"json_snippet.json");

FILE *output;
if ((output=fopen(output_confusion,"w")) == NULL)
	{
		printf("CANNOT OPEN the output file!\n");
		exit(0);
	}

FILE *out_html;
if ((out_html=fopen(output_html,"w")) == NULL)
	{
		printf("CANNOT OPEN the output file!\n");
		exit(0);
	}

FILE *F_J;
if ((F_J=fopen(output_json,"w")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", output_json);
			exit(0);
		}


//write the table start info into html
fprintf(out_html,"=============================================================\n"); 
//fprintf(out_html,"<H2>Tile %s, reference set: %s </H2><br>\n",argv[1], argv[4]); 
fprintf(out_html,"<table border=\"1\" cellpadding=\"2\" cellspacing=\"2\" width=\"100\%\">\n<tbody>\n");

//SETUP CvMATRIX for the absolute confusion values (PLUS FN and FP values)!, load and save methods for writing and reading (writing here, reading in an extra function which collectes all the matrices)
//need now two more cols as totallabels, because we store the FN but also missing points from validation
CvMat* confMat = cvCreateMat( totallabels+1, totallabels+2,CV_64F); //LAST COL contains FN per class, last ROW FP per class!!! saved here as error check later (because we can also compute it, of course)

printf("confusion matrix...\n");
double total_sum=0, diag_sum=0; //for accuracy computation
printf("confusion matrix absolute:\n");
printf("\t prediction \n");

fprintf(output,"confusion matrix absolute:\n");
fprintf(output,"\t prediction \n");

printf("reference ==>\t"); for (uint r=0;r<totallabels;r++) class_is_available[r]?printf(" %s ", label_name[r].c_str()):printf("   --    "); printf (" missing "); printf(" FN "); printf("\n");
fprintf(output,"reference ==>\t"); for (uint r=0;r<totallabels;r++) class_is_available[r]?fprintf(output," %s ", label_name[r].c_str()):fprintf(output,"    --   "); fprintf(output," missing    FN\n");


for (uint r=0;r<totallabels;r++){

	//printf("\t %s ",label_name[r].c_str());
	for (uint p=0;p<totallabels;p++)
		{
		 printf("\t    %d",confusion_abs[r][p]);
		fprintf(output,"\t   %d",confusion_abs[r][p]);
		cvmSet(confMat,r,p,confusion_abs[r][p]);

                 //normalize per row (i.e. use the class count from the reference)
		if (class_count_v[r]>0) confusion_rel[r][p]=(double) (confusion_abs[r][p])/((double) class_count_v[r]);
		//else {class_is_not_in_ref[r]=1;}//if class_count==0: this class is not in the reference, so in the statistics it should not be used for completness, F1 and overall, and in the confusion matrix (corresponding row) we add a - to indicate that this is not included. In the corresponding col, however, there still can be entries because it might be that some other classes are labeled as this one
		total_sum+=confusion_abs[r][p];

		if (r==p) 
			{diag_sum+=confusion_abs[r][p];
			TP_per_class[r]+=confusion_abs[r][p];
			}
		else if (r!=p) 
			{FN_per_class[r]+=confusion_abs[r][p];
			 FP_per_class[p]+=confusion_abs[r][p];
			}
		}
		//add to the last col the FN coming from the missing points in the validation
		FN_per_class[r]+=confusion_abs[r][totallabels];
		//print that value (missing)
		printf("\t %d",confusion_abs[r][totallabels]);
		printf("\t %d",FN_per_class[r]);
         	
		fprintf(output,"\t %d",confusion_abs[r][totallabels]);
		fprintf(output,"\t %d",FN_per_class[r]);
		
		printf("\n");
		fprintf(output,"\n");
		cvmSet(confMat,r,totallabels,confusion_abs[r][totallabels]); //Attention in case the matrix is used later!
		cvmSet(confMat,r,totallabels+1,FN_per_class[r]);

	}
printf(" FP ");
for (uint p=0;p<totallabels;p++)
		{
		 printf("\t %d",FP_per_class[p]);
		 cvmSet(confMat,totallabels,p,FP_per_class[p]); //missing values in the validation do not contribute to FP!!
		}
//double check for FN //FN_per_class[p]=class_count_v[p]-TP_per_class[p];
//output of class counts
//implementation check: sum of all class_counts must be identical to numbe4r of points in reference
int sum_class_count=0;
printf("\ncount"); for (uint r=0;r<totallabels;r++) {printf("\t %d ", class_count_v[r]); sum_class_count+=class_count_v[r];}printf("\n");
fprintf(output,"\ncount"); for (uint r=0;r<totallabels;r++) fprintf(output,"\t %d ", class_count_v[r]); fprintf(output,"\n");

if (sum_class_count != (ref.size()+missing_in_validation.size())) {printf("implmentation error: sum of all class count is not correct!\n"); exit(0);}
//exit(0);
/**/
//compute completenss etc


//vector<double> completeness_per_class,correctness_per_class,Fone_per_class;
//completeness is recall tp/(tp+fn)
//correctness is precision tp/(tp+fp)
//F-score:2*(precision*recall/(precision + recall))

for (uint p=0;p<totallabels;p++)
{
completeness_per_class[p]=(double) TP_per_class[p]/(double) (TP_per_class[p]+FN_per_class[p]);
correctness_per_class[p]=(double)TP_per_class[p]/(double) (TP_per_class[p]+FP_per_class[p]);
Fone_per_class[p]=2*completeness_per_class[p]*correctness_per_class[p]/(completeness_per_class[p]+correctness_per_class[p]);
}

//output of relative...

printf("\n\nconfusion matrix relative:\n");
printf("\t prediction \n");
printf("reference ==>\t"); for (uint r=0;r<totallabels;r++) class_is_available[r]?printf(" %s ", label_name[r].c_str()):printf("   --    "); printf("\n");

fprintf(output,"\n\nconfusion matrixrelative:\n");
fprintf(output,"\t prediction \n");
fprintf(output,"reference ==>\t"); for (uint r=0;r<totallabels;r++) class_is_available[r]?fprintf(output," %s ", label_name[r].c_str()):fprintf(output,"   --    "); fprintf(output,"\n");
fprintf(out_html,"<tr>\n");
//fprintf(out_html,"<td style=\"text-align: center; width: 15%;\">&#8595; predicted || reference &#8594;<br></td>\n"); for (uint r=0;r<totallabels;r++) class_is_available[r]?fprintf(out_html,"<td style=\"text-align: center; width: 15%;\"> %s <br> </td>\n", label_name[r].c_str()):fprintf(out_html,"<td style=\"text-align: center; width: 15%;\"> <del>%s</del> <br> </td>\n",label_name[r].c_str());fprintf(out_html,"<td style=\"text-align: center; width: 15%;\"> missing points <br> </td></tr>\n");

fprintf(out_html,"<td style=\"text-align: center; \">&#8595; predicted || reference &#8594;<br></td>\n"); for (uint r=0;r<totallabels;r++) class_is_available[r]?fprintf(out_html,"<td style=\"text-align: center; \"> %s <br> </td>\n", label_name[r].c_str()):fprintf(out_html,"<td style=\"text-align: center; \"> <del>%s</del> <br> </td>\n",label_name[r].c_str());fprintf(out_html,"<td style=\"text-align: center; \"> missing points <br> </td></tr>\n");



for (uint r=0;r<totallabels;r++)
	{//printf("\t %s ",label_name[r].c_str());
	/*if (class_is_available[r]) */fprintf(out_html,"<td style=\"text-align: center;\"> <i>%s</i> <br> </td>\n",label_name[r].c_str()); 
	//else fprintf(out_html,"<td style=\"text-align: center; width: 15%;\"> <del>%s</del> <br> </td>\n",label_name[r].c_str()); 
	
	for (uint p=0;p<totallabels;p++)
		{
			if (class_is_available[p])
			{
			 printf("\t\t %.3f",confusion_rel[r][p]);
			 fprintf(output,"\t\t %.3f",confusion_rel[r][p]);
			 if (r==p) fprintf(out_html,"<td style=\"text-align: center; \"> <b> %.3f </b><br> </td>\n", confusion_rel[r][p]); 
			 else fprintf(out_html,"<td style=\"text-align: center; \"> %.3f <br> </td>\n", confusion_rel[r][p]);
			}
			else
			{ //DEBUG CHECK
			  if (confusion_rel[r][p]!=0) {printf("ERROR: class r not classified, but the respective col in the confusion matrix contains information"); exit(0);} 
			  printf("\t\t ---");
			  fprintf(output,"\t\t --- ");
			  fprintf(out_html,"<td style=\"text-align: center; \"> --- <br> </td>\n"); 
			}
				
		}
        	printf("\n");
		fprintf(output,"\n");
		
		//ADD the number of missing points to the html output (only there)
		fprintf(out_html,"<td style=\"text-align: center; \"> %d <br> </td>\n", confusion_abs[r][totallabels]);
		fprintf(out_html,"</tr>\n");
	}

printf("Precision/Corr");
 for (uint p=0;p<totallabels;p++) class_is_available[p]?printf("\t\t %.3f",correctness_per_class[p]):printf("\t\t--"); printf("\n");
printf("Recall/Comp");
 for (uint p=0;p<totallabels;p++) class_is_available[p]?printf("\t\t %.3f",completeness_per_class[p]):printf("\t\t--"); printf("\n");
printf("F 1");
 for (uint p=0;p<totallabels;p++) class_is_available[p]?printf("\t\t %.3f",Fone_per_class[p]):printf("\t\t--"); printf("\n");
//the same for the json snipp
printf("JSON file snippet: ATTENTION! The relation object type -- id is hard coded here! So if the labelinfo for the labelchallenge changes, this needs to be changed here as well!!!\n");

class_is_available[2]?fprintf(F_J," \"F1_imp_surf\":%.1f,\n",Fone_per_class[2]*100):fprintf(F_J," \"F1_imp_surf\":\"--\",\n");
class_is_available[5]?fprintf(F_J," \"F1_build\":%.1f,\n",Fone_per_class[5]*100):fprintf(F_J," \"F1_build\":\"--\",\n");
class_is_available[1]?fprintf(F_J," \"F1_low_veg\":%.1f,\n",Fone_per_class[1]*100):fprintf(F_J," \"F1_low_veg\":\"--\",\n");
class_is_available[8]?fprintf(F_J," \"F1_tree\":%.1f,\n",Fone_per_class[8]*100):fprintf(F_J," \"F1_tree\":\"--\",\n");
class_is_available[3]?fprintf(F_J," \"F1_car\":%.1f,\n",Fone_per_class[3]*100):fprintf(F_J," \"F1_car\":\"--\",\n");
fprintf(F_J," \"overall\":%.1f,\n",diag_sum/total_sum*100);

fprintf(out_html,"<td style=\"text-align: center; \"><i>Precision/Correctness</i><br></td>\n"); for (uint p=0;p<totallabels;p++) class_is_available[p]?fprintf(out_html,"<td style=\"text-align: center; \"> <i> %.3f </i> <br> </td>\n", correctness_per_class[p]):fprintf(out_html,"<td style=\"text-align: center; \"> -- <br> </td>\n"); fprintf(out_html,"</tr>\n");
fprintf(out_html,"<td style=\"text-align: center; \"><i>Recall/Completeness</i><br></td>\n"); for (uint p=0;p<totallabels;p++) class_is_available[p]?fprintf(out_html,"<td style=\"text-align: center; \"> <i> %.3f </i> <br> </td>\n", completeness_per_class[p]):fprintf(out_html,"<td style=\"text-align: center; \"> -- <br> </td>\n"); fprintf(out_html,"</tr>\n");
fprintf(out_html,"<td style=\"text-align: center; \"><i>F1</i><br></td>\n"); for (uint p=0;p<totallabels;p++) class_is_available[p]?fprintf(out_html,"<td style=\"text-align: center; \"> <i> %.3f </i> <br> </td>\n", Fone_per_class[p]):fprintf(out_html,"<td style=\"text-align: center; \"> -- <br> </td>\n"); fprintf(out_html,"</tr>\n");

printf("diag_sum=%.0f, total_sum=%.0f\n",diag_sum,total_sum);

printf("Overall accuracy: %.3f\n", diag_sum/total_sum);
fprintf(output,"Overall accuracy: %.3f\n", diag_sum/total_sum);

fprintf(out_html,"</tbody>\n</table>\n");

fprintf(out_html,"<br><i><b>Overall accuracy %.3f</b></i><br>\n",diag_sum/total_sum);

fprintf(out_html,"<div align=\"center\"><br>");
  //fprintf(out_html,"<h3>Red/green image, indicating wrongly classified pixels</h3><img title=\"red/green image\" src=\"%s\"><br><h3>Original, resized true ortho image for better comparison</h3><p><img title=\"original, resized true ortho image\" src=\"%s\"><br><h3>Classified image</h3><p><img title=\"classified image\"src=\"%s\"><br></p>",output_redgreen_res,input_top_resized,output_cls_res);

fprintf(out_html,"</div>");

/**/
fclose(output);
fclose(out_html);
fclose(F_J);

 //CvFileStorage* fs = cvOpenFileStorage(output_confusion_cvmat, 0, CV_STORAGE_WRITE );
//cvWrite( fs, "confusion_matrix", confMat );
 //cvReleaseFileStorage( &fs );
cvSave(output_confusion_cvmat,confMat);
//cvReleaseImage(&redgreen);
  

}

bool same_color(LaserPoint & p1, LaserPoint & p2)
{
 if ((p1.Red()==p2.Red()) &&(p1.Green()==p2.Green()) && (p1.Blue()==p2.Blue())) return 1;
 else return 0;
}

int label_at_Point(LaserPoint & p1, ::map<uint, int> & label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B)
{
      
		//iterate over the assigned label colors. If one fits: take that one
		for (::map<uint, int>::iterator it_R=label_R.begin();it_R!=label_R.end();it_R++)
		{	uint labelid=it_R->first;
			if (label_R[labelid]==p1.Red() && label_G[labelid]==p1.Green() && label_B[labelid]==p1.Blue()) {
				return labelid;
				//printf("FOUND LABEL with ID %d at row=%d col=%d\n",labelid,(int) pos.GetX(),(int) pos.GetY());
				}
		}

//else return void label (-1)
return -1;
}

LaserPoint getlp_according_to_position(LaserPoints &lpset,LaserPoint &p, uint & last_pos, bool & ok)
{
  ok=1;
  for (uint i=last_pos;i<lpset.size();i++)
  if((lpset[i].GetX()==p.GetX()) && (lpset[i].GetY()==p.GetY()) && (lpset[i].GetZ()==p.GetZ())) {last_pos=i;return lpset[i];}
  
  for (uint i=0; i< last_pos; i++)
  if((lpset[i].GetX()==p.GetX()) && (lpset[i].GetY()==p.GetY()) && (lpset[i].GetZ()==p.GetZ()))  {last_pos=i;return lpset[i];}
  
  //last_pos=0;
  
  //printf("attention: nothing found!!\n");
  ok=0;
  return LaserPoint(0,0,0);
  
}

LaserPoint getlp_according_to_position_finder(LaserPoints &lpset,LaserPoint &p, KNNFinder<LaserPoint> & finder_ref, bool & ok)
{
  ok=1;
  
  Vector3D v(p);
  int index = finder_ref.FindIndex(v,1);
  double distanceP=(lpset[index]-v).Length();
  
  if (distanceP<0.05) return lpset[index]; 
  
  
  //printf("attention: nothing found!!\n"); 
 
  
  ok=0;
  return LaserPoint(0,0,0);
  
}

void check_color_for_classes(LaserPoints & lp, ::map<uint, int> & label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B)
{
//to make this simple we assume that each point has a color corresponding to one of the classes. Hence: if already at lp[0] we have a color which is not valid/known, we assume that the entire file is not colored. Then we assume that the reflectance contains the class info. No further check is done on the correctness of the reflectance tag then!

if (label_at_Point(lp[0],label_R, label_G, label_B)!=-1) {printf("The laserfile already contains valid colors, i.e. colors which encode the class\n");return;}

//else we just set the color according to the refl. value
printf("ATTENTION: color not valid, set to the class color according to label_info and the reflectance tag as class id\n");
for (int i=0;i<lp.size();i++) {int refl=lp[i].Reflectance();lp[i].SetColour(label_R[refl],label_G[refl],label_B[refl]);}
//for (int i=0;i<lp.size();i++) {int refl=lp[i].Reflectance();if (refl==2) continue; lp[i].SetColour(label_R[refl],label_G[refl],label_B[refl]);}
}

void fill_vector_available_classes(LaserPoints & lp, ::map<uint, int> & label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B, vector<bool> &class_is_available)
{
  for (int i=0;i<lp.size();i++)
  {
    int lab_=label_at_Point(lp[i],label_R, label_G, label_B); if (lab_>=class_is_available.size()) {printf("Error/BUG, label id is too large for vector<bool> class_is_available"); exit(0);}
    if (lab_!=-1) class_is_available[lab_]=1;
  }
  
  
}
void PrintUsage()
{
printf("3D_supervised_classification_assessment: Takes output from 3D_supervised_classification and produces a red-green point cloud\n");
  printf("Usage: \t3D_supervised_classification_assessment -reference <laserfile> -validation (i.e. classification of points not used for training, but which are present in the reference) <laserfile> -label_info <textfile>: labelid R G B labelname\n");
  printf("The validation file already does not contain the training set, therefore exclusion files not necessary!!!\n");
} 

int main(int argc, char *argv[])
{

LaserPoints referenceLS, validationLS, outputLS;//outputLS is the overall output, regardless of the class

uint count_correct=0;
uint count_incorrect=0;

FILE *labelinfo;
::map<uint, string> label_name; //maps the label id to the labelname
::map<uint, int> label_R; //maps the label id to the coding (R channel)
::map<uint, int> label_G; //maps the label id to the coding (G channel)
::map<uint, int> label_B; //maps the label id to the coding (B channel) 

vector<uint> count_correct_per_class;
vector<uint> count_incorrect_per_class;

vector <bool> class_is_available; //for each class which is available in the validation set, the switch is 1, else 0

count_correct_per_class.resize(0);
count_incorrect_per_class.resize(0);
vector<LaserPoints> outputLS_per_class; outputLS_per_class.resize(0);

//for detailed confusion matrix: reference and extraction per voxel
vector<int> ref, extr; ref.resize(0); extr.resize(0);

//new May2015: listing for each ref point for which we DO NOT have a partner in the validation set the actual class
vector<int> missing_in_validation; missing_in_validation.resize(0);


//new March 2012: if the validataion set is segmented, we also do a per-segment evaluation. However, it is no double checked whether within a segment the label changes! 
bool segments_available=0;
vector<int> ref_seg, extr_seg; ref_seg.resize(0); extr_seg.resize(0);
vector<uint> seen_segments; seen_segments.resize(0);

printf("3D_supervised_classification_assessment\n");

 InlineArguments *args = new InlineArguments(argc, argv);
 
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-reference") || !args->Contains("-validation") || !args->Contains("-label_info"))
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 

//read the label_info file
printf("reading the label_info file\n");
  if ((labelinfo=fopen(args->String("-label_info",""),"r")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", args->String("-label_info",""));
			return(0);
		}

 while(!feof(labelinfo))
		 {
			int labelid;
			char labelname[500];
			int R,G,B;
			
			fscanf(labelinfo,"%d %d %d %d %s",&labelid, &R, &G, &B, labelname);	
			if (labelid==-99) continue;	
			if (feof(labelinfo)) break;
			printf("%d %d %d %d %s\n",labelid, R, G, B, labelname );

			label_name[(uint)labelid]=labelname;
			label_R[(uint)labelid]=R;
			label_G[(uint)labelid]=G;
			label_B[(uint)labelid]=B;
			
		 }
fclose(labelinfo);

//read the reference file
printf("\n Reading reference point cloud ...\n");
referenceLS.ReInitialise();
	  if (!referenceLS.Read(args->String("-reference"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-reference"));
		  exit(0);
		}

//read the validation
printf("\n Reading classification result from %s\n",args->String("-validation"));
validationLS.ReInitialise();
	  if (!validationLS.Read(args->String("-validation"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-validation"));
		  exit(0);
		} 

//new May 2015: check if we have really the color in the input laser files. If we have instead just reflectance (as label id) we need to add the RGB color accordingly

check_color_for_classes(referenceLS,label_R, label_G, label_B);  referenceLS.WriteLAS("/tmp/refcol.laz",1); //referenceLS.Write("/tmp/refcol.laser",1);
check_color_for_classes(validationLS,label_R, label_G, label_B); validationLS.WriteLAS("/tmp/valcol.laz",1); //validationLS.Write("/tmp/valcol.laser",1);

char output_valcol[512];
strcpy(output_valcol,args->String("-validation"));
strcat(output_valcol,"_LAZ.laz");
validationLS.WriteLAS(output_valcol,1);


//check which classes are actually present in the validation file. All others will not be considered later
class_is_available.resize(label_B.size(),0);
fill_vector_available_classes(validationLS,label_R, label_G, label_B,class_is_available);

for (int i=0;i<class_is_available.size();i++)
  if (!class_is_available[i]) printf("Attention, class %s is not available in the validation file ... it will be omitted in the confusion matrix, but false classifications at the actual instances will count, of course\n ",label_name[i].c_str());
  
	    
//Set up the knnfinder for the reference
KNNFinder<LaserPoint> finder_ref(referenceLS);

		
//March 2012: check if we have segments
vector< int >  list_of_segments= validationLS.AttributeValues(SegmentNumberTag);
if (list_of_segments.size()>1) 
{segments_available=1;
 printf("Segmented validation file! also doing confusion matrix per segment!!!\n");
}

else printf("NON-segmented validation file! Not doing confusion matrix per segment!!!\n");


//Reserve memory for output per classes
uint num_classes=label_B.size();
printf("%u classes identified\n",num_classes);

count_correct_per_class.resize(num_classes,0);
count_incorrect_per_class.resize(num_classes,0);
outputLS_per_class.resize(num_classes);

//initialze the output with the validation (extracted) geometry, but color 0 0 0
printf("...Initializing output\n");
outputLS.ReInitialise();
outputLS.Read(args->String("-validation"),1);

for (uint i=0;i<outputLS.size();i++)
  outputLS[i].SetColour(0,0,0);

//the same for the individual classes
for (uint c=0; c<num_classes;c++)
{
  outputLS_per_class[c].ReInitialise();
 // outputLS_per_class[c].Read(args->String("-validation"),1);
  
 // for (uint i=0;i<outputLS_per_class[c].size();i++)
  // outputLS_per_class[c][i].SetColour(0,0,0);
}

printf("End of initialisation\n");

uint last_idx=0;
for (int i=0;i<validationLS.size();i++)
{
//get the reference point from pos. 
 bool ok;
 //LaserPoint thisref=getlp_according_to_position(referenceLS,validationLS[i],last_idx,ok);
 
 LaserPoint thisref=getlp_according_to_position_finder(referenceLS,validationLS[i],finder_ref,ok);

 if (!ok)
 {printf("attention: nothing found!! ==> error in the data, modified geomtry of validation set??\n"); 
  exit(0); //for the benchmark dataset (3D labeling) for each given validation point we need to find a ref point, else the particpants modified the geometry, in that case the program needs to stop and further checks are necessary 
  }
  
 int label_id=label_at_Point(thisref, label_R, label_G, label_B);
 
   uint segment_id;
 if (segments_available) segment_id=validationLS[i].SegmentNumber();
 
  //...if the color in ref and extr is the same: set green, else set red
  if (same_color(validationLS[i],thisref)) 
  {
    //for the general file
    outputLS[i].SetColour(0,255,0);
    count_correct++;
    
    
    //for the class specific files
    LaserPoint tmpP(outputLS[i]);
    tmpP.SetColour(0,255,0);
    outputLS_per_class[label_id].push_back(tmpP);
    count_correct_per_class[label_id]++;
    
  }
  
    else 
    {
      outputLS[i].SetColour(255,0,0);
      count_incorrect++;
      
      //for the class specific files
      LaserPoint tmpP(outputLS[i]);
      tmpP.SetColour(255,0,0);
      outputLS_per_class[label_id].push_back(tmpP);
      count_incorrect_per_class[label_id]++;
    }
    
    //for confusion matrix per point (voxel)
   int extr_id=label_at_Point(validationLS[i], label_R, label_G, label_B);
   ref.push_back(label_id);
   extr.push_back(extr_id);
   
   //if the validatoin set was segmented: add the per-segment info here, but only if the segment id was not used so far
   if ( segments_available && !check_value_in_vec(seen_segments,segment_id) ) 
   {
   ref_seg.push_back(label_id);
   extr_seg.push_back(extr_id);
    seen_segments.push_back(segment_id);
   }
}

//added MAy 2015 for the 3D labeling challenge.
//so far, the extr and ref vectors contain assignements for all validation (extr) points given. If the geometry was changed, an error was raised and the program quit.
//however, before we go to the confusion matrix, we need to ensure that also the reference points, for which we DO NOT HAVE a validation point (eg because peoole removed them or d did not consider a certain class) we have a different vector. It is very important, later called "point_missed" or so. To this end we compute a knn tree for the validation data and search for each ref point for a closest one (2cm or so), if not there the respective class label of the validation set is added to a vector.
//Check will be: sum of size of extr/ref-vector from above and this vector here must be the same as the given reference. (each point from the ref either has a partner from the validation set, OR was not contained there, and thus is listed in the new vectgor)
//Set up the knnfinder for the validation

KNNFinder<LaserPoint> finder_val(validationLS);
printf("...Initializing laserpoints instance for missed reference points\n");
LaserPoints missing_in_validationLS;
missing_in_validationLS.ReInitialise();

for (int i=0;i<referenceLS.size();i++)
{
  bool ok;
  LaserPoint thisval=getlp_according_to_position_finder(validationLS,referenceLS[i],finder_val,ok);
  //if it is find go on, only do action if there is no partner
  if (!ok)
  {
    //no partner found for this reference point! Add the label of the reference point to the vector
    int label_id=label_at_Point(referenceLS[i], label_R, label_G, label_B);
    missing_in_validation.push_back(label_id);
    missing_in_validationLS.push_back(referenceLS[i]);
  }
}


printf("%i points from the reference are not available in the validation set!\n",(uint) missing_in_validation.size());
//if (extr.size()+missing_in_validation.size()!=referenceLS.size()) {printf("Attention, error in implmentation: reference points with and without a partner from the validation set are not the same number as in the original refrence list\n"); exit(0);}

///TBD: check those measures for the 3D labeling (not_classified etc)
double accuracy=(double) (100*count_correct)/(double) validationLS.size();
printf("Simple statistics for double check with output from 3D sup. class. (Attention: here per voxel/point, not per segment):\n Anyhow if data is missing, this statistics cannot match with the later one where missing data is considered in a different way (FN contribution)");
printf("all: correct (accuracy): %.1lf %%, incorrect: %.1lf%%. Abs: %u and %u\n",accuracy,(double) (100*count_incorrect)/(double)validationLS.size(), count_correct, count_incorrect );
printf("per class:\n");
for (uint c=0; c<num_classes;c++)
{
  double sum_class=(double) (count_correct_per_class[c]+count_incorrect_per_class[c]);
 printf("class: %s,  correct (accuracy): %.1lf %%, incorrect: %.1lf%%. Abs: %u and %u\n",label_name[c].c_str(),(double) (100*count_correct_per_class[c])/sum_class,(double) (100*count_incorrect_per_class[c])/sum_class, count_correct_per_class[c], count_incorrect_per_class[c]);
}

//write output
char output[512];
//strcpy(output,args->String("-validation"));
//strcat(output,"_assessedREDGREEN.laser");
//outputLS.Write(output,0);

strcpy(output,args->String("-validation"));
strcat(output,"_assessedREDGREEN.laz");
outputLS.WriteLAS(output,0);

for (uint c=0; c<num_classes;c++)
{
//sprintf(output,"%s_assessedREDGREEN_%s.laser",args->String("-validation"),label_name[c].c_str());
//printf("outputname: %s\n",output);

//outputLS_per_class[c].Write(output,0);

sprintf(output,"%s_assessedREDGREEN_%s.laz",args->String("-validation"),label_name[c].c_str());
printf("outputname: %s\n",output);

if (outputLS_per_class.size()) outputLS_per_class[c].WriteLAS(output,0);

}

if (missing_in_validationLS.size()>0)
{
//strcpy(output,args->String("-validation"));
//strcat(output,"_missing_classified_points.laser");
//missing_in_validationLS.Write(output,0);

strcpy(output,args->String("-validation"));
strcat(output,"_missing_classified_points.laz");
missing_in_validationLS.WriteLAS(output,0);
}

strcpy(output,args->String("-validation"));
//correct names will be defined in the confusion matrix function

double accuracy_double_check;
confusion_matrix(ref, extr, label_name, output, class_is_available, missing_in_validation,  &accuracy_double_check);

//printf("accuracy from confusion (must be the same as above:%.2f ",accuracy_double_check);
//if (fabs(accuracy/100-accuracy_double_check)>0.1) printf("!!!ERROR Attention: Error in implementation: accuracies not identical\n");
//else printf("==> OK\n");

/*
//TODO: do for segments only
if (segments_available)
{
  printf("confusion per S E G M E N T:\n");
  strcpy(output,args->String("-validation"));
  strcat(output,"_confusion_per_segment.txt");
  confusion_matrix(ref_seg, extr_seg, label_name, output, NULL);
    
}
*/
  return EXIT_SUCCESS;
}
