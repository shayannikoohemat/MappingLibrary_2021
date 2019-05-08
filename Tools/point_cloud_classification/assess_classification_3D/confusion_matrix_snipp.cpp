printf("confusion matrix...\n");
double total_sum=0, diag_sum=0; //for accuracy computation
printf("confusion matrix absolute:\n");
printf("\t prediction \n");

fprintf(output,"confusion matrix absolute:\n");
fprintf(output,"\t prediction \n");

printf("reference ==>\t"); for (uint r=0;r<totallabels;r++) printf(" %s ", label_name[r].c_str()); printf(" FN "); printf("\n");
fprintf(output,"reference ==>\t"); for (uint r=0;r<totallabels;r++) fprintf(output," %s ", label_name[r].c_str()); fprintf(output,"\n");


for (uint r=0;r<totallabels;r++){

	//printf("\t %s ",label_name[r].c_str());
	for (uint p=0;p<totallabels;p++)
		{
		 printf("\t\t     %d",confusion_abs[r][p]);
		fprintf(output,"\t\t     %d",confusion_abs[r][p]);
		cvmSet(confMat,r,p,confusion_abs[r][p]);

                 //normalize per row (i.e. use the class count from the reference)
		if (class_count_v[r]>0) confusion_rel[r][p]=(double) (confusion_abs[r][p])/((double) class_count_v[r]);
		else {class_is_not_in_ref[r]=1;}//if class_count==0: this class is not in the reference, so in the statistics it should not be used for completness, F1 and overall, and in the confusion matrix (corresponding row) we add a - to indicate that this is not included. In the corresponding col, however, there still can be entries because it might be that some other classes are labeled as this one
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
		printf("\t\t %d",FN_per_class[r]);
        	printf("\n");
		fprintf(output,"\n");
		cvmSet(confMat,r,totallabels,FN_per_class[r]);

	}
printf(" FP ");
for (uint p=0;p<totallabels;p++)
		{
		 printf("\t\t %d",FP_per_class[p]);
		 cvmSet(confMat,totallabels,p,FP_per_class[p]);
		}
//double check for FN //FN_per_class[p]=class_count_v[p]-TP_per_class[p];
//output of class counts
printf("\ncount"); for (uint r=0;r<totallabels;r++) printf("\t\t %d ", class_count_v[r]); printf("\n");
fprintf(output,"\ncount"); for (uint r=0;r<totallabels;r++) fprintf(output,"\t\t %d ", class_count_v[r]); fprintf(output,"\n");

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
