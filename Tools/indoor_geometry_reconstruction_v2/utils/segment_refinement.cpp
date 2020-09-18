//
// Created by NikoohematS on 11/23/2016.
//
#include <iostream>
#include <LaserPoints.h>

LaserPoints segment_refinement(LaserPoints segmented_lp, int min_segment_size, double maxdistanceInComponent) {
    /// resegmenting laserpoints by removing longedges in the TIN and removing deformed segments
    LaserPoints relabeled_lpoints;
    int new_segment_number = 0;
    printf("Splitting strange shaped segments... & removing small (%d points) segments:\n", min_segment_size);
    if (segmented_lp.size() > min_segment_size) {
        if (segmented_lp.HasAttribute(SegmentNumberTag)) { /// check if points have segment attribute
            int dominanat_segment_nr1, count1;
            LaserPoints segment1_lpoints;
            SegmentationParameters *seg_parameter;
            seg_parameter = new SegmentationParameters;
            cout << "# laserpoints " << " # segment points " << endl;
            do {
                /// get the segment_number for the dominanat segment
                dominanat_segment_nr1 = segmented_lp.MostFrequentAttributeValue(SegmentNumberTag, count1);
                segment1_lpoints.ErasePoints();
                /// save points of the dominant segment
                segment1_lpoints.AddTaggedPoints(segmented_lp, dominanat_segment_nr1, SegmentNumberTag);

                printf("%10d %10d\r", segmented_lp.size(), segment1_lpoints.size());

                TINEdges *edges;
                edges = segment1_lpoints.DeriveEdges(*seg_parameter);
                seg_parameter->MaxDistanceInComponent() = maxdistanceInComponent;
                segment1_lpoints.RemoveLongEdges(edges->TINEdgesRef(),
                                                 seg_parameter->MaxDistanceInComponent(),
                                                 seg_parameter->DistanceMetricDimension() == 2);
                segment1_lpoints.LabelComponents(edges->TINEdgesRef(), SegmentNumberTag);

                /// do the same for 2nd dominant segment
                int dominanat_segment_nr2, count2;
                LaserPoints segment2_lpoints;

                do {
                    /// get the segment_number for the dominant segment
                    dominanat_segment_nr2 = segment1_lpoints.MostFrequentAttributeValue(SegmentNumberTag, count2);
                    if (count2 > min_segment_size) {
                        new_segment_number++;
                        segment2_lpoints.ErasePoints();
                        segment2_lpoints.AddTaggedPoints(segment1_lpoints, dominanat_segment_nr2, SegmentNumberTag);
                        segment2_lpoints.SetAttribute(SegmentNumberTag, new_segment_number);
                        relabeled_lpoints.AddPoints(segment2_lpoints);
                        segment1_lpoints.RemoveTaggedPoints(dominanat_segment_nr2, SegmentNumberTag);
                    }

                } while (count2 > min_segment_size);
                segmented_lp.RemoveTaggedPoints(dominanat_segment_nr1, SegmentNumberTag);
                dominanat_segment_nr1 = segmented_lp.MostFrequentAttributeValue(SegmentNumberTag, count1);
            } while (count1 > min_segment_size);
        }
        printf("\n");
        //relabeled_lpoints.Write("D://test//indoorgraph//data//segment_refine_relabeled.laser", false);
        return relabeled_lpoints;
    }
}


