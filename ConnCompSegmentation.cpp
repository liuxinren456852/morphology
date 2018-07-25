//
// Created by NikoohematS on 28-6-2018.
//

#include <TINEdges.h>
#include "ConnCompSegmentation.h"
#include "LaserPoints.h"
#include "SegmentationParameters.h"

LaserPoints Connected_Component_Segmentation(LaserPoints &laser_points,SegmentationParameters *segmentation_parameters)
{

    // TEMP: Store highest segment number in num_planar_segments
/*    num_planar_segments = 0;
    if (!segmentation_parameters->EraseOldLabels()) {
        int min_segm_num;
        if (!laser_points.AttributeRange(SegmentNumberTag, min_segm_num, num_planar_segments))
            num_planar_segments = 0;
    }*/
    // END TEMP

    // Derive the edges that define the neighbour relations
    TINEdges     *edges;
    edges = laser_points.DeriveEdges(*segmentation_parameters);

    // Remove long edges
    if (segmentation_parameters->MaxDistanceInComponent() > 0.0)
        laser_points.RemoveLongEdges(edges->TINEdgesRef(),
                                     segmentation_parameters->MaxDistanceInComponent(),
                                     segmentation_parameters->DistanceMetricDimension() == 2);

    // Label the connected components
    laser_points.LabelComponents(edges->TINEdgesRef(),
                                 segmentation_parameters->ComponentAttribute(),
                                 segmentation_parameters->EraseOldLabels());

    // Delete the edges
    laser_points.EraseNeighbourhoodEdges();

    // Remove labels of small components
    if (segmentation_parameters->MinNumberOfPointsComponent() > 1)
        laser_points.UnlabelSmallSegments(segmentation_parameters->ComponentAttribute(),
                                          segmentation_parameters->MinNumberOfPointsComponent());

    return laser_points;

}