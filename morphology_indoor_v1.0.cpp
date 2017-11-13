//
// Created by NikoohematS on 10/28/2016.
//
//
// Created by NikoohematS on 8/29/2016.
//

#include <iostream>
#include <windows.h>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <cstdlib>
#include <Matrix3.h>
#include <vector>
#include <sstream>
#include <ctime>
#include "LaserPoints.h"
#include "Laservoxel.h"
#include "morphology_indoor.h"

using namespace std;

//int LargerPointNumberqsort(const void *pt1, const void *pt2);

LaserPoints SubtractPointClouds(LaserPoints lp1,LaserPoints lp2){

    LaserPoints::iterator point1_it, point2_it;

    lp1.SortOnCoordinates();
    lp2.SortOnCoordinates();

    lp1.RemoveAttribute(IsSelectedTag);
    for (point1_it = lp1.begin(), point2_it = lp2.begin();
         point1_it != lp1.end(), point2_it != lp2.end(); point1_it++, point2_it++){

        if (point1_it-> X()  == point2_it-> X() &&
                point1_it-> Y()  == point2_it-> Y() &&
                point1_it-> Z()  == point2_it-> Z())
            point1_it -> Select();
    }

    int num_selected = lp1.RemoveTaggedPoints(1, IsSelectedTag);
    printf("%d \n removed", num_selected);
    //qsort((void*) &*lp1.begin(), lp1.size(), sizeof(lp1), LargerPointNumberqsort);
    lp1.SortOnCoordinates();
    return lp1;
}

/* THIS is old version */
void morphological_indoor (char* laserFile, double resolution, char *method){


    method = (char *) "door_top_detection";
    bool read_trajectory = 1;

    //LaserVoxel vox(LaserPoints, double);
    std::clock_t start;
    double duration;
    start = std::clock();

    /// read laser points
    LaserPoints laserpoints, vox_centers, vox_centers_occupied, morph14k, result;
    //laserFile   = (char *) "D://test//morph//data//morph_3rdFloor_830k_thinned_seg_noFloorCeil.laser";  //morph_3rdFloor_small  //morph_small_83k_noFloorCeil
    laserFile = (char *) "E://BR_data//ZebR//cc_sub2mil_basement_crop_seg10cm.laser";
    laserpoints.Read(laserFile);
    printf (" input point size: %d \n ", laserpoints.size());

    //Vector3D eigenvalues;
   // laserpoints.NormalAndEigenValues(eigenvalues);
    //laserpoints.SelectTagValue(SegmentNumberTag, 0).ErasePoints();
    //laserpoints.Write("D://test//morph//result//lp.laser", false);

    /// read segmented pointclouds, and extract floor and ceiling height
/*    LaserPoints segmented_points;
    char * laserFile_segmented;
    //laserFile_segmented = (char*) "D://test//morph//data//morph_3rdFloor_2mil_thinned_seg.laser";
    segmented_points.Read(laserFile_segmented);

    LaserPoints floor_points,ceiling_points;
    ceiling_points = segmented_points.SelectTagValue(SegmentNumberTag, 3); //LabelTag 6
    floor_points = segmented_points.SelectTagValue(SegmentNumberTag, 0); // LabelTag 5*/

    Vector3D floor_mean, ceiling_mean;
    //floor_mean = floor_points.Mean();
    //ceiling_mean = ceiling_points.Mean();

    //cout << floor_mean.Z() << endl;
    //cout << ceiling_mean.Z() << endl;


    ///initialize the LaserVoxel
    double vox_l=0.10;
    LaserVoxel vox(laserpoints, vox_l);
    //vox.statistics();

    char * vox_centers_path;
    //vox_centers_path = (char *) "D://test//morph//result//vox_centers.laser";
    vox_centers_path = "E://BR_data//ZebR//out//vox_centers.laser";
    vector< vector < vector < int > > > vec_ijk;
    vec_ijk = vox.export_vox_centres(2, vox_centers_path);
    vox_centers.Read(vox_centers_path);
    vox_centers_occupied = vox_centers.SelectTagValue(LabelTag, 1);
    //vox_centers_occupied.Write("D://test//morph//result//vox_centers_occupied.laser", false);
    vox_centers_occupied.Write("E://BR_data//ZebR//out//vox_centers_occupied.laser", false);

    //if (read_trajectory){
        /// read trajectory
        LaserPoints     traj_laserpoints, traj_vox_centers, traj_voxcent_occupied;
        char* traj_laserFile;
        //traj_laserFile  = (char *) "D://test//morph//data//trajectory3.laser";  //trajectory3_crop.laser  //trajectory_17k.laser
        traj_laserFile = (char*) "E://BR_data//ZebR//traj_basement_crop.laser";
        traj_laserpoints.Read(traj_laserFile);
        printf ("traj input size: %d \n ", traj_laserpoints.size());

/*        /// initialize the LaserVoxel for trajectory
        double traj_vox_l=0.10;
        LaserVoxel traj_vox(traj_laserpoints, traj_vox_l);

        char * traj_voxcents_path;
        traj_voxcents_path = (char *) "D://test//morph//result//traj_vox_centers.laser";
        vector< vector < vector < int > > > traj_vec_ijk;
        traj_vec_ijk = traj_vox.export_vox_centres(2, traj_voxcents_path);
        traj_vox_centers.Read(traj_voxcents_path);
        traj_voxcent_occupied = traj_vox_centers.SelectTagValue(LabelTag, 1);
        traj_voxcent_occupied.Write("D://test//morph//result//traj_vox_centers_occupied.laser", false);*/
    //}

    /// door detection by top
    if (method == "door_top_detection"){
        LaserPoints top_doors_all;
        top_doors_all = vox.door_detection(vox_centers, traj_laserpoints, 2, 9 * vox_l, 9 * vox_l, 21 * vox_l, vec_ijk);
        printf("\n");
        //top_doors_all.Write("D://test//morph//result//top_doors_all.laser", false);
        top_doors_all.Write("E://BR_data//ZebR//out//top_door_all.laser");
    }

    /// find edge_change voxels
    if (method == "find_edge_change"){
        LaserPoints edge_voxels = vox.edge_change();
        edge_voxels.Write("D://test//morph//result//edge_voxels.laser", false);
    }

    //TODO saving vox_centers_erosion per each k (or z value) for representing different layer of Z and to comapre
    /// closing
    if (method == "Close"){
        printf("...performing CLOSING... \n");
        LaserPoints vox_centers_close, vox_centers_close_occupied;
        vox_centers_close = vox.morph_close(vox_centers, 2, 5*vox_l, 5*vox_l, 11*vox_l, vec_ijk);
        vox_centers_close_occupied = vox_centers_close.SelectTagValue(LabelTag, 1);
        cout << endl;
        vox_centers_close.Write("D://test//morph//result//vox_centers_close.laser", false);
        vox_centers_close_occupied.Write("D://test//morph//result//vox_centers_close_occupied.laser", false);

        result = vox.compare_voxels_tag(vox_centers_close, vox_centers, vec_ijk);
        result.Write("D://test//morph//result//result.laser", false);
    }

    /// opening
    if (method == "Open"){
        printf("...performing OPENING... \n");
        LaserPoints vox_centers_open, vox_centers_open_occupied;
        vox_centers_open = vox.morph_open(vox_centers, 2, 3*vox_l, 9*vox_l, 21*vox_l, vec_ijk);
        vox_centers_open_occupied = vox_centers_open.SelectTagValue(LabelTag, 1);
        cout << endl;
        vox_centers_open.Write("D://test//morph//result//vox_centers_open.laser", false);
        vox_centers_open_occupied.Write("D://test//morph//result//vox_centers_open_occupied.laser", false);
    }


    /// erosion
    if (method == "Erosion"){
        LaserPoints vox_cents_erose, vox_centers_erose_occupied;
        vox_cents_erose = vox.erosion(vox_centers, 2, 9*vox_l, 9*vox_l, 9*vox_l, vec_ijk);
        cout << endl;
        vox_cents_erose.Write("D://test//morph//result//vox_centers_erosion.laser", false);
        vox_centers_erose_occupied = vox_cents_erose.SelectTagValue(LabelTag, 1);
        cout << endl;
        vox_centers_erose_occupied.Write("D://test//morph//result//vox_centers_erose_occupied.laser", false);
    }


    /// dilation
    if (method == "Dilation"){
        LaserPoints vox_cents_dilate, vox_centers_dilate_occupied;
        vox_cents_dilate = vox.dilation(vox_centers, 2, 9*vox_l, 9*vox_l, 21*vox_l, vec_ijk);
        cout << endl;
        vox_cents_dilate.Write("D://test//morph//result//vox_cents_dilate.laser", false);
        vox_centers_dilate_occupied = vox_cents_dilate.SelectTagValue(LabelTag, 1);
        cout << endl;
        vox_centers_dilate_occupied.Write("D://test//morph//result//vox_centers_dilate_occupied.laser", false);
    }

    /// door template_cylinder
    if (method == "door_cyl"){
        LaserPoints doors_points;
        doors_points = vox.structure_template_cylinder(vox_centers, traj_laserpoints, 2, 11 * vox_l, 11 * vox_l, 21 * vox_l, vec_ijk);
        doors_points.Write("D://test//morph//result//doors_points.laser", false);
    }

/*    /// closed doors
    if (method == "closed_door"){
        LaserPoints closed_doors_points;
        closed_doors_points = vox.closed_door(vox_centers, traj_laserpoints, 2, 11 * vox_l, 11 * vox_l, 21 * vox_l, vec_ijk);
        closed_doors_points.Write("D://test//morph//result//closed_doors_points.laser", false);
    }*/

    /// door template
    if (method == "door"){
        LaserPoints doors_points;
        doors_points = vox.structure_template(vox_centers, traj_laserpoints, 2, 11 * vox_l, 11 * vox_l, 21 * vox_l, vec_ijk);
        doors_points.Write("D://test//morph//result//doors_points.laser", false);
    }

    if (method == "emptyspace"){
        LaserPoints empty_space;
        empty_space = vox.empty_space(2,2, floor_mean.Z(), ceiling_mean.Z());
        empty_space.Write("D://test//morph//result//empty_space.laser", false);
    }



    //TODO apply opening on emty-space results
    // TODO applying the result of navigable space with doors_template
    // TODO applying floor/ceiling value also for opening
    // TODO saving the result in different z value and compare
    // TODO find a solution for the occlusion gaps in the data and walls

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"total processing time: "<< duration/60 << "m" << '\n';
}

/*void MorphologyIndoor::morphological_indoor(char *laserFile, double resolution, char *method) {

}*/



