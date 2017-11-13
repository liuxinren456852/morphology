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


using namespace std;

void morphological_indoor (char* laserFile, double resolution, char *method){

    method = (char *) "door_top_detection"; //"emptyspace" ; //"door_top_detection"; // "Erosion"

    //LaserVoxel vox(LaserPoints, double);
    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    char *root = (char*) "E:/BR_data/Diemen/process/out/door_detection/";
    strcpy (str_root,root); // initialize the str_root with root string

    /// read laser points
    LaserPoints laserpoints;
    //laserFile   = (char *) "D://test//morph//data//ifp_segmented.laser";  //morph_3rdFloor_small  //morph_small_83k_noFloorCeil
    //laserFile = (char*) "D://test//indoor_reconstruction//data//space_partitioning_test.laser";  // walls2nditer_floor_clutter_openings
    //laserFile = (char*) "D://test//morph//result//walls2nditer_nofloor_clutter_openings.laser"; //3rdfloor_modifiedpoints_nofloor_ceiling.laser";
    //aserFile = (char *) "E://BR_data//ZebR//doors_test.laser";  //door_2
    laserpoints.Read(laserFile);
    printf (" input point size: %d \n ", laserpoints.size());

    Vector3D floor_mean, ceiling_mean;
    double ceiling_z, floor_z;
    ceiling_z =  1.70;  //1.35;   //1.95; // -1.62;
    floor_z   =  -1.75; //-0.85;  //-0.73; // -4.29; // hardcoded

    ///initialize the LaserVoxel
    double vox_l=0.10;
    LaserVoxel vox(laserpoints, vox_l);
    //vox.statistics();

    LaserPoints vox_centers, vox_centers_occupied;
    char * vox_centers_path;
    strcpy (str_root,root);
    //vox_centers_path = (char *) "D://test//morph//result//vox_centers.laser";
    //vox_centers_path = (char *) "E://BR_data//ZebR//out//doors_space//vox_centers.laser";
    vox_centers_path = strcat(str_root, "vox_centers.laser");
    vector< vector < vector < int > > > vec_ijk;
    /// generating vox_centers in the given path and generating a vector to relate voxel ijk and centers
    vec_ijk = vox.export_vox_centres(2, vox_centers_path);
    vox_centers.Read(vox_centers_path);
    /// generating occupied voxels
    vox_centers_occupied = vox_centers.SelectTagValue(LabelTag, 1);
    //vox_centers_occupied.Write("D://test//morph//result//vox_centers_occupied.laser", false);
    //vox_centers_occupied.Write("E://BR_data//ZebR//out//doors_space//vox_centers_occupied.laser", false);
    strcpy (str_root,root);
    vox_centers_occupied.Write(strcat(str_root, "vox_centers_occupied.laser"), false);


    /// read trajectory
    LaserPoints     traj_laserpoints, traj_vox_centers, traj_voxcent_occupied;
    char* traj_laserFile;
    strcpy (str_root,root);
    //traj_laserFile  = (char *) "D://test//morph//data//trajectory3.laser";  //trajectory3_crop.laser  //trajectory_17k.laser
    //traj_laserFile = (char*) "E://BR_data//ZebR//traj_basement.laser";  //traj_door2 //
    //traj_laserFile = strcat(str_root, "traj.laser");
    traj_laserFile = (char *) "E:/BR_data/Diemen/process/traj_diemen1_s_rec.laser";
    traj_laserpoints.Read(traj_laserFile);
    printf ("traj input size: %d \n ", traj_laserpoints.size());

    /// move traj height to middle of the door position
    LaserPoints new_traj;
    double average_traj_height;
    DataBoundsLaser traj_db = traj_laserpoints.DeriveDataBounds(0);
    average_traj_height = (traj_db.Minimum().GetZ() + traj_db.Maximum().GetZ()) /2;
    cout << "average_traj_height: " << average_traj_height << endl;
    for(auto p : traj_laserpoints){
        LaserPoint new_p;
        double new_z;
        new_z = p.GetZ() - 0.70; //abs(average_traj_height -1.0);
        new_p = p;
        new_p.Z() = new_z;
        new_traj.push_back(new_p);
        //printf("time: %lf \n", p.DoubleAttribute(TimeTag));
    }
    strcpy (str_root,root);
    new_traj.Write(strcat(str_root,"traj_Zoffset.laser"), false);

    /// door detection by top
    if (method == "door_top_detection"){
        LaserPoints top_doors_all;
        top_doors_all = vox.door_detection(vox_centers, new_traj, 2, 5 * vox_l, 5 * vox_l, 19 * vox_l, vec_ijk); //7, 7, 23, vox_l=0.12 // 5, 5, 17, l=10cm // 5, 5, 19, l=10, d=10, #top=13
        printf("\n");
        strcpy (str_root,root);
        //top_doors_all.Write("D://test//morph//result//top_doors_all.laser", false);
        top_doors_all.Write(strcat(str_root,"top_door_all.laser"), false);
    }

    /// find edge_change voxels
    if (method == "find_edge_change"){
        LaserPoints edge_voxels = vox.edge_change();
        edge_voxels.Write("D://test//morph//result//edge_voxels.laser", false);
    }

    /*
     * Detecting empty space using  user defined window_size (odd numbers, e.g. 3, 5,...) empty neighboorhood
     * input: laserpoints including clutter and walls, openings, but no floor and ceiling
     * input: floor and ceiling z also is not needed always
     * input: window_size: is important for searching occupied neighbourhood
     * */

    if (method == "emptyspace"){
        LaserPoints empty_space;
        empty_space = vox.empty_space(2,2, floor_z, ceiling_z, 5);
        strcpy (str_root,root);
        //empty_space.Write("D://test//morph//result//empty_space.laser", false);
        //empty_space.Write("E:/BR_data/Diemen/process/out/empty_space/empty_space.laser", false);
        empty_space.Write(strcat(str_root, "empty_space.laser"), false);
    }


    /// this is the output of emptyspace program and is needed for erosion program
    LaserPoints empty_space_all;
    char* emptyspace_file;
    strcpy (str_root,root);
    //emptyspace_file  = (char *) "D://test//morph//result//all_voxels_relabeled.laser";
    emptyspace_file = strcat(str_root, "all_voxels_relabeled.laser");
    //empty_space_all.Read(emptyspace_file);

    /// erosion
    /*
     * in case of usage with empty_space result, we need all space vox centers (including empty and occupied) as input
     * */
    if (method == "Erosion"){
        printf("...performing EROSION... \n");
        LaserPoints vox_cents_erose, vox_centers_erose_occupied;
        vox_cents_erose = vox.erosion(empty_space_all, 2, 3*vox_l, 3*vox_l, 3*vox_l, vec_ijk);
        cout << endl;
        strcpy (str_root,root);
        vox_cents_erose.Write(strcat(str_root, "vox_centers_erosion.laser"), false);
        //vox_cents_erose.Write("D://test//morph//result//vox_centers_erosion.laser", false);
        vox_centers_erose_occupied = vox_cents_erose.SelectTagValue(LabelTag, 0);  // 0 for empty sapce, 1 for other
        cout << endl;
        strcpy (str_root,root);
        vox_centers_erose_occupied.Write(strcat(str_root, "vox_centers_erose_occupied.laser"), false);
        //vox_centers_erose_occupied.Write("D://test//morph//result//vox_centers_erose_occupied.laser", false);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"total processing time: "<< duration/60 << "m" << '\n';
}


