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
#include "ConnCompSegmentation.h"
#include "morphology_indoor.h"
#include "indoor_reconstruction.h"


using namespace std;

enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
    DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);


void morphological_indoor (char* laserFile, double resolution, char *method){

    method = (char *) "Erosion"; //"emptyspace" ; //"door_top_detection"; // "Erosion"

    //LaserVoxel vox(LaserPoints, double);
    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    //char *root = (char*)  "E:/BR_data/Diemen/process/out/door_detection/";
    //char *root = (char*) "D:/test/morph2/out/";
    //char *root = (char*) "E:/publication_data/FB_dataset/process/out/spacepartitioning/";
    //char *root = (char*) "../output/space_partitioning/"; ///server path output
    //auto *root = (char*) "E:/publication_data/Delft_zebrevo/space_partitioning/out/";
   //char* root = (char*) "E:/publication_data/BR_backpack/out/";
   // auto * root = (char*) "E:/publication_data/Delft_zebrevo/door_detection/out/";
   // auto *root = (char*) "E:/publication_data/Navvis/Kadaster/door_detection/out/";
    //auto *root = (char*) "E:/publication_data/Haaksbergen/space_partitioning/wall_fl_cl.laser";
    auto *root = (char*) "E:/publication_data/Haaksbergen/space_partitioning/result/" ;
   // auto *root = (char*) "../output/door_detection/"; //server path
    strcpy (str_root,root); // initialize the str_root with root string

    /// read laser points
    LaserPoints laserpoints;
    //laserFile   = (char *) "D://test//morph//data//ifp_segmented.laser";  //morph_3rdFloor_small  //morph_small_83k_noFloorCeil
    //laserFile = (char*) "D://test//indoor_reconstruction//data//space_partitioning_test.laser";  // walls2nditer_floor_clutter_openings
    //laserFile = (char*) "D://test//morph//result//walls2nditer_nofloor_clutter_openings.laser"; //3rdfloor_modifiedpoints_nofloor_ceiling.laser";
    //laserFile = (char *) "E://BR_data//ZebR//doors_test.laser";  //door_2
    //laserFile = (char *) "D:/test/morph2/data/2rooms_all.laser";
    //laserFile = (char *) "E:/publication_data/indoor_changeDetection/process/backpack_intersection_final/wall_floor_ceiling.laser";
    //laserFile = (char*) "E:/publication_data/FB_dataset/data/data/1stfloor/space_partitioning/wall_floor_op_occl_ceil_noFalseOpen.laser";
    //laserFile = (char*) "E:/publication_data/FB_dataset/data/data/1stfloor/space_partitioning/true_spaces/wall_floor_ceiling_manaual.laser";
    //laserFile = (char*) "E:/publication_data/FB_dataset/data/data/1stfloor/door_detection/walls.laser";
    //laserFile = (char*) "E:/publication_data/FB_dataset/data/data/3rdfloor/door_detection/test.laser";
    //laserFile = (char*) "E:/publication_data/indoor_changeDetection/process/backpack_intersection_final/walls.laser";
    //char * input_file = (char*) "E:/publication_data/Delft_zebrevo/door_detection/walls.laser";
    //laserFile = (char*) "E:/publication_data/input/spaces_input.laser";
    //laserFile = (char*) "E:/publication_data/Delft_zebrevo/door_detection/low_ceiling_problem.laser";  //low_ceiling_problem //wall_sample
    //laserFile = (char*) "E:/publication_data/Delft_zebrevo/space_partitioning/data/space_partitioning_input.laser";
    //laserFile = (char*) "E:/publication_data/Navvis/Kadaster/space_partitioning/wall_floor_ceiling_cropZ.laser";
    //laserFile = (char*) "E:/publication_data/Navvis/Kadaster/door_detection/walls_reseg_reduced_final.laser";
    laserFile = (char*) "E:/publication_data/Haaksbergen/space_partitioning/second_floor/wall_fl_cl.laser";
    //laserFile = (char*) "../data/door_detection.laser"; // server path
    //laserFile = (char*) "../data/morphology.laser"; // server path
    laserpoints.Read(laserFile);
    printf (" input point size: %d \n ", laserpoints.size());

    Vector3D floor_mean, ceiling_mean;
    double ceiling_z, floor_z;
    //ceiling_z =  1.70;  //1.35;   //1.95; // -1.62;
    //floor_z   =  -1.75; //-0.85;  //-0.73; // -4.29; // hardcoded

    /// extract the lowest ceiling point and the highest floor point
    /* NOTE: point clouds should have label of wall=4, floor=5 and ceiling=6 */

    LaserPoints floor_lp, ceil_lp;
    floor_lp = laserpoints.SelectTagValue (LabelTag, 5);
    ceil_lp = laserpoints.SelectTagValue (LabelTag, 6);

   // double floor_Zmax, ceil_Zmin; /// with this we loose the 3D shape of the ceiling
    double floor_Zmin, ceil_Zmax;
    DataBoundsLaser floor_db = floor_lp.DeriveDataBounds (0);
    //floor_Zmax = floor_db.Maximum ().GetZ ();
    floor_Zmin = floor_db.Minimum ().GetZ ();
    printf("Floor MinZ: %.2f \n", floor_Zmin);

    DataBoundsLaser ceil_db = ceil_lp.DeriveDataBounds (0);
    //ceil_Zmin = ceil_db.Minimum ().GetZ ();
    ceil_Zmax = ceil_db.Maximum ().GetZ ();
    printf("Ceil MaxZ: %.2f \n", ceil_Zmax);

    /// hardcode /// for space partitioning
    ceil_Zmax =  15.35; //13.65; //9.00 ;//-1.17;  //12.00;  //1.68;//;
    floor_Zmin = 11.80; //8.40; //6.50; //-7.64; //-1.00; //-1.75;//;

    if(ceil_Zmax <= floor_Zmin){
        printf("Ceiling height can not be lower than floor, abort ... \n");
        exit(EXIT_FAILURE);
    }


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
    vec_ijk = vox.export_vox_centres(1, vox_centers);
    vox_centers.Write(vox_centers_path, false);
    //vox_centers.Read(vox_centers_path);

    /// generating occupied voxels
    vox_centers_occupied = vox_centers.SelectTagValue(LabelTag, 1);
    //vox_centers_occupied.Write("D://test//morph//result//vox_centers_occupied.laser", false);
    //vox_centers_occupied.Write("E://BR_data//ZebR//out//doors_space//vox_centers_occupied.laser", false);
    strcpy (str_root,root);
    vox_centers_occupied.Write(strcat(str_root, "vox_centers_occupied.laser"), false);


/***********************************************************************************************************************/

    /*  Trajectory operations for door detection */
    /// read trajectory
/*    LaserPoints     traj_laserpoints, traj_vox_centers, traj_voxcent_occupied;
    char* traj_laserFile;
    strcpy (str_root,root);
    //traj_laserFile  = (char *) "D://test//morph//data//trajectory3.laser";  //trajectory3_crop.laser  //trajectory_17k.laser
    //traj_laserFile = (char*) "E://BR_data//ZebR//traj_basement.laser";  //traj_door2 //
    //traj_laserFile = strcat(str_root, "traj.laser");
    //traj_laserFile = (char *) "E:/BR_data/Diemen/process/traj_diemen1_s_rec.laser";
    //traj_laserFile = (char*) "E:/publication_data/indoor_changeDetection/data/traj_backpack.laser";
    traj_laserFile = (char*) "E:/publication_data/Navvis/Kadaster/door_detection/level2_kadaster_traj.laser";
    traj_laserpoints.Read(traj_laserFile);
    printf ("traj input size: %d \n ", traj_laserpoints.size());

    /// transfer traj height to middle of the door position
    LaserPoints new_traj;
    //double average_traj_height;
    //DataBoundsLaser traj_db = traj_laserpoints.DeriveDataBounds(0);
   // average_traj_height = (traj_db.Minimum().GetZ() + traj_db.Maximum().GetZ()) /2;

    for(auto &p : traj_laserpoints){
        LaserPoint new_p;
        double new_z;
        new_z = p.GetZ() + 1.60;
        new_p = p;
        new_p.Z() = new_z;
        new_traj.push_back(new_p);
        //printf("time: %lf \n", p.DoubleAttribute(TimeTag));
    }
    strcpy (str_root,root);
    new_traj.Write(strcat(str_root,"traj_Zoffset.laser"), false);*/


    /*******************************************************************************************************************/

    /// door detection by top
    if (method == "door_top_detection"){
        LaserPoints top_doors_all;
        LaserPoints traj_laserpoints;
        //traj_laserpoints = new_traj;  /// for backpack and trolley
        char* traj_laserFile;
        //traj_laserFile = (char *) "E:/publication_data/FB_dataset/data/data/1stfloor/door_detection/traj_1stfloor.laser";
        //traj_laserFile = (char*) "E:/publication_data/indoor_changeDetection/data/traj_backpack.laser";
        //traj_laserFile = (char*) "E:/publication_data/Delft_zebrevo/door_detection/trajectory_1stfloor.laser";
        //traj_laserFile = (char*) "E:/publication_data/Navvis/Kadaster/door_detection/level2_kadaster_traj_Zoffset.laser";
        traj_laserFile = (char*) "E:/publication_data/Navvis/Kadaster/door_detection/level2_kadaster_traj_Zoffset.laser";
        //traj_laserFile = (char*) "../data/trajectory1.laser";
        traj_laserpoints.Read(traj_laserFile);
        /// bigger window could cause void-hood for open door won't be fulfilled
        /// tall windows could cause including the low ceiling in the result
        /// *** important NOTE: the input laserpoints should n't have irregular floor height or something below the floor, since the bbox would change
        top_doors_all = vox.door_detection(vox_centers, traj_laserpoints, root, 2, 7 * vox_l, 7 * vox_l, 19 * vox_l, vec_ijk); //7, 7, 23, vox_l=0.12 // 5, 5, 17, l=10cm // 5, 5, 19, l=10, d=10, #top=13
        //top_doors_all = vox.door_detection2(vox_centers, traj_laserpoints, 2, 0.4, 1.5, 0.30, vec_ijk);
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

    if (method == "emptyspace"){  /// expensive method
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
        SetColor (GREEN);
        printf("...performing EROSION... \n");
        SetColor (GRAY);
        LaserPoints vox_cents_erose, vox_centers_erose_unoccupied;
        LaserPoints vox_centers_erosion_lablechanged; /// extra output for changed labels
        int erosion_win = 11;//9; //13, 7;
        //vox_cents_erose = vox.erosion(empty_space_all, 2, 3*vox_l, 3*vox_l, 3*vox_l, vec_ijk);
        vox_cents_erose = vox.erosion(vox_centers, 1,
                                      erosion_win*vox_l, erosion_win*vox_l, erosion_win*vox_l, vec_ijk);
        cout << endl;
        strcpy (str_root,root);
        vox_cents_erose.Write(strcat(str_root, "vox_centers_erosion.laser"), false);
        //vox_centers_erosion_lablechanged.Write(strcat(str_root, "vox_centers_erosion_lablechanged.laser"), false);
        //vox_cents_erose.Write("D://test//morph//result//vox_centers_erosion.laser", false);
        vox_centers_erose_unoccupied = vox_cents_erose.SelectTagValue(LabelTag, 0);  // 0 for empty sapce, 1 for other
        cout << endl;
        strcpy (str_root,root);
        vox_centers_erose_unoccupied.Write(strcat(str_root, "vox_centers_erose_unoccupied.laser"), false);
        //vox_centers_erose_occupied.Write("D://test//morph//result//vox_centers_erose_occupied.laser", false);

        // first remove all existent segment numbers
        vox_cents_erose.RemoveAttribute (SegmentNumberTag);

        /// performing ConnecCompAnl. on unoccupied voxels between floor and ceiling
        LaserPoints vox_centers_erose_unoccupied_selected;
        for(auto &p : vox_centers_erose_unoccupied){
            if( p.GetZ () <= (ceil_Zmax - vox_l) && p.GetZ () >= (floor_Zmin + vox_l)){
                vox_centers_erose_unoccupied_selected.push_back (p);
            }
        }

        /// this is for splitting huge voxel points to several vertical partitions for double check
/*        LaserPoints z_crop_erose1, z_crop_erose2;
        z_crop_erose1 = select_byZ (vox_centers_erose_unoccupied, 6.60, 7.40);
        z_crop_erose2 = select_byZ (vox_centers_erose_unoccupied, 7.45, 8.80);

        cout << endl;
        strcpy (str_root,root);
        z_crop_erose1.Write(strcat(str_root, "z_crop_erose1.laser"), false);

        strcpy (str_root,root);
        z_crop_erose2.Write(strcat(str_root, "z_crop_erose2.laser"), false);*/

        /// for debug and double check
        //cout << endl;
        //strcpy (str_root,root);
        //vox_centers_erose_unoccupied_selected.Write(strcat(str_root, "vox_centers_erose_unoccupied_selected.laser"), false);

        /// ConnCompAnalyze
        SetColor (GREEN);
        printf("... Performing ConnComp Segmentation ...\n");
        SetColor (GRAY);
        SegmentationParameters *segmentationParameters;
        segmentationParameters = new SegmentationParameters;
        segmentationParameters -> MaxDistanceInComponent () = 1.0;

        LaserPoints unoccupied_ConnComp_segments;
        unoccupied_ConnComp_segments = Connected_Component_Segmentation (vox_centers_erose_unoccupied_selected, segmentationParameters);

        cout << endl;
        strcpy (str_root,root);
        unoccupied_ConnComp_segments.Write(strcat(str_root, "unoccupied_ConnComp_segments.laser"), false);

        /// transfer the segment labels to the vox_cents_erose for dilation step
        /// extract the bounds of the voxel space
        //get the databounds from the original laserpoints
        SetColor (GREEN);
        printf("... Transferring segmentNo to voxels ...\n");
        SetColor (GRAY);
        DataBoundsLaser db = laserpoints.DeriveDataBounds(0);
        double  min_X,
                min_Y,
                min_Z;

        min_X = db.Minimum().GetX();
        min_Y = db.Minimum().GetY();
        min_Z = db.Minimum().GetZ();
        for (auto &p : unoccupied_ConnComp_segments){

            uint i = floor((p.GetX () - min_X)/vox_l);
            uint j = floor((p.GetY () - min_Y)/vox_l);
            uint k = floor((p.GetZ () - min_Z)/vox_l);

            int segment_no;
            if(p.HasAttribute (SegmentNumberTag))
                segment_no = p.Attribute (SegmentNumberTag);

            int vox_inx;
            vox_inx = vec_ijk[i][j][k];
            vox_cents_erose[vox_inx].SetAttribute(SegmentNumberTag, segment_no);
        }

        // for debug and double check
        cout << endl;
        strcpy (str_root,root);
        vox_cents_erose.Write(strcat(str_root, "vox_centers_erosion_withSegNo.laser"), false);


        /// performing a dilation on erosion result
        SetColor (GREEN);
        printf ("... performing DILATION ...\n");
        SetColor (GRAY);
        int dilation_win = 9;  //9, 5;
        LaserPoints vox_cents_dilate, vox_centers_dilate_unoccupied;
        vox_cents_dilate = vox.dilation(vox_cents_erose, 1,
                dilation_win*vox_l, dilation_win*vox_l, dilation_win*vox_l, vec_ijk);
        cout << endl;
        strcpy (str_root,root);
        vox_cents_dilate.Write(strcat(str_root, "vox_cents_dilate.laser"), false);
        vox_centers_dilate_unoccupied = vox_cents_dilate.SelectTagValue(LabelTag, 0);  // 0 for empty sapce, 1 for other
        cout << endl;
        strcpy (str_root,root);
        vox_centers_dilate_unoccupied.Write(strcat(str_root, "vox_centers_dilate_unoccupied.laser"), false);

        /// offset
        //// this offset is to keep the dilated layers near the ceiling and floor
        /// important step to choose the points between the ceilign and floor for resegmentation
        double ceiling_offset_threshold, floor_offset_threshold;
        ceiling_offset_threshold = 3*vox_l; //3 /// important threshold to keep the partitions seperated
        floor_offset_threshold = 2*vox_l;   //2 /// important threshold

        double ceil_Zmax_offset, floor_Zmin_offset;
        ceil_Zmax_offset    = ceil_Zmax  - ceiling_offset_threshold;  /// offset to lower value
        floor_Zmin_offset   = floor_Zmin + floor_offset_threshold; /// offset to higher values

    /// AGAIN select points between floor and ceiling for propagation of segment numbers with majority filtering
        LaserPoints vox_centers_dilate_unoccupied_reselected;
        for(auto &p : vox_centers_dilate_unoccupied){
            if( p.GetZ() <= ceil_Zmax_offset && p.GetZ() >= floor_Zmin_offset){
            //if( p.GetZ() <= (ceil_db.Minimum ().GetZ () - 2*vox_l) && p.GetZ() >= (floor_db.Maximum ().GetZ () + 2*vox_l)){ // not correct for ceilings with different heights
                vox_centers_dilate_unoccupied_reselected.push_back (p);
            }
        }

        cout << endl;
        strcpy (str_root,root);
        vox_centers_dilate_unoccupied_reselected.Write(strcat(str_root, "vox_centers_dilate_unoccupied_reselected.laser"), false);

        /// several selection by z
        LaserPoints z_crop1, z_crop2, z_crop3;
        z_crop1 = select_byZ (vox_centers_dilate_unoccupied_reselected, 14.37, 12.37); //1.0, 5.0
        //z_crop2 = select_byZ (vox_centers_dilate_unoccupied_reselected, 7.45, 8.80); //4.93, 6.70
        //z_crop3 = select_byZ (vox_centers_dilate_unoccupied_reselected, ); //6.60, 11.60

        cout << endl;
        strcpy (str_root,root);
        z_crop1.Write(strcat(str_root, "z_crop1.laser"), false);

        strcpy (str_root,root);
        z_crop2.Write(strcat(str_root, "z_crop2.laser"), false);

        strcpy (str_root,root);
       // z_crop3.Write(strcat(str_root, "z_crop3.laser"), false);


        // Derive the edges that define the neighbour relations
        TINEdges     *edges;
        edges = vox_centers_dilate_unoccupied_reselected.DeriveEdges(*segmentationParameters);

        /// apply majority_filter
        //do{
            vox_centers_dilate_unoccupied_reselected.MajorityFilter (*segmentationParameters, *edges); /// resegmentation
        //}
        //while (!vox_centers_dilate_unoccupied_reselected.HasAttribute (SegmentNumberTag));

        cout << endl;
        strcpy (str_root,root);
        vox_centers_dilate_unoccupied_reselected.Write(strcat(str_root, "vox_centers_dilate_unoccupied_resegment.laser"), false);

        /// remove the biggest segment, since it is all the voxels around the building
        /// there is the chance of removing some of the interior spaces if they are connected
        LaserPoints reduced_lp;
        reduced_lp = vox_centers_dilate_unoccupied_reselected;
        int dominant_segment_nr, count;
        dominant_segment_nr = reduced_lp.MostFrequentAttributeValue (SegmentNumberTag, count);
        reduced_lp.RemoveTaggedPoints (dominant_segment_nr, SegmentNumberTag);

        cout << endl;
        strcpy (str_root,root);
        reduced_lp.Write(strcat(str_root, "first_result.laser"), false);

    }

    if(method == "Dilation"){
        printf("...performing DILATION... \n");
        LaserPoints vox_cents_dilate, vox_centers_dilate_unoccupied;
        vox_cents_dilate = vox.dilation (vox_centers, 1, 9*vox_l, 9*vox_l, 9*vox_l, vec_ijk);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"total processing time: "<< duration/60 << "m" << '\n';
}

/// erosion step2 is for large datasets that all steps together may crash.
/// erosion_step2 contains Conn_Comp Analysis on the result of erosion and then a "dilation" process
void Erosion_step2 (LaserPoints laserpoints, LaserPoints vox_centers_erose_unoccupied,
                    LaserPoints vox_cents_erose, double vox_l){

    std::clock_t start;
    double duration;
    start = std::clock();

    /// input files
    laserpoints.Read("E:/publication_data/Delft_zebrevo/space_partitioning/data/space_partitioning_input_wallcropped.laser");
    vox_centers_erose_unoccupied.Read("E:/publication_data/Delft_zebrevo/space_partitioning/out/vox_centers_erose_unoccupied.laser");
    vox_cents_erose.Read("E:/publication_data/Delft_zebrevo/space_partitioning/out/vox_centers_erosion.laser");


    char str_root[500];
    auto *root = (char*) "E:/publication_data/Delft_zebrevo/space_partitioning/out/";

    /// for erosion step2 again we need to generate voxels to reconstruct vec_ijk
    LaserVoxel vox(laserpoints, vox_l);
    //vox.statistics();

    LaserPoints vox_centers, vox_centers_occupied;
    char * vox_centers_path;
    strcpy (str_root,root);

    vox_centers_path = strcat(str_root, "vox_centers.laser");
    vector< vector < vector < int > > > vec_ijk;

    /// generating vox_centers in the given path and generating a vector to relate voxel ijk and centers
    vec_ijk = vox.export_vox_centres(1, vox_centers);
    vox_centers.Write(vox_centers_path, false);
    //vox_centers.Read(vox_centers_path);

    // first remove all existent segment numbers
    vox_cents_erose.RemoveAttribute (SegmentNumberTag);

    double ceil_Zmax, floor_Zmin;
    /// hardcode /// for space partitioning
    ceil_Zmax = 6.35;
    floor_Zmin = -1.00;

    /// performing ConnecCompAnl. on unoccupied voxels between floor and ceiling
    LaserPoints vox_centers_erose_unoccupied_selected;
    for(auto &p : vox_centers_erose_unoccupied){
        if( p.GetZ () <= (ceil_Zmax - vox_l) && p.GetZ () >= (floor_Zmin + vox_l)){
            vox_centers_erose_unoccupied_selected.push_back (p);
        }
    }

    /// ConnCompAnalyze
    SetColor (GREEN);
    printf("... Performing ConnComp Segmentation ...\n");
    SetColor (GRAY);
    SegmentationParameters *segmentationParameters;
    segmentationParameters = new SegmentationParameters;
    segmentationParameters -> MaxDistanceInComponent () = 1.0;

    LaserPoints unoccupied_ConnComp_segments;
    unoccupied_ConnComp_segments = Connected_Component_Segmentation (vox_centers_erose_unoccupied_selected, segmentationParameters);

    cout << endl;
    strcpy (str_root,root);
    unoccupied_ConnComp_segments.Write(strcat(str_root, "unoccupied_ConnComp_segments.laser"), false);

    /// transfer the segment labels to the vox_cents_erose for dilation step
    /// extract the bounds of the voxel space
    //get the databounds from the original laserpoints
    SetColor (GREEN);
    printf("... Transferring segmentNo to voxels ...\n");
    SetColor (GRAY);
    DataBoundsLaser db = laserpoints.DeriveDataBounds(0);
    double  min_X,
            min_Y,
            min_Z;

    min_X = db.Minimum().GetX();
    min_Y = db.Minimum().GetY();
    min_Z = db.Minimum().GetZ();
    for (auto &p : unoccupied_ConnComp_segments){

        uint i = floor((p.GetX () - min_X)/vox_l);
        uint j = floor((p.GetY () - min_Y)/vox_l);
        uint k = floor((p.GetZ () - min_Z)/vox_l);

        int segment_no;
        if(p.HasAttribute (SegmentNumberTag))
            segment_no = p.Attribute (SegmentNumberTag);

        int vox_inx;
        vox_inx = vec_ijk[i][j][k];
        vox_cents_erose[vox_inx].SetAttribute(SegmentNumberTag, segment_no);
    }

    // for debug and double check
    cout << endl;
    strcpy (str_root,root);
    vox_cents_erose.Write(strcat(str_root, "vox_centers_erosion_withSegNo.laser"), false);


    /// performing a dilation on erosion result
    SetColor (GREEN);
    printf ("... performing DILATION ...\n");
    SetColor (GRAY);
    int dilation_win =9;  //9, 5;
    LaserPoints vox_cents_dilate, vox_centers_dilate_unoccupied;
    vox_cents_dilate = vox.dilation(vox_cents_erose, 1,
                                    dilation_win*vox_l, dilation_win*vox_l, dilation_win*vox_l, vec_ijk);
    cout << endl;
    strcpy (str_root,root);
    vox_cents_dilate.Write(strcat(str_root, "vox_cents_dilate.laser"), false);
    vox_centers_dilate_unoccupied = vox_cents_dilate.SelectTagValue(LabelTag, 0);  // 0 for empty sapce, 1 for other
    cout << endl;
    strcpy (str_root,root);
    vox_centers_dilate_unoccupied.Write(strcat(str_root, "vox_centers_dilate_unoccupied.laser"), false);

    /// offset
    //// this offset is to keep the dilated layers near the ceiling and floor
    /// important step to choose the points between the ceilign and floor for resegmentation
    double ceiling_offset_threshold, floor_offset_threshold;
    ceiling_offset_threshold = 3*vox_l; //3 /// important threshold to keep the partitions seperated
    floor_offset_threshold = 2*vox_l;   //2 /// important threshold

    double ceil_Zmax_offset, floor_Zmin_offset;
    ceil_Zmax_offset    = ceil_Zmax  - ceiling_offset_threshold;  /// offset to lower value
    floor_Zmin_offset   = floor_Zmin + floor_offset_threshold; /// offset to higher values

    /// AGAIN select points between floor and ceiling for propagation of segment numbers with majority filtering
    LaserPoints vox_centers_dilate_unoccupied_reselected;
    for(auto &p : vox_centers_dilate_unoccupied){
        if( p.GetZ() <= ceil_Zmax_offset && p.GetZ() >= floor_Zmin_offset){
            //if( p.GetZ() <= (ceil_db.Minimum ().GetZ () - 2*vox_l) && p.GetZ() >= (floor_db.Maximum ().GetZ () + 2*vox_l)){ // not correct for ceilings with different heights
            vox_centers_dilate_unoccupied_reselected.push_back (p);
        }
    }

    cout << endl;
    strcpy (str_root,root);
    vox_centers_dilate_unoccupied_reselected.Write(strcat(str_root, "vox_centers_dilate_unoccupied_reselected.laser"), false);

    // Derive the edges that define the neighbour relations
    TINEdges     *edges;
    edges = vox_centers_dilate_unoccupied_reselected.DeriveEdges(*segmentationParameters);

    /// apply majority_filter
    //do{
    vox_centers_dilate_unoccupied_reselected.MajorityFilter (*segmentationParameters, *edges); /// resegmentation
    //}
    //while (!vox_centers_dilate_unoccupied_reselected.HasAttribute (SegmentNumberTag));

    cout << endl;
    strcpy (str_root,root);
    vox_centers_dilate_unoccupied_reselected.Write(strcat(str_root, "vox_centers_dilate_unoccupied_resegment.laser"), false);

    /// remove the biggest segment, since it is all the voxels around the building
    /// there is the chance of removing some of the interior spaces if they are connected
    LaserPoints reduced_lp;
    reduced_lp = vox_centers_dilate_unoccupied_reselected;
    int dominant_segment_nr, count;
    dominant_segment_nr = reduced_lp.MostFrequentAttributeValue (SegmentNumberTag, count);
    reduced_lp.RemoveTaggedPoints (dominant_segment_nr, SegmentNumberTag);

    cout << endl;
    strcpy (str_root,root);
    reduced_lp.Write(strcat(str_root, "first_result.laser"), false);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"total processing time: "<< duration/60 << "m" << '\n';
}



LaserPoints select_byZ(LaserPoints &dilation_result, double min_z, double max_z){
    LaserPoints vox_centers_dilate_unoccupied_cropbyZ;
    for(auto &p : dilation_result){
        if( p.GetZ() <= max_z && p.GetZ() >= min_z){
            vox_centers_dilate_unoccupied_cropbyZ.push_back (p);
        }
    }

    return vox_centers_dilate_unoccupied_cropbyZ;
}


LaserPoints transfer_voxel_attributes (LaserPoints original_points, LaserPoints voxel_centers_reference,
                                LaserPoints voxel_cents_temp, double vox_l){

    ///
    LaserVoxel vox(original_points, vox_l);

    LaserPoints vox_centers, vox_centers_occupied;
    char * vox_centers_path;
    //strcpy (str_root,root);
    //vox_centers_path = strcat(str_root, "vox_centers.laser");
    vector< vector < vector < int > > > vec_ijk;
    /// generating vox_centers in the given path and generating a vector to relate voxel ijk and centers
    vec_ijk = vox.export_vox_centres(1, vox_centers);
    //vox_centers.Read(vox_centers_path);



    /// extract the bounds of the voxel space
    //get the databounds from the ls
    DataBoundsLaser db = original_points.DeriveDataBounds(0);
    double  min_X,
            min_Y,
            min_Z;

    min_X = db.Minimum().GetX();
    min_Y = db.Minimum().GetY();
    min_Z = db.Minimum().GetZ();

    //length of vector componetnts: compute index for max vals - +1;
/*    uint  vox_num_X,
          vox_num_Y,
          vox_num_Z;
    //length of vector components: compute index for max vals - +1;
    vox_num_X = floor((db.Maximum().GetX() - min_X)/vox_l) + 1;
    vox_num_Y = floor((db.Maximum().GetY() - min_Y)/vox_l) + 1;
    vox_num_Z = floor((db.Maximum().GetZ() - min_Z)/vox_l) + 1;*/

    for (auto &p : voxel_cents_temp){

        uint i = floor((p.GetX () - min_X)/vox_l);
        uint j = floor((p.GetY () - min_Y)/vox_l);
        uint k = floor((p.GetZ () - min_Z)/vox_l);

        int segment_no, label_no;
        if(p.HasAttribute (SegmentNumberTag))
            segment_no = p.Attribute (SegmentNumberTag);

        if(p.HasAttribute (LabelTag))
            label_no = p.Attribute (LabelTag);


        int vox_inx;
        vox_inx = vec_ijk[i][j][k];
        voxel_centers_reference[vox_inx].SetAttribute(SegmentNumberTag, segment_no);
        voxel_centers_reference[vox_inx].SetAttribute(LabelTag, label_no);
        //TODO:  what about labels, shouldn't be transferred?
    }

    return voxel_centers_reference;
}


/* This function gets the segmented laser points, per segment generates the voxels, fit a plane and select points that
 * are close to the plane. This is for generating points in the gaps and openings for space partitioning step*/
LaserPoints Planar_Surface_Reconstruction_Voxels (LaserPoints &laserPoints, double voxel_length){

    /// first make a vector of all segments
    //vector <LaserPoints> segments_vec;
    //segments_vec =  PartitionLpByTag (laserPoints, SegmentNumberTag);

    vector<int> segment_numbers;
    segment_numbers = laserPoints.AttributeValues (SegmentNumberTag);
    LaserPoints surfaces_voxels;
    for (auto & seg_no : segment_numbers){
        LaserPoints segment;
        segment = laserPoints.SelectTagValue (SegmentNumberTag, seg_no);
        /// reconstruct the voxel from the segment
        LaserVoxel vox(segment, voxel_length);
        LaserPoints voxel_centers;
        vox.export_vox_centres (2, voxel_centers);
        voxel_centers.SetAttribute (SegmentNumberTag, seg_no);

/// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
    LaserPoints surface_voxels;
    Plane plane;
    plane = segment.FitPlane (seg_no);

    int planar_vox_cnt =0, occupy_cnt=0;
    for (int i=0; i<voxel_centers.size (); i++){
        double dist_plane_voxel;
        dist_plane_voxel = plane.Distance(voxel_centers[i]);
        if(fabs (dist_plane_voxel) < voxel_length/2.0){
            //planar_vox_cnt++;
            //if(voxel_centers[i].Attribute (LabelTag) == 11)
               // occupy_cnt++;
            surface_voxels.push_back(voxel_centers[i]);
        }
    }
    surfaces_voxels.AddPoints (surface_voxels);
    }
    return surfaces_voxels;
}

/*  this function fills the gaps and holes with the voxel center in the segment based on the fitting plane
 * voxel length should be almost equal to the point spacing*/
LaserPoints Filling_Gaps_withVoxelCenters (LaserPoints &laserPoints, double voxel_length){

    /// first make a vector of all segments
    //vector <LaserPoints> segments_vec;
    //segments_vec =  PartitionLpByTag (laserPoints, SegmentNumberTag);

    vector<int> segment_numbers;
    segment_numbers = laserPoints.AttributeValues (SegmentNumberTag);
    LaserPoints repaired_segments;
    for (auto & seg_no : segment_numbers){
        LaserPoints segment;
        segment = laserPoints.SelectTagValue (SegmentNumberTag, seg_no);
        /// reconstruct the voxel from the segment
        LaserVoxel vox(segment, voxel_length);
        LaserPoints voxel_centers;
        vox.export_vox_centres (2, voxel_centers);
        voxel_centers.SetAttribute (SegmentNumberTag, seg_no);

/// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
        LaserPoints repaired_segment;
        repaired_segment = segment;
        Plane plane;
        plane = segment.FitPlane (seg_no);

        int planar_vox_cnt =0, occupy_cnt=0;
        for (int i=0; i<voxel_centers.size (); i++){
            double dist_plane_voxel;
            dist_plane_voxel = plane.Distance(voxel_centers[i]);
            if(fabs (dist_plane_voxel) < voxel_length/2.0){
                //planar_vox_cnt++;
                if(voxel_centers[i].Attribute (LabelTag) == 0){ // if is an empty voxel
                   repaired_segment.push_back (voxel_centers[i]);
                }
            }
        }
        repaired_segments.AddPoints (repaired_segment);
    }
    return repaired_segments;
}


