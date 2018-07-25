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


using namespace std;


void morphological_indoor (char* laserFile, double resolution, char *method){

    method = (char *) "Erosion"; //"emptyspace" ; //"door_top_detection"; // "Erosion"

    //LaserVoxel vox(LaserPoints, double);
    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    //char *root = (char*)  "E:/BR_data/Diemen/process/out/door_detection/";
    //char *root = (char*) "D:/test/morph2/out/";
    char *root = (char*) "E:/publication_data/indoor_changeDetection/process/spaces/";
            strcpy (str_root,root); // initialize the str_root with root string

    /// read laser points
    LaserPoints laserpoints;
    //laserFile   = (char *) "D://test//morph//data//ifp_segmented.laser";  //morph_3rdFloor_small  //morph_small_83k_noFloorCeil
    //laserFile = (char*) "D://test//indoor_reconstruction//data//space_partitioning_test.laser";  // walls2nditer_floor_clutter_openings
    //laserFile = (char*) "D://test//morph//result//walls2nditer_nofloor_clutter_openings.laser"; //3rdfloor_modifiedpoints_nofloor_ceiling.laser";
    //laserFile = (char *) "E://BR_data//ZebR//doors_test.laser";  //door_2
    //laserFile = (char *) "D:/test/morph2/data/2rooms_all.laser";
    laserFile = (char *) "E:/publication_data/indoor_changeDetection/process/backpack_intersection_final/wall_floor_ceiling.laser";
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

    /// hardcode
    //ceil_Zmin = 1.23;
    //floor_Zmax = -1.71;

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
    vec_ijk = vox.export_vox_centres(1, vox_centers_path);
    vox_centers.Read(vox_centers_path);

    /// generating occupied voxels
    vox_centers_occupied = vox_centers.SelectTagValue(LabelTag, 1);
    //vox_centers_occupied.Write("D://test//morph//result//vox_centers_occupied.laser", false);
    //vox_centers_occupied.Write("E://BR_data//ZebR//out//doors_space//vox_centers_occupied.laser", false);
    strcpy (str_root,root);
    vox_centers_occupied.Write(strcat(str_root, "vox_centers_occupied.laser"), false);


/***********************************************************************************************************************/

    /*  Trajectory operations for door detection */
    /// read trajectory
/*    {
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
    }*/


    /*******************************************************************************************************************/

    /// door detection by top
    if (method == "door_top_detection"){
        LaserPoints top_doors_all;
        LaserPoints new_trajectory;
        top_doors_all = vox.door_detection(vox_centers, new_trajectory, 2, 5 * vox_l, 5 * vox_l, 19 * vox_l, vec_ijk); //7, 7, 23, vox_l=0.12 // 5, 5, 17, l=10cm // 5, 5, 19, l=10, d=10, #top=13
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
        printf("...performing EROSION... \n");
        LaserPoints vox_cents_erose, vox_centers_erose_unoccupied;
        LaserPoints vox_centers_erosion_lablechanged; /// extra outptu for changed labels
        int erosion_win = 13;
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

        /// for debug and double check
        //cout << endl;
        //strcpy (str_root,root);
        //vox_centers_erose_unoccupied_selected.Write(strcat(str_root, "vox_centers_erose_unoccupied_selected.laser"), false);

        /// ConnCompAnalyze
        printf("... Performing ConnComp Segmentation ...\n");
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
        printf ("... performing DILATION ...\n");
        int dilation_win =9;
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
        ceiling_offset_threshold = 3*vox_l; /// important threshold to keep the partitions seperated
        floor_offset_threshold = 2*vox_l;   /// important threshold

        double ceil_Zmax_offset, floor_Zmin_offset;
        ceil_Zmax_offset    = ceil_Zmax  - ceiling_offset_threshold;  /// offset to lower value
        floor_Zmin_offset   = floor_Zmin + floor_offset_threshold; /// offset to higher values

    /// AGAIN select points between floor and ceiling for propagation of segment numbers with majority filtering
        LaserPoints vox_centers_dilate_unoccupied_reselected;
        for(auto &p : vox_centers_dilate_unoccupied){
            if( p.GetZ() <= ceil_Zmax_offset && p.GetZ() >= floor_Zmin_offset){
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

    }

    if(method == "Dilation"){
        printf("...performing DILATION... \n");
        LaserPoints vox_cents_dilate, vox_centers_dilate_unoccupied;
        vox_cents_dilate = vox.dilation (vox_centers, 1, 9*vox_l, 9*vox_l, 9*vox_l, vec_ijk);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"total processing time: "<< duration/60 << "m" << '\n';
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
    vec_ijk = vox.export_vox_centres(1, "D:/test/morph2/out/tmp/vox_centers_temp.laser");
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


