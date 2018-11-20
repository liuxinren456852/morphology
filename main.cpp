#include <iostream>
#include <cstdlib>
#include <vector>
#include "InlineArguments.h"
#include "LaserPoints.h"
#include "morphology_indoor.h"
#include "ConnCompSegmentation.h"



using namespace std;

void PrintUsage()
{
    printf("Usage: morphological_indoor -input_laser <input laserpoints , foo.laser>\n");
    printf("                  -resolution <element structure size e.g. 1.00 m>\n");
    printf("                  -method <morphological method: -open, -close, -dilate, -erosion>\n");
}

int main(int argc, char *argv[]) {

    InlineArguments     *args = new InlineArguments(argc, argv);
    //void morphological_indoor (char* laserFile, double resolution, char *method);
    void threedbbox(ObjectPoints &corners, LineTopology &polygon_line, LaserPoints lp, double height);

    /// Check on required input files
/*    if (args->Contains("-usage") ||
        !args->Contains("-input_laser"))
        //!args->Contains("-method"))
    {
        if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }*/

    /// Call the main function
    char * input_file;
    //morphological_indoor (args->String("-input_laser"), args->Contains("-resolution"),args->String("-method"));
    //char * input_file = (char *) "E:/BR_data/ZebR/doors_test2.laser"; //doors_test //cc_sub2mil_basement_1mil_seg10cm
    //char * input_file = (char *) "E://BR_data//Diemen//process//cropped_for_doordetection.laser"; //doors_test2.laser"; //block3_seg05cm_1-7mil_seg10cm_label0.laser"; //out/walls.laser"  ; //tworooms_noceil.laser"; //;
    //input_file = (char*) "E:/publication_data/Haaksbergen/space_partitioning/repaired_segments.laser";
    morphological_indoor(input_file, 0, (char *) "door_top_detection");  /// the parameters are again overwritten inside the main function
    LaserPoints lp, vox_erosion, vox_erosion_unoccupied;
    //Erosion_step2 (lp, vox_erosion_unoccupied, vox_erosion, 0.10);
    LaserPoints planar_voxels, repaired_segments;
    //lp.Read("E:/publication_data/Haaksbergen/space_partitioning/second_floor/walls_merged.laser");
    //planar_voxels = Planar_Surface_Reconstruction_Voxels (lp , 0.05);
    //repaired_segments = Filling_Gaps_withVoxelCenters (lp, 0.05);
    //planar_voxels.Write("E:/publication_data/Haaksbergen/space_partitioning/planar_voxels.laser", false);
    //repaired_segments.Write("E:/publication_data/Haaksbergen/space_partitioning/second_floor/repaired_segments.laser", false);


    /// several selection by z
/*    lp.Read ("E:/publication_data/FB_dataset/process/out/spacepartitioning/vox_centers_dilate_unoccupied_reselected.laser");
    LaserPoints z_crop;
    z_crop = select_byZ (lp, -5.70, -4.60);
    z_crop.Write("E:/publication_data/FB_dataset/process/out/spacepartitioning/z_crop3.laser", false);*/

/*    LaserPoints vox_centers_dilate_unoccupied_reselected, vox_centers_erose_unoccupied;
    vox_centers_dilate_unoccupied_reselected.Read(
            R"(E:\publication_data\Delft_zebrevo\space_partitioning\out\spaces_20cm_eros9dil3\vox_centers_dilate_unoccupied_reselected.laser)");

    vox_centers_erose_unoccupied.Read(
            R"(E:\publication_data\Delft_zebrevo\space_partitioning\out\spaces_20cm_eros9dil5\vox_centers_erose_unoccupied.laser)");
    LaserPoints z_crop1, z_crop2, z_crop3, z_crop_erose;
    z_crop1 = select_byZ (vox_centers_dilate_unoccupied_reselected, 1.0, 5.0);
    z_crop2 = select_byZ (vox_centers_dilate_unoccupied_reselected, 4.93, 6.70);
    z_crop3 = select_byZ (vox_centers_dilate_unoccupied_reselected, 6.60, 11.60);
    z_crop_erose = select_byZ (vox_centers_erose_unoccupied, 1.0, 5.0);

    z_crop1.Write(R"(E:\publication_data\Delft_zebrevo\space_partitioning\out\spaces_20cm_eros9dil5\z_crop1.laser)", false);
    z_crop2.Write(R"(E:\publication_data\Delft_zebrevo\space_partitioning\out\spaces_20cm_eros9dil5\z_crop2.laser)", false);
    z_crop3.Write(R"(E:\publication_data\Delft_zebrevo\space_partitioning\out\spaces_20cm_eros9dil5\z_crop3.laser)", false);
    z_crop_erose.Write(R"(E:\publication_data\Delft_zebrevo\space_partitioning\out\spaces_20cm_eros9dil5\z_crop_erose.laser)", false);*/


    /*********/
    /* testing the label transfer after erosion */
    /*LaserPoints voxel_centers, voxel_centers_morph, lp, voxel_centers_relabeled;
    voxel_centers.Read("D:/test/morph2/data/voxel_centers_reference.laser");
    voxel_centers_morph.Read ("D:/test/morph2/data/voxel_empty_eroded_occupied_walls_1room.laser");
    lp.Read ("D:/test/morph2/data/2rooms.laser");
    voxel_centers_relabeled = transfer_voxel_attributes (lp, voxel_centers, voxel_centers_morph, 0.10);
    voxel_centers_relabeled.Write("D:/test/morph2/out/voxel_centers_relabeled.laser", false);*/
    /*******************************************************************************************************************/
    //LaserPoints lp;
    //lp.Read("E:/BR_data/ZebR/out/doors_space/top_doors_lp_segmented.laser");
    double height = 1.80;//2.40;
    ObjectPoints corners;
    LineTopology polygon_line;
    //threedbbox(corners, polygon_line, lp, height);

    /*******************************************************************************************************************/
    /*  test ConnComp Analyze*/
/*    LaserPoints lp, conn_comp_lp;
    lp.Read ("D:/test/morph2/out/vox_centers_dilate_unoccupied_selection.laser");
    SegmentationParameters *seg_param;
    seg_param = new SegmentationParameters;
    seg_param->MaxDistanceInComponent () = 1.0;
    conn_comp_lp = Connected_Component_Segmentation (lp, seg_param);
    conn_comp_lp.Write ("D:/test/morph2/out/vox_centers_dilate_unoccupied_segmented.laser", false);

    /// print number of segments
    int s1, sn;
    conn_comp_lp.AttributeRange (SegmentNumberTag, s1, sn);
    printf ("s1: %d and sn: %d \n", s1, sn);*/

    /******************************************************************************************************************/
    /* test majority filtering */
    //TODO : do... while majority_filter for unoccupied voxels until all of them get the segment numebr
/*    LaserPoints lp;
    lp.Read("D:/test/morph2/out/vox_centers_dilate_unoccupied_selected.laser");
    bool has_attribute=false;
    // Derive the edges that define the neighbour relations
    SegmentationParameters *segmentationParameters;
    segmentationParameters = new SegmentationParameters;  /// very important
    segmentationParameters -> MaxDistanceInComponent () = 1.0;
    TINEdges     *edges;
    edges = lp.DeriveEdges(*segmentationParameters);
    int cnt_noattribute;
    LaserPoints lp_temp;
    lp_temp = lp;
    do {
        lp_temp.MajorityFilter (*segmentationParameters, *edges); /// resegmentation
        lp_temp.Write("D:/test/morph2/out/lp_MF_tmp.laser", false);
        //
        //lp_temp = lp;
        cnt_noattribute=0;
        for (auto &p: lp_temp){
            if(!p.HasAttribute (SegmentNumberTag)){
                cnt_noattribute++;
            }
        }
        printf("cnt: %d \n", cnt_noattribute);
        lp_temp = lp_temp;
    }while(cnt_noattribute < 2);
    lp = lp_temp;*/


    /// while there is a point WITHOUT atrribute segmentTag, continue the Majority filter
/*    while(has_attribute = true){
        lp.MajorityFilter (*segmentationParameters, *edges); /// resegmentation
        for (auto &p: lp){
            if(!p.HasAttribute (SegmentNumberTag)){
                has_attribute =false;
            } else has_attribute = true;
        }
    }*/

//    lp.Write("D:/test/morph2/out/lp_MF.laser", false);

    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

    return EXIT_SUCCESS;
}