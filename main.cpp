#include <iostream>
#include <cstdlib>
#include <vector>
#include "InlineArguments.h"
#include "LaserPoints.h"



using namespace std;

void PrintUsage()
{
    printf("Usage: morphological_indoor -input_laser <input laserpoints , foo.laser>\n");
    printf("                  -resolution <element structure size e.g. 1.00 m>\n");
    printf("                  -method <morphological method: -open, -close, -dilate, -erosion>\n");
}

int main(int argc, char *argv[]) {

    InlineArguments     *args = new InlineArguments(argc, argv);
    void morphological_indoor (char* laserFile, double resolution, char *method);
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
    //morphological_indoor (args->String("-input_laser"), args->Contains("-resolution"),args->String("-method"));
    //char * input_file = (char *) "E:/BR_data/ZebR/doors_test2.laser"; //doors_test //cc_sub2mil_basement_1mil_seg10cm
    char * input_file = (char *) "E://BR_data//Diemen//process//cropped_for_doordetection.laser"; //doors_test2.laser"; //block3_seg05cm_1-7mil_seg10cm_label0.laser"; //out/walls.laser"  ; //tworooms_noceil.laser"; //;
    morphological_indoor(input_file, 0, (char *) "door_top_detection");

    LaserPoints lp;
    //lp.Read("E:/BR_data/ZebR/out/doors_space/top_doors_lp_segmented.laser");
    double height = 1.80;//2.40;
    ObjectPoints corners;
    LineTopology polygon_line;
    //threedbbox(corners, polygon_line, lp, height);

    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

    return EXIT_SUCCESS;
}