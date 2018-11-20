//
// Created by NikoohematS on 9/8/2016.
//

#ifndef MORPHOLOGY_MORPHOLOGY_INDOOR_H
#define MORPHOLOGY_MORPHOLOGY_INDOOR_H

#endif //MORPHOLOGY_MORPHOLOGY_INDOOR_H

#include "LaserPoints.h"
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

void morphological_indoor(char* laserFile, double resolution, char *method);

void Erosion_step2 (LaserPoints laserpoints, LaserPoints vox_centers_erose_unoccupied,
                    LaserPoints vox_cents_erose, double vox_l);

LaserPoints select_byZ(LaserPoints &dilation_result, double max_z, double min_z);

LaserPoints transfer_voxel_attributes (LaserPoints original_points, LaserPoints voxel_centers_reference,
                                       LaserPoints voxel_cents_temp, double vox_l);

LaserPoints Planar_Surface_Reconstruction_Voxels (LaserPoints &laserPoints, double voxel_length);

LaserPoints Filling_Gaps_withVoxelCenters (LaserPoints &laserPoints, double voxel_length);

//LaserPoints laserpoints;

