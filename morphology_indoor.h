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

LaserPoints transfer_voxel_attributes (LaserPoints original_points, LaserPoints voxel_centers_reference,
                                       LaserPoints voxel_cents_temp, double vox_l);

//LaserPoints laserpoints;

