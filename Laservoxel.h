
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


#ifndef LaserVoxel_H
#define LaserVoxel_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>       /* fmod */
#include "LaserPoints.h"
#include <vector>
#include <map>
#include <KNNFinder.h>


#ifndef uint
typedef unsigned int uint;
#endif


bool double_equals(double a, double b, double epsilon = 0.001);

class LaserVoxel
{
  
  private:
    vector< vector < vector < LaserPoints* > > > Vox;
    //vector< vector < vector < int > > > v, vec;

    /// attributes
    int tag;
    int nrOfPoints;
    ///edge length of a voxel
    double vox_length; 
    ///number of voxels in XYZ
    uint vox_num_X, vox_num_Y, vox_num_Z;
    ///min value in real X Y Z (bounding box)
    double min_X, min_Y, min_Z;
    ///given a real object coordinate the indices for the Voxel field are computed
    uint index_from_real(double real, double min) {return (uint) floor((real-min)/vox_length);}
    uint index_from_realX(double real) {return (index_from_real(real,min_X));}
    uint index_from_realY(double real) {return (index_from_real(real,min_Y));}
    uint index_from_realZ(double real) {return (index_from_real(real,min_Z));}
    
 public:
   LaserVoxel(LaserPoints ls, double vox_l);
   ~LaserVoxel();

    void setAttributes(int min_points_in_vox=1);
    vector< vector < vector < LaserPoints* > > >& getVox();
    void setNrOfPoints(int nr);
    void setTag(int t);

    int getNrOfPoints();
    int getTag();

    void statistics();
    //void morphological_indoor (char* laserFile, double resolution=1.0, char *method);


    LaserPoint voxel_center(int i=0, int j=0, int k=0);
    /// closing
    LaserPoints morph_close(LaserPoints vox_centers, uint min_morph=2,double struct_element_a=1.0,
                            double struct_element_b=1.0, double struct_element_c=1.0,
                            vector< vector < vector < int > > > vec ={});
    /// opening
    LaserPoints morph_open(LaserPoints vox_centers, uint min_morph=2,double struct_element_a=1.0,
                           double struct_element_b=1.0, double struct_element_c=1.0,
                           vector< vector < vector < int > > > vec ={});
    /// erosion
    LaserPoints erosion(LaserPoints vox_cnts, uint min_erosion=1,double struct_element_a=1.0,
                        double struct_element_b=1.0, double struct_element_c=1.0,
                        vector< vector < vector < int > > > vec_ijk={});
    /// dilation
    LaserPoints dilation(LaserPoints vox_cnts, uint min_dilation=1,double struct_element_a=1.0,
                                     double struct_element_b=1.0, double struct_element_c=1.0,
                                     vector< vector < vector < int > > > vec_ijk={});

    /// defining a template (e.g. a door size) the method finds occupied voxels in the border of the templates
    /// the center of the template is the refrence for marching the nbr-hood voxels
    LaserPoints structure_template(LaserPoints vox_cnts, LaserPoints traj_points, uint min_dilation=1,double struct_element_a=1.0,
                                   double struct_element_b=1.0, double struct_element_c=1.0,
                                   vector< vector < vector < int > > > vec_ijk={});


    LaserPoints structure_template_cylinder(LaserPoints vox_cnts, LaserPoints traj_points, uint min_dilation=1,double struct_element_a=1.0,
                                   double struct_element_b=1.0, double struct_element_c=1.0,
                                   vector< vector < vector < int > > > vec_ijk={});

    LaserPoints door_detection(LaserPoints vox_cnts, LaserPoints traj_points, uint min_dilation=1,double struct_element_a=1.0,
                                            double struct_element_b=1.0, double struct_element_c=1.0,
                                            vector< vector < vector < int > > > vec_ijk={});

/*    LaserPoints closed_door(LaserPoints vox_cnts, LaserPoints traj_points, uint min_dilation=1,double struct_element_a=1.0,
                                            double struct_element_b=1.0, double struct_element_c=1.0,
                                            vector< vector < vector < int > > > vec_ijk={});*/


    /// compare 0 and 1 voxels between two datasets and save changed voxels from reference dataset
    LaserPoints compare_voxels_tag(LaserPoints vox_cnts_reference,LaserPoints vox_cnts_compare,
                                               vector< vector < vector < int > > > vec={});

    /// find where the voxel changes from 1 to zero
    LaserPoints edge_change();

    /// construct empty space with a margin of clutter and walls
    LaserPoints empty_space(int min_cnt_occupiedVox=2, int min_pnts_vox=2, double floor_z=0.0, double ceil_z=0.0, int windows_size = 3);

    /// check if a voxel has empty neighbor voxels
    int void_neighbourhood(uint min=1, int i=0, int j=0, int k=0, int window_size=3);

    /// check if a voxel has empty voxels on top of it in a nbr-hood
    int void_top(uint min=1, int i=0, int j=0, int k=0, int step_c=1, int window_size=3);

    void top_cylinder_cap(LaserPoints& door_center_topcap, LaserPoints& door_topcap_lp,
                          uint min=1, int i=0, int j=0, int k=0, int step_c=1, double r=0.50);


    vector< vector < vector < LaserPoints* > > >  trajlaser_to_voxel(LaserPoints traj_points);

   ///Filter: min. number of points in a voxel
   void filter_size(uint min=2,bool verbose=1);
   ///Filter according to neighb.hood: in the 26-neighb.hood of a voxel a min. number of points is required, otherwise it is deleted
   void filter_neighbour(uint min=1,bool verbose=1);
   LaserPoints export_all();
    /// export points inside a given voxel
    LaserPoints export_voxelpnts(int i=0, int j=0, int k=0);
    vector<vector<vector<int> > > export_vox_centres(int min_points_in_vox=1, char* output="D://test//morph//result//vox_centers.laser"); ///only exports the centres of voxels (where laserpoints are)
};

#endif
