//
// Created by NikoohematS on 10/28/2016.
//


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

#include "Laservoxel.h"
#include <algorithm>
#include <vector>
#include <sstream>

#define EPS_DEFAULT 1e-4
//#include <LaserPointsFitting.h>


bool double_equals(double a, double b, double epsilon) {
    return std::abs(a - b) < epsilon;
}

void threedbbox(ObjectPoints& , LineTopology& , LaserPoints , double );

LaserVoxel::LaserVoxel(LaserPoints ls, double vox_l) {
    vox_length = vox_l;
    bool verbose = 1;

    //LaserPlane   dominanat_plane = FitPlane(ls);
    //Vector3D     dominanat_plane_normal = dominanat_plane.NormalDirection();
    //dominanat_plane_normal.Direction2D()


    //get the databounds from the ls
    DataBoundsLaser db = ls.DeriveDataBounds(0);
    min_X = db.Minimum().GetX();
    min_Y = db.Minimum().GetY();
    min_Z = db.Minimum().GetZ();

//get the databounds from the ls TO visualize
    LineTopology rect_top;
    ObjectPoints rect_objpts;
    LineTopologies bounding_box;
    ls.DeriveTIN();
    //db = ls.DeriveDataBounds(0);
    ls.EnclosingRectangle(0.1, rect_objpts, rect_top);
    rect_objpts.Write("D:\\test\\bounding_box.objpts");
    bounding_box.insert(bounding_box.end(), rect_top);
    bounding_box.Write("D:\\test\\bounding_box.top", false);

    //length of vector componetnts: compute index for max vals - +1;
    vox_num_X = index_from_realX(db.Maximum().GetX()) + 1;
    vox_num_Y = index_from_realY(db.Maximum().GetY()) + 1;
    vox_num_Z = index_from_realZ(db.Maximum().GetZ()) + 1;
    if (verbose) {
        //printf("Number of Laserpoints:%d\n",ls.size());
        printf("length of voxel:%.2f\n", vox_length);
        //printf("min_X=%.2f, min_Y=%.2f, min_Z=%.2f\n",min_X,min_Y,min_Z);
        printf("num of voxels in X:%d, Y:%d, Z:%d: Total:%d\n", vox_num_X, vox_num_Y, vox_num_Z,
               vox_num_X * vox_num_Y * vox_num_Z);

        printf("...allocating memory...wait\n");
    }
    //allocate memory
    Vox.resize(vox_num_X);
    for (uint i = 0; i < vox_num_X; i++) {
        Vox[i].resize(vox_num_Y);

        for (uint j = 0; j < vox_num_Y; j++) {
            Vox[i][j].resize(vox_num_Z);

            for (uint k = 0; k < vox_num_Z; k++)
                Vox[i][j][k] = NULL;
        }

    }

    if (verbose) printf("...putting data into structure...wait\n");
    //put the data into the voxel
    LaserPoints::iterator point;
    for (point = ls.begin(); point != ls.end(); point++) {
        LaserPoint lp = point->LaserPointRef();
        double X = lp.GetX();
        double Y = lp.GetY();
        double Z = lp.GetZ();

        uint i = index_from_realX(X);
        uint j = index_from_realY(Y);
        uint k = index_from_realZ(Z);

        //printf("in reading: i=%d, j=%d, k=%d\n",i,j,k);
        if (Vox[i][j][k] == NULL) Vox[i][j][k] = new LaserPoints;

        Vox[i][j][k]->push_back(lp);
    }
//if (verbose) printf("...done!\n");
} //Constructor

LaserVoxel::~LaserVoxel() {
    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                if (Vox[i][j][k] != NULL) delete (Vox[i][j][k]);
            }
}

void LaserVoxel::statistics() {
    ::map<uint, uint> statmap; //first: number of points per voxel, second: occurence
    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                if (Vox[i][j][k] != NULL) {
                    //printf("in cell %d %d %d: %d points\n",i,j,k,Vox[i][j][k]->LaserPointsReference().size());
                    uint size = Vox[i][j][k]->LaserPointsReference().size();
                    statmap[size]++;
                }
            }

    printf("\nStatistics:\n");
    uint total_voxels = 0;
    for (::map<uint, uint>::iterator it = statmap.begin(); it != statmap.end(); it++) {
        printf("number of points/voxel: %d, occurence: %d\n", it->first, it->second);
        total_voxels += it->second;
    }
    printf("\nTotal number of voxels:%d\n", total_voxels);
    printf("\n\n");
}

void LaserVoxel::filter_size(uint min, bool verbose) {
    if (verbose) printf("Voxel fiter: min number of points per voxel required: %d\n", min);
    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                if (Vox[i][j][k] != NULL)
                    if (Vox[i][j][k]->LaserPointsReference().size() < min) {//delete(Vox[i][j][k]);
                        Vox[i][j][k] = NULL;
                    }
            }
    if (verbose) printf("...done\n");
}

void LaserVoxel::filter_neighbour(uint min, bool verbose) {
    if (verbose) printf("Voxel fiter: min number of points in neighborhood required: %d\n", min);
    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                if (Vox[i][j][k] != NULL) {
                    uint found_p = 0;
                    uint zero = 0;

                    for (uint a = max(i - 1, zero); a <= i + 1; a++) //the indices may not be <0
                    {
                        if (a >= vox_num_X) continue;

                        for (uint b = max(j - 1, zero); b <= j + 1; b++) {
                            if (b >= vox_num_Y) continue;

                            for (uint c = max(k - 1, zero); c <= k + 1; c++) {
                                if (c >= vox_num_Z) continue;
                                if (a == i && b == j && c == k) continue; //the point itself is not counted
                                if (Vox[a][b][c] != NULL)
                                    found_p += Vox[a][b][c]->LaserPointsReference().size();
                            }//c
                        }//b
                    }//a
                    if (found_p < min) Vox[i][j][k] = NULL;
                }
            }
    if (verbose) printf("...done\n");
}

LaserPoint LaserVoxel::voxel_center(int i, int j, int k) {
    double X = min_X + (double) ((double) (i + 0.5) * vox_length);
    double Y = min_Y + (double) ((double) (j + 0.5) * vox_length);
    double Z = min_Z + (double) ((double) (k + 0.5) * vox_length);
    LaserPoint lp(X, Y, Z);
    return lp;
}

LaserPoints LaserVoxel::export_all() {
    LaserPoints l;
    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++)
                if (Vox[i][j][k] != NULL) {
                    //l=l+Vox[i][j][k]->LaserPointsReference(); //+ operator too expensive
                    LaserPoints::iterator point;
                    LaserPoints tmpl = Vox[i][j][k]->LaserPointsReference();

                    for (point = tmpl.begin(); point != tmpl.end(); point++) {
                        LaserPoint lp = point->LaserPointRef();
                        l.push_back(lp);
                    }
                }
    return l;
}

LaserPoints LaserVoxel::export_voxelpnts(int i, int j, int k) {
    LaserPoints l;
    if (Vox[i][j][k] != NULL) {
        //l=l+Vox[i][j][k]->LaserPointsReference(); //+ operator too expensive
        LaserPoints::iterator point;
        LaserPoints tmpl = Vox[i][j][k]->LaserPointsReference();

        for (point = tmpl.begin(); point != tmpl.end(); point++) {
            LaserPoint lp = point->LaserPointRef();
            l.push_back(lp);
        }
    }
    return l;
}

vector<vector<vector<int> > > LaserVoxel::export_vox_centres(int min_points_in_vox, char *output) {
    LaserPoints l;
    size_t found_points_in_vox = 0;
    size_t inx;
    vector<vector<vector<int> > > v;

    //allocate memory to v
    v.resize(vox_num_X);
    for (uint i = 0; i < vox_num_X; i++) {
        v[i].resize(vox_num_Y);

        for (uint j = 0; j < vox_num_Y; j++) {
            v[i][j].resize(vox_num_Z);

            for (uint k = 0; k < vox_num_Z; k++)
                v[i][j][k] = 0;
        }
    }

    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                double X = min_X + (double) ((double) (i + 0.5) * vox_length);
                double Y = min_Y + (double) ((double) (j + 0.5) * vox_length);
                double Z = min_Z + (double) ((double) (k + 0.5) * vox_length);
                if (Vox[i][j][k] != NULL) {
                    found_points_in_vox = Vox[i][j][k]->LaserPointsReference().size();
                    //also approximate the color...
                    int rsum = 0, gsum = 0, bsum = 0, num = 0;
                    LaserPoints::iterator point;
                    LaserPoints tmpl = Vox[i][j][k]->LaserPointsReference();
                    for (point = tmpl.begin(); point != tmpl.end(); point++) {
                        LaserPoint lp = point->LaserPointRef();
                        rsum += lp.Red();
                        gsum += lp.Green();
                        bsum += lp.Blue();
                        num++;
                    }
                    LaserPoint lp(X, Y, Z);
                    {
                        lp.SetColour((int) rsum / num, (int) gsum / num, (int) bsum / num);
                        /// set LabelTag 1 if nr of points in the voxel are more than min_points_in_vox, otherwise 0
                        lp.SetAttribute(LabelTag, found_points_in_vox >= min_points_in_vox ? 1 : 0);
                        int vox_points = found_points_in_vox;
                        lp.SetAttribute(ScalarTag, vox_points);
                        l.push_back(lp);
                        inx = l.size() - 1;
                        v[i][j][k] = int(inx);
                    }
                } else /// export empty voxel-centers (empty = voxel without point)
                {
                    LaserPoint lp(X, Y, Z);
                    lp.SetAttribute(LabelTag, 0);
                    lp.SetAttribute(ScalarTag, 0);
                    l.push_back(lp);
                    inx = l.size() - 1;
                    v[i][j][k] = int(inx);
                }
            }

    l.Write(output, false);
    //l.Write("D://test//morph//result//vox_centers.laser",false);
    return v;
}

LaserPoints LaserVoxel::compare_voxels_tag(LaserPoints vox_cnts_reference, LaserPoints vox_cnts_compare,
                                           vector<vector<vector<int> > > vec) {
    LaserPoints changed_voxels;
    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                int ijk_inx = vec[i][j][k];
                if (vox_cnts_reference[ijk_inx].Attribute(LabelTag) != vox_cnts_compare[ijk_inx].Attribute(LabelTag)) {
                    changed_voxels.push_back(vox_cnts_reference[ijk_inx]);
                }
            }
    return changed_voxels;
}


// ************************************************** CLOSING ********************************************************//
// *******************************************************************************************************************//

LaserPoints LaserVoxel::morph_close(LaserPoints vox_centers, uint min_morph, double struct_element_a,
                                    double struct_element_b, double struct_element_c,
                                    vector<vector<vector<int> > > vec) {

    /// dilation
    printf("...performing DILATION... \n");
    LaserPoints vox_centers_dilate, vox_centers_dilate_occupied;
    vox_centers_dilate = this->dilation(vox_centers, min_morph, struct_element_a, struct_element_b, struct_element_c,
                                        vec);
    cout << endl;
    vox_centers_dilate.Write("D://test//morph//result//vox_centers_dilate.laser", false);
    vox_centers_dilate_occupied = vox_centers_dilate.SelectTagValue(LabelTag, 1);
    cout << endl;
    vox_centers_dilate_occupied.Write("D://test//morph//result//vox_centers_dilate_occupied.laser", false);

    /// erosion
    printf("...performing EROSION... \n");
    LaserPoints vox_centers_erosion;
    vox_centers_erosion = this->erosion(vox_centers_dilate, min_morph, struct_element_a, struct_element_b,
                                        struct_element_c, vec);
    LaserPoints vox_centers_close = vox_centers_erosion;

    return vox_centers_close;
}

// ************************************************** OPENING ********************************************************//
// *******************************************************************************************************************//
LaserPoints LaserVoxel::morph_open(LaserPoints vox_centers, uint min_morph, double struct_element_a,
                                   double struct_element_b, double struct_element_c,
                                   vector<vector<vector<int> > > vec) {

    /// erosion
    printf("...performing EROSION... \n");
    LaserPoints vox_centers_erosion, vox_centers_erosion_occupied;
    vox_centers_erosion = this->erosion(vox_centers, min_morph, struct_element_a, struct_element_b,
                                        struct_element_c, vec);
    cout << endl;
    vox_centers_erosion.Write("D://test//morph//result//vox_centers_erosion.laser", false);
    vox_centers_erosion_occupied = vox_centers_erosion.SelectTagValue(LabelTag, 1);
    cout << endl;
    vox_centers_erosion_occupied.Write("D://test//morph//result//vox_centers_erosion_occupied.laser", false);

    /// dilation
    printf("...performing DILATION... \n");
    LaserPoints vox_centers_dilate;
    vox_centers_dilate = this->dilation(vox_centers_erosion, min_morph, struct_element_a, struct_element_b,
                                        struct_element_c, vec);

    LaserPoints vox_centers_open = vox_centers_dilate;
    return vox_centers_open;
}

// ************************************************** DILATION *******************************************************//
// *******************************************************************************************************************//
LaserPoints LaserVoxel::dilation(LaserPoints vox_cnts, uint min_dilation, double struct_element_a,
                                 double struct_element_b, double struct_element_c,
                                 vector<vector<vector<int> > > vec_ijk) {


    /// n,m,r are the numbers of voxels that window covers in x,y,z dimensions
    uint n = floor(struct_element_a / vox_length); /// n is the number of voxels the a-dimension of window covers
    uint m = floor(struct_element_b / vox_length);
    uint r = floor(struct_element_c / vox_length);

    /// respectively aligned with X,Y,Z axis
    /// number of voxel counting for neighbors before and after current voxel
    int step_a = (n - 1) / 2; // n-1 because we exclude i-th index
    int step_b = (m - 1) / 2;
    int step_c = (r - 1) / 2;

    /// dilation
    LaserPoints vox_centers_dilate;
    vox_centers_dilate = vox_cnts; // vox_centers_dilate later will be labeled
    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = 0; k < int(vox_num_Z); k++) {
                printf("\r in reading: i=%d, j=%d, k=%d", i, j, k); /// debugger
                fflush(stdout);

                int a = 0, b = 0, c = 0;
                uint labeltag1_cnt = 0;

                for (a = i - step_a; a <= (i + step_a); a++) {
                    if (a < 0) continue;
                    if (a >= int(vox_num_X))
                        continue;
                    for (b = j - step_b; b <= (j + step_b); b++) {
                        if (b < 0) continue;
                        if (b >= int(vox_num_Y))
                            continue;
                        for (c = k - step_c; c <= (k + step_c); c++) {
                            if (c < 0) continue;
                            if (c >= int(vox_num_Z))
                                continue;
                            if (a == i && b == j && c == k) continue; //the point itself is not counted
                            //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger

                            int abc_inx = vec_ijk[a][b][c];
                            int abc_labeltag = vox_cnts[abc_inx].Attribute(LabelTag);

                            //win_centers.push_back(vox_centers[abc_inx]); //debugger
                            if (abc_labeltag == 1) {
                                labeltag1_cnt++;
                                if (labeltag1_cnt >= min_dilation)
                                    goto ChangeLabel1;
                            }
                        } //c
                    } //b
                }//a
                ChangeLabel1:

                if (labeltag1_cnt >= min_dilation) {
                    int ijk_inx = vec_ijk[i][j][k];
                    vox_centers_dilate[ijk_inx].SetAttribute(LabelTag, 1);
                }
            }
    //vox_centers_dilate.Write("D://test//morph//vox_centers_dilate.laser", false);

    return vox_centers_dilate;
}

// ************************************************** EROSION ********************************************************//
// *******************************************************************************************************************//
LaserPoints LaserVoxel::erosion(LaserPoints vox_cnts, uint min_erosion, double struct_element_a,
                                double struct_element_b, double struct_element_c,
                                vector<vector<vector<int> > > vec_ijk) {


    /// n,m,r are the numbers of voxels that window covers in x,y,z dimensions
    uint n = floor(struct_element_a / vox_length); /// n is the number of voxels the a-dimension of window covers
    uint m = floor(struct_element_b / vox_length);
    uint r = floor(struct_element_c / vox_length);

    /// respectively aligned with X,Y,Z axis
    /// number of voxel counting for neighbors before and after current voxel
    int step_a = (n - 1) / 2; // n-1 because we exclude i-th index
    int step_b = (m - 1) / 2;
    int step_c = (r - 1) / 2;

    /// erosion
    LaserPoints vox_cnt_erosion;
    vox_cnt_erosion = vox_cnts; // vox_centers_erosion later will be labeled
    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = 0; k < int(vox_num_Z); k++) {
                printf("\r in reading: i=%d, j=%d, k=%d", i, j, k); /// debugger
                fflush(stdout);

                int a = 0, b = 0, c = 0;
                //uint labeltag1_cnt = 0;
                uint labeltag0_cnt = 0;

                for (a = i - step_a; a <= (i + step_a); a++) {
                    if (a < 0) continue;
                    if (a >= int(vox_num_X))
                        continue;
                    for (b = j - step_b; b <= (j + step_b); b++) {
                        if (b < 0) continue;
                        if (b >= int(vox_num_Y))
                            continue;
                        for (c = k - step_c; c <= (k + step_c); c++) {
                            if (c < 0) continue;
                            if (c >= int(vox_num_Z))
                                continue;
                            if (a == i && b == j && c == k) continue; //the point itself is not counted
                            //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger

                            int abc_inx = vec_ijk[a][b][c];
                            int abc_labeltag = vox_cnts[abc_inx].Attribute(LabelTag);

                            //win_centers.push_back(vox_centers[abc_inx]); //debugger
                            //if (abc_labeltag == 1) labeltag1_cnt++;
                            if (abc_labeltag == 0) {
                                labeltag0_cnt++;
                                if (labeltag0_cnt >= min_erosion)
                                    goto ChangeLabel0;
                            }
                        } //c
                    } //b
                }//a
                ChangeLabel0:

                if (labeltag0_cnt >= min_erosion) {
                    int ijk_inx = vec_ijk[i][j][k];
                    vox_cnt_erosion[ijk_inx].SetAttribute(LabelTag, 0);
                }
/*                if (labeltag1_cnt <= min_erosion) {
                    int ijk_inx = vec_ijk[i][j][k];
                    vox_cnt_erosion[ijk_inx].SetAttribute(LabelTag, 0);
                }*/
            }
    //vox_centers_erosion.Write("D://test//morph//vox_centers_erosion.laser", false);

    return vox_cnt_erosion;
}

LaserPoints LaserVoxel::edge_change() {

    //LaserPoint edge_voxel;
    LaserPoints edge_voxels, occupied_voxels;

    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                if (Vox[i][j][k] != NULL) {
                    uint found_p = 0;
                    uint zero = 0;
                    bool empty_neighbor_flag;
                    empty_neighbor_flag = 0;

                    for (uint a = max(i - 1, zero); a <= i + 1; a++) //the indices may not be <0
                    {
                        if (a >= vox_num_X) continue;

                        for (uint b = max(j - 1, zero); b <= j + 1; b++) {
                            if (b >= vox_num_Y) continue;

                            for (uint c = max(k - 1, zero); c <= k + 1; c++) {
                                if (c >= vox_num_Z) continue;
                                if (a == i && b == j && c == k) continue; //the point itself is not counted
                                if (Vox[a][b][c] == NULL) {
                                    //LaserPoint edge_voxel_empty = this -> voxel_center(a,b,c);
                                    double X = min_X + (double) ((double) (a + 0.5) * vox_length);
                                    double Y = min_Y + (double) ((double) (b + 0.5) * vox_length);
                                    double Z = min_Z + (double) ((double) (c + 0.5) * vox_length);
                                    LaserPoint edge_voxel_empty(X, Y, Z);
                                    edge_voxels.push_back(edge_voxel_empty);
                                    edge_voxels.SetAttribute(LabelTag, 0);
                                    empty_neighbor_flag = 1;
                                }
                            }//c
                        }//b
                    }//a
                    if (empty_neighbor_flag == 1) {
                        double X = min_X + (double) ((double) (i + 0.5) * vox_length);
                        double Y = min_Y + (double) ((double) (j + 0.5) * vox_length);
                        double Z = min_Z + (double) ((double) (k + 0.5) * vox_length);
                        LaserPoint occupied_voxel(X, Y, Z);
                        occupied_voxel.SetAttribute(LabelTag, 1);
                        occupied_voxels.push_back(occupied_voxel);
                    }
                }
            }
    edge_voxels.RemoveAlmostDoublePoints(false, 0.0001);
    occupied_voxels.Write("D://test//morph//result//occupied_voxels.laser", false);
    return edge_voxels;
}

/// function to output empty voxels with a margin of occupied voxels between floor and ceiling
/// ... smaller void nbr-hood more empty voxels between clutters, reference can be a door width
LaserPoints LaserVoxel::empty_space(int min_cnt_occupiedVox, int min_pnts_vox, double floor_z, double ceil_z) {

    std::ostringstream sstream;
    LaserPoints emptySapce_voxels, navigable_space;
    uint found_p = 0;
    floor_z = min_Z; //***** NOTE: just to override wrong ceili_z, should be removed later

    uint k_inx_floor = index_from_realZ(floor_z);
    uint k_inx_ceil = index_from_realZ(ceil_z);
    LaserPoints empty_space_Z;

    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = k_inx_floor + 1; k < int(vox_num_Z -
                                                  2); k += 2) {   //***** NOTE: k increments by 2, also replace vox_num_Z by  k_inx_ceil *****
                printf("\r in reading: i=%d, j=%d, k=%d", i, j, k); /// debugger
                fflush(stdout);

                bool flag_Null = 0, flag_minpoints = 0;

                if (Vox[i][j][k] != NULL) {
                    found_p = Vox[i][j][k]->LaserPointsReference().size();
                    if (found_p <= min_pnts_vox) flag_minpoints = 1;
                } else
                    flag_Null = 1;

                /// if current voxel is NULL OR has less than min_pnts_vox is empty voxel
                if (flag_Null || flag_minpoints) {
                    LaserPoint lp;
                    min_cnt_occupiedVox = 2;
                    int labeltag1_cnt;
                    /// key function
                    labeltag1_cnt = this->void_neighbourhood(min_pnts_vox, i, j, k, 9);
                    if (labeltag1_cnt <= min_cnt_occupiedVox) {
                        lp = this->voxel_center(i, j, k);
                        lp.SetAttribute(LabelTag, 0);
                        emptySapce_voxels.push_back(lp);
                        if (2 < k && k < 6) navigable_space.push_back(lp);

                        /// saving points in different z values
/*                        string      out_name;
                        sstream.str(""); //clear stream
                        sstream << k;
                        string k_str = sstream.str();
                        out_name = "D://test//morph//result//empty_space_" + k_str + ".laser";
                        empty_space_Z.push_back(lp);
                        cout << endl;
                        empty_space_Z.Write(out_name.c_str(), false);*/
                    }
                }
            } // k
    cout << endl;
    navigable_space.Write("D://test//morph//result//navigable_space.laser", false);
    return emptySapce_voxels;
}

/// this method is not reliable, better not to be used
vector<vector<vector<LaserPoints *> > > LaserVoxel::trajlaser_to_voxel(LaserPoints traj_points) {
    //put the data into the voxel
    LaserPoints::iterator point_it;
    vector<vector<vector<LaserPoints *> > > traj_Vox;
    for (point_it = traj_points.begin(); point_it != traj_points.end(); point_it++) {
        LaserPoint lp = point_it->LaserPointRef();
        lp.SetAttribute(LabelTag, 2);
        double X = lp.GetX();
        double Y = lp.GetY();
        double Z = lp.GetZ();

        uint i = index_from_realX(X);
        uint j = index_from_realY(Y);
        uint k = index_from_realZ(Z);
        traj_Vox[i][j][k] = Vox[i][j][k];

//printf("in reading: i=%d, j=%d, k=%d\n",i,j,k);
        if (traj_Vox[i][j][k] == NULL) traj_Vox[i][j][k] = new LaserPoints;

        traj_Vox[i][j][k]->push_back(lp);
    }
    return traj_Vox;
}

int LaserVoxel::void_neighbourhood(uint min, int i, int j, int k, int window_size) {

    int a = 0, b = 0, c = 0;

    uint labeltag1_cnt = 0;
    uint found_p = 0;

    int step_a = (window_size - 1) / 2; // each step counts one side of the voxel, e.g. for -i and +i
    int step_b = (window_size - 1) / 2;
    int step_c = (window_size - 1) / 2;

    /// check for n occupied neighbors
    for (a = i - step_a; a <= (i + step_a); a++) {
        if (a < 0) continue;
        if (a >= int(vox_num_X))
            continue;
        for (b = j - step_b; b <= (j + step_b); b++) {
            if (b < 0) continue;
            if (b >= int(vox_num_Y))
                continue;
            for (c = k - step_c; c <= (k + step_c); c++) {
                if (c < 0) continue;
                if (c >= int(vox_num_Z))
                    continue;
                if (a == i && b == j && c == k) continue; //the point itself is not counted
                //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger

                if (Vox[a][b][c] != NULL){
                    found_p = Vox[a][b][c]->LaserPointsReference().size();
                    if (found_p > min) labeltag1_cnt++;
                }
            } //c
        } //b
    } //a

    return labeltag1_cnt;
}


int LaserVoxel::void_top(uint min, int i, int j, int k, int step_c, int window_size) {

    int a = 0, b = 0, c = 0;
    uint labeltag1_cnt = 0;
    uint labeltag0_cnt = 0;
    uint found_p = 0;

    int step_a = (window_size - 1) / 2;
    int step_b = (window_size - 1) / 2;

    /// check for n occupied neighbors
    for (a = i - step_a; a <= (i + step_a); a++) {
        if (a < 0) continue;
        if (a >= int(vox_num_X))
            continue;
        for (b = j - step_b; b <= (j + step_b); b++) {
            if (b < 0) continue;
            if (b >= int(vox_num_Y))
                continue;
            for (c = k + step_c; c <= k + step_c + 1; c++) {
                if (c < 0) continue;
                if (c >= int(vox_num_Z))
                    continue;
                //if (a == i && b == j && c == k) continue; //the point itself is not counted
                //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger

                if (Vox[a][b][c] != NULL){
                    found_p = Vox[a][b][c]->LaserPointsReference().size();
                    if (found_p > min) labeltag1_cnt++;
                }
            } //c
        } //b
    } //a

    return labeltag1_cnt;
}

void LaserVoxel::top_cylinder_cap(LaserPoints& door_center_topcap, LaserPoints& door_topcap_lp,
                                  uint min, int i, int j, int k, int step_c, double cyl_radius){


    int step_a = (int) (cyl_radius / vox_length);
    int step_b = (int) (cyl_radius / vox_length);

    uint labeltag1_cnt = 0;
    uint found_p = 0;
    //LaserPoints door_center_topcap;

    /// create a cylinder for current ijk
    /// we need p1, p2 for axis and radius
    LaserPoint p1, p2;

    //cyl_radius = step_a * vox_length;  // or step_b they should be equal
    p1 = voxel_center(i, j, k);
    p2 = voxel_center(i, j, k + step_c);

    Position3D p1_pos, p2_pos;
    Line3D cyl_axis;
    double cyl_axis_len;
    p1_pos = Position3D(p1);
    p2_pos = Position3D(p2);

    /// calculate cylinder axis
    cyl_axis = Line3D(p1_pos, p2_pos);

    /// check for n occupied neighbors on top cap
    for (int a = i - step_a; a <= (i + step_a); a++) {
        if (a < 0) continue;
        if (a >= int(vox_num_X))
            continue;
        for (int b = j - step_b; b <= (j + step_b); b++) {
            if (b < 0) continue;
            if (b >= int(vox_num_Y))
                continue;
            for (int c = k + step_c; c <= k + step_c + 1; c++) {
                if (c < 0) continue;
                if (c >= int(vox_num_Z))
                    continue;
                //if (a == i && b == j && c == k) continue; //the point itself is not counted
                //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger

                if (Vox[a][b][c] != NULL){
                    found_p = Vox[a][b][c]->LaserPointsReference().size();
                    if (found_p > min) {
                        labeltag1_cnt++;
                        LaserPoint p = this -> voxel_center(a,b,c);
                        double p_dist_to_axis = cyl_axis.DistanceToPoint(Position3D(p));
                        if(p_dist_to_axis <= cyl_radius){
                            p.SetAttribute(LabelTag, 1);
                            door_center_topcap.push_back(p);
                            door_topcap_lp.AddPoints(this->export_voxelpnts(a,b,c)); // store laserpoints inside the p voxel
                        }
                    }
                }
            } //c
        } //b
    } //a

    //door_topcap_lp.Write("D://test//morph//result//door_topcap_lp_test.laser", false);
    //door_center_topcap.Write("D://test//morph//result//door_center_topcap_test.laser", false);
}


// ************************************************** Door Detection *************************************************//
LaserPoints LaserVoxel::door_detection(LaserPoints vox_cnts, LaserPoints traj_points, uint min,
                                                    double struct_element_a,
                                                    double struct_element_b, double struct_element_c,
                                                    vector<vector<vector<int> > > vec_ijk) {

    std::ostringstream sstream;
    /// n,m,r are the numbers of voxels that window covers in x,y,z dimensions
    uint n = floor(struct_element_a / vox_length); /// n is the number of voxels the a-dimension of window covers
    uint m = floor(struct_element_b / vox_length);
    uint r = floor(struct_element_c / vox_length);

    /// respectively aligned with X,Y,Z axis
    /// number of voxel counting for neighbors before and after current voxel
    int step_a = (n - 1) / 2; // n-1 because we exclude i-th index
    int step_b = (m - 1) / 2;
    int step_c = (r - 1) / 2;
    int door_count = 0;

    double cyl_radius = step_a * vox_length;
    LaserPoints top_doors,   // all voxels(center) above the door centers
                //top_door,    // voxels(centers) above a single door center
                doors_centers, // all doors centers
                closed_doors_cnt,
                top_doors_lp;  // all laser points above door centers
    int i_previous = 0, j_previous = 0;

    LineTopologies minrectangle_topolines;
    ObjectPoints   corners;

    KNNFinder<LaserPoint> finder_traj(traj_points);

    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = step_c; k < step_c + 3; k++) {  /// +3 becasue we search above the door center and below ceiling
                //printf("in reading: i=%d, j=%d, k=%d \n", i, j, k); /// debugger
                //fflush(stdout);

                bool vox_trajhoods = 0, vox_voidhoods = 0, vox_tophoods = 0;
                /// should be smaller than door size
                int ijk_inx = vec_ijk[i][j][k];

                /// check if there is a close by trajectory
                double vox_to_traj_dist = finder_traj.FindDistance(vox_cnts[ijk_inx], 1, EPS_DEFAULT);
                if (vox_to_traj_dist <= 0.15)
                    vox_trajhoods = 1;

                /// check if current voxel has less than N occupied neighbors (for open doors),
                if (vox_trajhoods){
                    int void_hood_window = n-2;
                    int nr_of_occupiedhoods = this->void_neighbourhood(min, i, j, k, void_hood_window);
                    if (nr_of_occupiedhoods <= 5)  /// being void indicator
                        vox_voidhoods = 1;
                }

                /// for closed doors
                /// check if current voxel has more than N occupied neighbors (for closed doors),
                bool  vox_occupiedhood =0;
                if (vox_trajhoods && vox_cnts[ijk_inx].Attribute(LabelTag) == 1){
                    int nr_of_occupiedhoods2 = this->void_neighbourhood(min, i, j, k, 3);
                    if (nr_of_occupiedhoods2 >= 17) // 17 = 9 +9 voxels -1 ijk voxel
                        vox_occupiedhood =1;
                }

                /// check if current voxel has some points on top-door
                if (vox_trajhoods){
                    LaserPoints tophoods_center_points, lp_tmp; //outputs of the function
                    this -> top_cylinder_cap(tophoods_center_points, lp_tmp, min, i, j, k, step_c, vox_length);
                    if (tophoods_center_points.size() >= 3)  /// # should be variable not a fix parameter
                        vox_tophoods = 1;
                }

                /// closed doors
                if (vox_tophoods && vox_occupiedhood){
                    LaserPoints top_closedcnt_points; // voxels above a closed_door center
                    LaserPoints top_closedcnt_lp; //  door voxel and laserpoints

                    this -> top_cylinder_cap(top_closedcnt_points, top_closedcnt_lp, min, i, j, k, step_c, cyl_radius);
                    if (top_closedcnt_points.size() >= 5) {
                        top_closedcnt_points.push_back(vox_cnts[ijk_inx]);
                        closed_doors_cnt.AddPoints(top_closedcnt_points);
                    }
                }

                /// three conditions to candidate door centers
                if (vox_tophoods && vox_trajhoods && vox_voidhoods){
                    LaserPoints top_center_points; // voxels above a door center
                    LaserPoints top_center_lp, top_door_lp; //  door voxel and laserpoints
                    bool new_door_cluster =0;
                    this -> top_cylinder_cap(top_center_points, top_center_lp, min, i, j, k, step_c, cyl_radius);

                    if (top_center_points.size() >= 5) {
                        top_center_points.push_back(vox_cnts[ijk_inx]);  // uncomment later

                        //printf("*** in reading: i=%d, j=%d, k=%d *** \n", i, j, k); /// debugger
                        /// clustering door centers and points per door
                        if ((abs(i - i_previous) > 5) || (abs(j - j_previous > 5))){
                            door_count++;
                            top_center_points.SetAttribute(SegmentNumberTag, door_count);
                            top_center_lp.SetAttribute(SegmentNumberTag, door_count);
                            /// add door center to top_points
                            doors_centers.push_back(vox_cnts[ijk_inx]);
                            //new_door_cluster = !new_door_cluster;
                            if (top_door_lp.size() == 0) top_door_lp = top_center_lp; // just applies for the first door

/*                            string out_name;
                            sstream.str(""); //clear stream
                            sstream << door_count;
                            string door_count_str = sstream.str();
                            out_name = "D://test//morph//result//top_door_" + door_count_str + ".laser";
                            top_door.Write(out_name.c_str(), false);*/

                            /// storing minimum enclosing 3d rectangle per door
/*                            LineTopology rect_topoline;
                            double height = struct_element_c * vox_length;
                            threedbbox(corners, rect_topoline, top_door_lp, height);
                            minrectangle_topolines.push_back(rect_topoline);*/
                            /// erasing current top_door points
                            top_door_lp.ErasePoints();
                        }
                        /// segment and cluster points belong to the same cluster of centers
                        if ((abs(i - i_previous) <= 1) || (abs(j - j_previous <= 1))){
                            top_center_points.SetAttribute(SegmentNumberTag, door_count);
                            top_center_lp.SetAttribute(SegmentNumberTag, door_count);
                            /// add door center to top_points
                            doors_centers.push_back(vox_cnts[ijk_inx]);
                        }
                        i_previous = i, j_previous = j;
                        //top_door.AddPoints(top_center_points); // single top_door
                        top_doors.AddPoints(top_center_points); // all top_doors
                        /// to store laserpoints inside the voxels
                        top_door_lp.AddPoints(top_center_lp);  // single door laserpoints inside the voxels
                        top_doors_lp.AddPoints(top_door_lp); // all top_doors laserpoints inside the voxels
                    }
                }
            }  //k

    /// count doors clusters
    printf("door_cnt= %d \n", door_count);

    doors_centers.Write("D://test//morph//result//doors_centers.laser", false);
    top_doors_lp.Write("D://test//morph//result//top_doors_lp.laser", false);
    closed_doors_cnt.Write("D://test//morph//result//closed_doors.laser", false);

    /// storing minimum enclosing 3d rectangle per door
    LineTopology rect_topoline;
    double height = struct_element_c;
    threedbbox(corners, rect_topoline, top_doors_lp, height);
    //corners.Write("D://test//morph//result//minrectangle_objpts.objpts");
    //minrectangle_topolines.Write("D://test//morph//result//minrectangle_topologies.top", false);

   return top_doors;
}


// ************************************************** Door detection-cylinder ********************************************************//
// *******************************************************************************************************************//
LaserPoints LaserVoxel::structure_template_cylinder(LaserPoints vox_cnts, LaserPoints traj_points, uint min,
                                                    double struct_element_a,
                                                    double struct_element_b, double struct_element_c,
                                                    vector<vector<vector<int> > > vec_ijk) {

/*   One Voxels is eligible for door nbr-hood search if :
     1. Its nbr-hood voxels are empty/void; 2. If it is close to the trajectory; 3. If on top border there are points.
     The nbr-hood window is hard-coded in LaserVoxel::void_neighbourhood method, it can be also
     be defined as input parameter. Door nbr-hood search windows is user defined.
    */


    LaserPoints doors_points, doors_centroid, doors_pc, closed_doors_points, doors_points_fals_pos;
    std::ostringstream sstream;
    /// n,m,r are the numbers of voxels that window covers in x,y,z dimensions
    uint n = floor(struct_element_a / vox_length); /// n is the number of voxels the a-dimension of window covers
    uint m = floor(struct_element_b / vox_length);
    uint r = floor(struct_element_c / vox_length);

    /// respectively aligned with X,Y,Z axis
    /// number of voxel counting for neighbors before and after current voxel
    int step_a = (n - 1) / 2; // n-1 because we exclude i-th index
    int step_b = (m - 1) / 2;
    int step_c = (r - 1) / 2;

    // TODO change the upper limit for K value, to avoid storing points above the ceiling
    // TODO store the result every two k for different slices of Z value

    KNNFinder<LaserPoint> finder_traj(traj_points);

    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = step_c;
                 k < step_c + 3; k++) {  /// +3 becasue we search above the door center and below ceiling
                printf("\r in reading: i=%d, j=%d, k=%d", i, j, k); /// debugger
                fflush(stdout);

                LaserPoints door_points, door;
                uint top_door_counter = 0;
                bool false_door_counter = 0;
                bool vox_trajhoods = 0, vox_voidhoods = 0, vox_tophoods = 0, vox_occupiedhoods = 0;
                int ijk_inx = vec_ijk[i][j][k];
                int void_hood_window = 9;

                /// create a cylinder for current ijk
                /// we need p1, p2 for axis and radius
                LaserPoint p1, p2;
                double cyl_radius = step_a * vox_length;  // or step_b they should be equal
                double cyl_height = struct_element_c;
                p1 = voxel_center(i, j, k - step_c);
                p2 = voxel_center(i, j, k + step_c);

                Position3D p1_pos, p2_pos;
                Line3D cyl_axis;
                double cyl_axis_len;
                p1_pos = Position3D(p1);
                p2_pos = Position3D(p2);

                /// calculate cylinder axis
                cyl_axis = Line3D(p1_pos, p2_pos);
                //cyl_axis_len = p1_pos.Distance(p2_pos);  // should be equal to structurelelement c


                ///Only return maximum distance
                double vox_to_traj_dist = finder_traj.FindDistance(vox_cnts[ijk_inx], 1, EPS_DEFAULT);

                /// check if there is a close by trajectory
                if (vox_to_traj_dist <= 2 * vox_length)
                    vox_trajhoods = 1;

                /// check if current voxel has less than N occupied neighbors (for open doors),
                int nr_of_occupiedhoods = this->void_neighbourhood(min, i, j, k, void_hood_window);
                if (nr_of_occupiedhoods <= 5)  /// ??? being void indicator
                    vox_voidhoods = 1;

                /// check if current voxel has some points on top-door
                int nr_of_topdhoods = this->void_top(min, i, j, k, step_c);
                if (nr_of_topdhoods >= 5)  ///
                    vox_tophoods = 1;

                ///check if voxel-ijk is eligible for door nbr-hood search
                if (vox_voidhoods && vox_trajhoods && vox_tophoods) {
                    //if (vox_voidhoods) {
                    int a = 0, b = 0, c = 0;
                    //uint labeltag1_cnt = 0;
                    uint labeltag0_cnt = 0;
/*                    ofstream abc_file;
                    abc_file.open ("abc.txt");
                    abc_file << "ijk \n";
                    abc_file << i << j << k << endl;
                    abc_file << "abc \n";*/

                    for (a = i - step_a; a <= (i + step_a); a++) {
                        if (a < 0) continue;
                        if (a >= int(vox_num_X))
                            continue;
                        for (b = j - step_b; b <= (j + step_b); b++) {
                            if (b < 0) continue;
                            if (b >= int(vox_num_Y))
                                continue;
                            for (c = k - step_c; c <= (k + step_c); c++) {
                                if (c < 0) continue;
                                if (c >= int(vox_num_Z)) continue;
                                if (a == i && b == j && c == k) continue; //the point itself is not counted
                                //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger
                                int abc_inx = vec_ijk[a][b][c];
                                int abc_labeltag = vox_cnts[abc_inx].Attribute(LabelTag);
                                LaserPoint testpoint = vox_cnts[abc_inx];
                                Position3D testpoint_pos = Position3D(testpoint);
                                double axisdist_to_testp = cyl_axis.DistanceToPoint(testpoint_pos);
                                //double dist_diif = std::abs(axisdist_to_testp - cyl_radius); // debugger
                                //printf ("dist_diff %f \n", dist_diif);

                                /// check if the vox_cnt(abc) is on the cyl shell
                                if (abs(axisdist_to_testp - cyl_radius) <= 0.02) {
                                    if (abc_labeltag == 1) {
                                        //printf ("dist_diff %f \n", dist_diif);
                                        /// saving abc to a file
                                        //abc_file << a << b << c << endl;
                                        door_points.push_back(vox_cnts[abc_inx]);
                                        //TODO save points inside the voxel
                                        door.AddPoints(this->export_voxelpnts(a, b, c));
                                    }
                                }

                                /// storing door top border
                                if (c == k + step_c) {
                                    if (abc_labeltag == 1) {
                                        door_points.push_back(vox_cnts[abc_inx]);
                                        door.AddPoints(this->export_voxelpnts(a, b, c));
                                        // TODO check this is effective after void_top implementation
                                        top_door_counter++; /// counts nr of points on top of the door edge
                                    }
                                }
                                //abc_file.close();
                            }//c
                        }//b
                    }//a

                    /// write door_centroids in a laserfile
                    doors_centroid.push_back(vox_cnts[ijk_inx]);


                    /// write valid door_points in individual laserfile, input parameter can be sum up of structure elements

                    if ((door_points.size() >= 60)
                        && (top_door_counter >= (n - 2))) {
                        //&&  !false_door_counter) {

                        string out_name;
                        sstream.str(""); //clear stream
                        sstream << i << "-" << j << "-" << k;
                        string ijk_str = sstream.str();
                        out_name = "D://test//morph//result//door_points_" + ijk_str + ".laser";

                        door_points.push_back(vox_cnts[ijk_inx]); /// add door_centroid to door_points
                        door.push_back(vox_cnts[ijk_inx]);  /// single door
                        //cout << endl;
                        //door_points.Write(out_name.c_str(), false); //debugger

                        doors_points.AddPoints(door_points); // save door_points to a laserfile
                        doors_pc.AddPoints(door);
                    }
                }
            }

    doors_centroid.Write("D://test//morph//result//doors_centroids.laser", false);
    doors_pc.Write("D://test//morph//result//doors_pc.laser", false);
    return doors_points;
}


// ************************************************** CLOSED - Door Detection ********************************************************//
// *******************************************************************************************************************//
/*LaserPoints LaserVoxel::closed_door(LaserPoints vox_cnts, LaserPoints traj_points, uint min,
                                                    double struct_element_a,
                                                    double struct_element_b, double struct_element_c,
                                                    vector<vector<vector<int> > > vec_ijk) {

    LaserPoints doors_centroid, closed_doors_points, closed_doors_pc;
    std::ostringstream sstream;
    /// n,m,r are the numbers of voxels that window covers in x,y,z dimensions
    uint n = floor(struct_element_a / vox_length); /// n is the number of voxels the a-dimension of window covers
    uint m = floor(struct_element_b / vox_length);
    uint r = floor(struct_element_c / vox_length);

    /// respectively aligned with X,Y,Z axis
    /// number of voxel counting for neighbors before and after current voxel
    int step_a = (n - 1) / 2; // n-1 because we exclude i-th index
    int step_b = (m - 1) / 2;
    int step_c = (r - 1) / 2;

    KNNFinder<LaserPoint> finder_traj(traj_points);

    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = step_c;
                 k < step_c + 3; k++) {  /// +3 becasue we search above the door center and below ceiling
                printf("\r in reading: i=%d, j=%d, k=%d", i, j, k); /// debugger
                fflush(stdout);

                LaserPoints closed_door_points, closed_door_pc;
                uint top_door_counter = 0;
                bool vox_trajhoods = 0, vox_voidhoods = 0, vox_tophoods = 0, vox_occupiedhoods = 0;
                int ijk_inx = vec_ijk[i][j][k];
                int void_hood_window = 9;

                ///Only return maximum distance
                double vox_to_traj_dist = finder_traj.FindDistance(vox_cnts[ijk_inx], 1, EPS_DEFAULT);

                /// check if there is a close by trajectory
                if (vox_to_traj_dist <= vox_length)
                    vox_trajhoods = 1;

                /// nr of occupied voxel-hood
                int nr_of_occupiedhoods = this->void_neighbourhood(min, i, j, k, void_hood_window);

                /// check if cuurent voxel has more than N occupied voxels (for closed doors)
                if (nr_of_occupiedhoods >= void_hood_window * void_hood_window - 1)
                    vox_occupiedhoods = 1;

                /// check if current voxel has some points on top-door
                int nr_of_topdhoods = this->void_top(min, i, j, k, step_c);
                if (nr_of_topdhoods >= 5)  ///
                    vox_tophoods = 1;

                /// store closed doors
                if (vox_occupiedhoods && vox_trajhoods && vox_tophoods) {
                    //uint labeltag1_cnt = 0;
                    uint labeltag0_cnt = 0;
                    for (int a = i - step_a; a <= (i + step_a); a++) {
                        if (a < 0) continue;
                        if (a >= int(vox_num_X))
                            continue;
                        for (int b = j - step_b; b <= (j + step_b); b++) {
                            if (b < 0) continue;
                            if (b >= int(vox_num_Y))
                                continue;
                            for (int c = k - step_c; c <= (k + step_c); c++) {
                                if (c < 0) continue;
                                if (c >= int(vox_num_Z)) continue;
                                if (a == i && b == j && c == k) continue; //the point itself is not counted
                                //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger
                                int abc_inx = vec_ijk[a][b][c];
                                int abc_labeltag = vox_cnts[abc_inx].Attribute(LabelTag);
                                LaserPoint testpoint = vox_cnts[abc_inx];
                                Position3D testpoint_pos = Position3D(testpoint);
                                double axisdist_to_testp = cyl_axis.DistanceToPoint(testpoint_pos);
                                //double dist_diif = std::abs(axisdist_to_testp - cyl_radius); // debugger
                                //printf ("dist_diff %f \n", dist_diif);

                                /// check if the vox_cnt(abc) is on the cyl shell
                                if (abs(axisdist_to_testp - cyl_radius) <= 0.02) {
                                    if (abc_labeltag == 1) {
                                        //printf ("dist_diff %f \n", dist_diif);
                                        /// saving abc to a file
                                        //abc_file << a << b << c << endl;
                                        door_points.push_back(vox_cnts[abc_inx]);
                                        //TODO save points inside the voxel
                                        door.AddPoints(this->export_voxelpnts(a, b, c));
                                    }
                                }

                                /// storing door top border
                                if (c == k + step_c) {
                                    if (abc_labeltag == 1) {
                                        door_points.push_back(vox_cnts[abc_inx]);
                                        door.AddPoints(this->export_voxelpnts(a, b, c));
                                        // TODO check this is effective after void_top implementation
                                        top_door_counter++; /// counts nr of points on top of the door edge
                                    }
                                }
                                //abc_file.close();
                            }//c
                        }//b
                    }//a

                    closed_door_points.push_back(vox_cnts[ijk_inx]);  // add door center to the door_voxels
                    closed_doors_points.AddPoints(closed_door_points); // save closed_doors voxels to a laserfile
                    closed_doors_pc.AddPoints(closed_door_pc); // doors pointclouds
                    //closed_doors_points.Write("D://test//morph//result//closed_doors_points_tmp.laser", false); //debugger
                }
            }

    doors_centroid.Write("D://test//morph//result//closed_doors_centroids.laser", false);
    closed_doors_points.Write("D://test//morph//result//closed_doors_points.laser", false);
    closed_doors_pc.Write("D://test//morph//result//closed-doors_pc.laser", false);
    return closed_doors_points;
}*/

// ************************************************** Door detection- box ********************************************************//
// *******************************************************************************************************************//
LaserPoints
LaserVoxel::structure_template(LaserPoints vox_cnts, LaserPoints traj_points, uint min, double struct_element_a,
                               double struct_element_b, double struct_element_c,
                               vector<vector<vector<int> > > vec_ijk) {

/*   One Voxels is eligible for door nbr-hood search if :
     1. Its nbr-hood voxels are empty/void; 2. If it is close to the trajectory
     the nbr-hood window is hard-coded in LaserVoxel::void_neighbourhood method, it can be also
     be defined as input parameter. Door nbr-hood search windows is user defined.
    */


    LaserPoints doors_points, doors_centroid, doors_points_fals_pos;
    std::ostringstream sstream;
    /// n,m,r are the numbers of voxels that window covers in x,y,z dimensions
    uint n = floor(struct_element_a / vox_length); /// n is the number of voxels the a-dimension of window covers
    uint m = floor(struct_element_b / vox_length);
    uint r = floor(struct_element_c / vox_length);

    /// respectively aligned with X,Y,Z axis
    /// number of voxel counting for neighbors before and after current voxel
    int step_a = (n - 1) / 2; // n-1 because we exclude i-th index
    int step_b = (m - 1) / 2;
    int step_c = (r - 1) / 2;

    // TODO change the upper limit for K value, to avoid storing points above the ceiling
    // TODO store the result every two k for different slices of Z value

    KNNFinder<LaserPoint> finder_traj(traj_points);

    for (int i = 0; i < int(vox_num_X); i++)
        for (int j = 0; j < int(vox_num_Y); j++)
            for (int k = step_c;
                 k < int(vox_num_Z - step_c); k++) {  /// -step_c becasue we search below the ceiling and door top
                printf("\r in reading: i=%d, j=%d, k=%d", i, j, k); /// debugger
                fflush(stdout);

                LaserPoints door_points, door_points_false_pos;
                uint top_door_counter = 0;
                bool false_door_counter = 0;

                bool vox_trajhoods = 0, vox_voidhoods = 0, vox_tophoods = 0;

                int ijk_inx = vec_ijk[i][j][k];
                ///Only return maximum distance
                double vox_to_traj_dist = finder_traj.FindDistance(vox_cnts[ijk_inx], 1, EPS_DEFAULT);
                /// check if there is a close by trajectory
                if (vox_to_traj_dist <= 2 * vox_length)
                    vox_trajhoods = 1;

                /// check if current voxel has less than n occupied neighbors,
                int nr_of_occupiedhoods = this->void_neighbourhood(min, i, j, k, 9);
                if (nr_of_occupiedhoods <= 10)  /// ??? being void indicator
                    vox_voidhoods = 1;

                /// check if current voxel has some points on top-door
                int nr_of_topdhoods = this->void_top(min, i, j, k, step_c);
                if (nr_of_topdhoods >= 5)  ///
                    vox_tophoods = 1;

                //TODO write the result just with one condition and check
                ///check if voxel-ijk is eligible for door nbr-hood search
                if (vox_voidhoods && vox_trajhoods && vox_tophoods) {
                    //if (vox_voidhoods) {
                    int a = 0, b = 0, c = 0;
                    //uint labeltag1_cnt = 0;
                    uint labeltag0_cnt = 0;

                    for (a = i - step_a; a <= (i + step_a); a++) {
                        if (a < 0) continue;
                        if (a >= int(vox_num_X))
                            continue;
                        for (b = j - step_b; b <= (j + step_b); b++) {
                            if (b < 0) continue;
                            if (b >= int(vox_num_Y))
                                continue;
                            for (c = k - step_c; c <= (k + step_c); c++) {
                                if (c < 0) continue;
                                if (c >= int(vox_num_Z)) continue;
                                if (a == i && b == j && c == k) continue; //the point itself is not counted
                                //printf(" in reading: a=%d, b=%d, c=%d\n",a,b,c); /// debugger
                                int abc_inx = vec_ijk[a][b][c];
                                int abc_labeltag = vox_cnts[abc_inx].Attribute(LabelTag);
                                //if (abc_labeltag ==0) labeltag0_cnt++;

                                /// check door wall
/*                                if (c < (k+step_c-2) &&  (j-step_b-2)< b && b < (j+step_b-2)){
                                    if (abc_labeltag ==1){
                                        door_points_false_pos.push_back(vox_cnts[abc_inx]);
                                    }
                                }*/

                                /// storing door top border
                                if (c == k + step_c) {
                                    if (abc_labeltag == 1) {
                                        door_points.push_back(vox_cnts[abc_inx]);
                                        top_door_counter++; /// counts nr of points on top of the door edge
                                    }
                                }

                                /// indicating just doors in Y direction
                                if ((b == j - step_b) || (b == j + step_b)) {
                                    if (abc_labeltag == 1) {
                                        door_points.push_back(vox_cnts[abc_inx]);
                                    }
                                }
                                /// indicating just doors in X direction
                                if ((a == i - step_a) || (a == i + step_a)) {
                                    if (abc_labeltag == 1) {
                                        door_points.push_back(vox_cnts[abc_inx]);
                                    }
                                }

                            }//c
                        }//b
                    }//a

                    /// write door_centroids in a laserfile
                    doors_centroid.push_back(vox_cnts[ijk_inx]);

                    /// check for false-detected doors
/*                    int false_pos_points = n*m;
                    if (door_points_false_pos.size() >= false_pos_points){
                        door_points_false_pos.push_back(vox_cnts[ijk_inx]);
                        door_points_false_pos.Write("D://test//morph//result//door_points_false_pos.laser", false);
                        doors_points_fals_pos.AddPoints(door_points_false_pos);
                        false_door_counter =1; /// counts nr of points below the top of the door edge
                    }*/

                    /// write valid door_points in individual laserfile, input parameter can be sum up of structure elements
                    /// NOTE: bigger struct-element cause this condition to be true faster, consequently the result is more door-templates
                    /// also check if there are points on top of the door-template border
                    if ((door_points.size() >= 53)
                        && (top_door_counter >= (n - 2))) {
                        //&&  !false_door_counter) {

                        string out_name;
                        sstream.str(""); //clear stream
                        sstream << i << "-" << j << "-" << k;
                        string ijk_str = sstream.str();
                        out_name = "D://test//morph//result//door_points_" + ijk_str + ".laser";

                        door_points.push_back(vox_cnts[ijk_inx]); /// add door_centroid to door_points
                        //cout << endl;
                        //door_points.Write(out_name.c_str(), false); //debugger

                        doors_points.AddPoints(door_points); // save door_points to a laserfile
                    }
                }
            }

    doors_centroid.Write("D://test//morph//result//door_centroids.laser", false);
    //doors_points_fals_pos.Write("D://test//morph//result//doors_points_fals_pos.laser", false);
    return doors_points;
}


vector<vector<vector<LaserPoints *> > > &LaserVoxel::getVox() { return Vox; };

void LaserVoxel::setNrOfPoints(int nr) {
    this->nrOfPoints = nr;
}

void LaserVoxel::setTag(int t) {
    this->tag = t;
}

int LaserVoxel::getNrOfPoints() { return nrOfPoints; }

int LaserVoxel::getTag() { return tag; }

void LaserVoxel::setAttributes(int min_points_in_vox) {
    LaserPoints laserpoints;
    vector<LaserVoxel> voxels;
    ///initialize the LaserVoxel
    double vox_l = 0.10;
    LaserVoxel vox(laserpoints, vox_l);
    size_t found_points_in_vox = 0;

    for (uint i = 0; i < vox_num_X; i++)
        for (uint j = 0; j < vox_num_Y; j++)
            for (uint k = 0; k < vox_num_Z; k++) {
                if (Vox[i][j][k] != NULL) {
                    found_points_in_vox = Vox[i][j][k]->LaserPointsReference().size();
                    if (found_points_in_vox >= min_points_in_vox) {
                        vox.setTag(1);
                        vox.setNrOfPoints(int(found_points_in_vox));
                        voxels.push_back(vox);
                    }
                } else {
                    vox.setTag(0);
                    vox.setNrOfPoints(int(found_points_in_vox));
                    voxels.push_back(vox);
                }
            }
    //cout << vox.getTag() << endl;
    //return v;
}
