#ifndef RING_EXTENDING_COCO_H
#define RING_EXTENDING_COCO_H

#include <ctime>

#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <glib/gprintf.h>
#include <sstream>


#include "PointCloud.h"
#include "DataInput.h"
#include "coco.h"
#include "border.h"
#include "border_facet_selection.h"

void ring_extending_file_input(Point *p_tmp, string file_path, Tree *kd_tree, Point_input *Pointmap, vector<VertexData*>
*point_list);


void ring_extending_initial_surface(Point p, int k, Tree *kd_tree, double e, Point_input *Pointmap,
vector<Point>*border, vector<VertexData*> *point_list);


void ring_extending_adding(vector<Point> *border,
                           Tree *kd_tree,
                           vector<VertexData*>* point_list,
                           Point_input *Pointmap,
                           double e,
                           int loop);



void ring_extending_advancing_adding(vector<Point> *border,
                                     Tree *kd_tree,
                                     vector<VertexData*>* point_list,
                                     Point_input *pointmap,
                                     double e,
                                     int loop);



//获取波前扩展的待重建点
void ring_extending_get_reconstruction_data(Point p, vector<Point>*pl_recons,  vector<Point> *pl_extend, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap);


void ring_extending_point_reconstruction(vector<Point>*recons_pl,
                                         Tree *kd_tree,
                                         vector<VertexData*>* point_list,
                                         Point_input *pointmap,
                                         double e,
                                         int times);


#endif
