#ifndef BORDER_FACET_SELECTION_H
#define BORDER_FACET_SELECTION_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <glib/gprintf.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_multifit.h>
#include "PointCloud.h"
#include "DataInput.h"
#include "coco.h"



using namespace std;


void facet_selection(vector<FaceData*> *face_list_in,vector<FaceData*> *face_list_out, vector<VertexData*> *point_list,Point_input *pointmap);



void coco_facelist_print_multi(int i, vector<VertexData*> *point_list, vector<FaceData*> *face_list);

void coco_vertexdata_reset_mark(vector<VertexData*> *vl);

bool coco_border_extend_point_test(Point p, Point_input *pointmap);
void coco_vertexdata_mark__border_near(vector<VertexData*> *vl);

void coco_border_inner_mark(vector<Point> *pl, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap);




#endif
