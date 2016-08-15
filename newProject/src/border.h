#ifndef BORDER_H
#define BORDER_H


#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_multifit.h>
#include "PointCloud.h"
#include "DataInput.h"
#include "coco.h"
#include "border_facet_selection.h"
#include "test_ring_extending_coco.h"


//#include "local.h"
using namespace std;

void
coco_border_kboundary_extract_boundary (vector<Point> *boundary, vector<Point> *pl, guint k, gdouble e, Tree *kd_tree);

void
coco_border_kboundary_extract_boundary (vector<Point>* boundary, vector<VertexData*> *pl, guint k, gdouble e, Tree *kd_tree);


Point coco_border_get_square_plane(vector <Point> *kn);


void coco_border_get_rejection_points ( vector<Point> *rpoints, vector<Point> *kn, Point func);


gboolean is_boundary (Point point, vector<Point> *neighbors, gdouble e);


void selection_sort (vector<gdouble>* angle);


double coco_border_calculate_ang_b(Vector n1, Vector n2);


void coco_point_save_to_file(vector<Point>* pl);

void coco_point_save_to_file_2th(vector<Point>* pl);

void
coco_border_sort_neighbors_angle(vector<double> *ang_l, vector<Point>*kn);

void coco_pl_to_vl(vector<Point>*pl, vector<VertexData*>* vl, Point_input *pointmap);

void coco_mark_vertexdata_recon(vector<VertexData*>* vl);

void coco_border_extend_for_assure(vector<Point> *pl, vector<Point> *pl_extend, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap);

void coco_border_extend_for_recons(vector<Point> *pl, vector<Point> *pl_extend, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap);
#endif
