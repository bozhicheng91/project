#ifndef COCO_H
#define COCO_H
#include "DataInput.h"
#include "PointCloud.h"
#include <algorithm>

using namespace std;


void
coco_facet_to_face (std::vector<Facet> *facet_list,std::vector<FaceData*> *face_list, Point_input *Pointmap);

VertexData *
coco_get_pointdata(Point point, Point_input* Pointmap);

void
coco_sort_triangle_point(int &id0, int &id1, int &id2);

void coco_association_analysis(vector<FaceData*> *face_list,
                               vector<EdgeData*>*edge_list,
                               vector<VertexData*>*point_list,
                               Point_input *pointmap,
                               Edge_input *edgemap,
                               Face_input *facemap);

void coco_association_analysis_edge( Edge *edge,
                                     FaceData* fd,
                                     vector<VertexData*>*point_list,
                                     vector<EdgeData*>*edge_list,
                                     Edge_input *edgemap);


Point coco_facet_normal(Point p1, Point p2, Point p3);


void
coco_facelist_print(vector<VertexData*> *point_list, vector<FaceData*> *face_list);

void
coco_pointlist_print(vector<VertexData*> *point_list);

void
coco_edgelist_print(vector<EdgeData*> *edge_list);



void coco_extract_manifold(vector<VertexData*> *point_list,
                           vector<EdgeData*> *edge_list,
                           vector<FaceData*> *face_list,
                           Point_input *pointmap,
                           Edge_input *edgemap,
                           Face_input *facemap);

void coco_mark_face(VertexData* v, vector<VertexData*> *point_list, vector<EdgeData*> *edge_list,vector<FaceData*> *face_list, Point_input *pointmap,Edge_input *edgemap,Face_input *facemap);

double coco_calculate_ang(Vector n1, Vector n2);
double coco_calculate_ang_b(Vector n1, Vector n2);
void coco_face_add(Face face,vector<FaceData*> *face_list);




void coco_mark_single_face(vector<FaceData*> *face_list,
                               vector<EdgeData*>*edge_list,
                               vector<VertexData*>*point_list,
                               Point_input *pointmap,
                               Edge_input *edgemap,
                               Face_input *facemap);



void coco_mark_relate_face(vector<int>*id_list,vector<FaceData*> *face_list,vector<VertexData*>*point_list, Point_input *pointmap,Face_input
                           *facemap);

bool Comp(const int &a,const int &b);


void coco_coco_association_analysis_2th(vector<FaceData*> *face_list,
                                        vector<VertexData*>*point_list,
                                        Point_input *pointmap);

#endif
