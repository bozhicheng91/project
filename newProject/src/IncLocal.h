#ifndef INCLOCAL_H
#define INCLOCAL_H
#include "test_ring_extending_coco.h"
#include "IncTestFunc.h"
//初始网格重建

void IncLocal_initialMesh(Point p,
                          int k,
                          Tree *kd_tree,
                          Point_input *pointmap,
                          vector<VertexData*> *point_lis);
//实现点集间的转化。
void IncLocal_pointTOvertexdata(vector<Point>*pl,
                                vector<VertexData*>* vl,
                                Point_input *pointmap );


//将输入点集标记为重建点
void IncLocal_markReconstructionVertex(vector<VertexData*> *vl);


//辅助点添加
void IncLocal_addAuxiliaryPoints(vector<Point>*pointInput,
                                 vector<Point>*pointOutput,
                                 Tree *kd_tree,
                                 Point_input *pointmap);




/*获取点对应的点结点信息*/
VertexData *IncLocal_getVertexData(Point point, Point_input *Pointmap);

/*获取边对于的边结点信息*/
EdgeData *IncLocal_getEdgeData(Edge edge, Edge_input *edgemap);

/*获取面片对应的面片结点信息*/
FaceData *IncLocal_getFaceData(Face face, Face_input *facemap);



/*局部点集cocone重建，重建后将cgal中的facet数据结构转变为自定义的facedata数据结构*/
void IncLocal_coconeReconstruction(vector<Point>*pointInput, vector<FaceData*>*facelistOutput, Point_input *pointmap);


void IncLocal_facetSelection(vector<FaceData*> *facelist, vector<VertexData*> *pointlist, Point_input *pointmap);

/*对面，边进行相关性解析*/
void IncLocal_association_analysis(vector<FaceData*> *face_list,
                               vector<EdgeData*>*edge_list,
                               vector<VertexData*>*point_list,
                               Point_input *pointmap,
                               Edge_input *edgemap,
                                   Face_input *facemap);



/*对边进行解析*/
void IncLocal_association_analysis_edge( Edge *edge,
                                     FaceData* fd,
                                     vector<VertexData*>*point_list,
                                     vector<EdgeData*>*edge_list,
                                         Edge_input *edgemap);



/*流形提取操作*/
void IncLocal_extractManifold(vector<VertexData*> *point_list,
                           vector<EdgeData*> *edge_list,
                           vector<FaceData*> *face_list,
                           Point_input *pointmap,
                           Edge_input *edgemap,
                              Face_input *facemap);





/*对点v的映射面片进行标记，并重新生成点v的映射面片*/
void IncLocal_extractFacet(VertexData* v, 
                           vector<VertexData*> *point_list, 
                           vector<EdgeData*> *edge_list, 
                           vector<FaceData*> *face_list,
                           Point_input *pointmap,  
                           Edge_input *edgemap, 
                           Face_input *facemap);






/*对面链表中由输入样点构成的面片进行标记*/
void IncLocal_markFace_accidentToVertex(
                       vector<int>*id_list,
                       vector<FaceData*> *face_list,
                       vector<VertexData*>*point_list,
                       Point_input *pointmap,
                       Face_input *facemap);



/*比较函数，判断两个int值的大小*/
bool IncLocal_Comp(const int &a,const int &b);



//计算面片的法向
Point IncLocal_facetNormal(Point p1, Point p2, Point p3);


/*计算两个向量之间夹角*/
double IncLocal_calculateAngle(Vector n1, Vector n2);



/*将面片附加到面片链表*/
void IncLocal_faceAddToList(Face face,vector<FaceData*> *face_list);



/*提取波前环*/
void IncLocal_frontExtract(vector<EdgeData*> *edgeList, 
                           vector<VertexData*> frontList);

/*对波前边排序*/
void IncLocal_sortFrontEdge(vector<EdgeData*> *frontEdge);
/*比较两条边的信息，若两边不同，返回０，若存在一个共同顶点，返回１，存在两个共同顶点，返回２*/
int IncLocal_compEdgeData(EdgeData* firstEdge, EdgeData* secondEdge);


#endif
