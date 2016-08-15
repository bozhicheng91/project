#ifndef DATA_INPUT_H
#define DATA_INPUT_H

#include <iostream>
#include <fstream>
#include <list>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>



#include <CGAL/IO/io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <boost/iterator/transform_iterator.hpp>
#include <vector>
#include <cmath>


//bimap库相关头文件
#include <boost/bimap/bimap.hpp>  
#include <boost/bimap/multiset_of.hpp>  
      

#define PI 3.1415926
#define ANGLE PI*0.125

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Point_3<K> Point;

typedef CGAL::Search_traits_3<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Point_3<K> Point;
typedef CGAL::Plane_3<K> Plane;
typedef CGAL::Vector_3<K> Vector;
typedef CGAL::Line_3<K> Line;

typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb> Tds;
typedef CGAL::Delaunay_triangulation_3<K,Tds,CGAL::Fast_location> Delaunay;
typedef Delaunay::Vertex_handle  Vertex_handle;
typedef Delaunay::Cell_handle    Cell_handle;
typedef Delaunay::Facet          Facet;
typedef Delaunay::Triangle       Triangle;



using namespace boost::bimaps;
typedef std::pair<int,int>Edge;
typedef std::tuple<int,int,int>Face;
typedef std::pair<int,int>e_id;
     
typedef std::vector<int> Data_list;

typedef std::vector<e_id> e_list;

typedef struct FaceData{

     Face *face ;
     Vector *normal;
   
     Data_list *edges;
     int id;
     int flag;
     int tag;
     int state;
     double ang;

}FaceData;


typedef struct EdgeData{

     Edge *edge;
     Data_list *faces;
     
     int id;
     int flag;

}EdgeData;


typedef struct VertexData{

     Point *point;
     Data_list *edges;
     Data_list *faces;
     
     int id ;
     int flag ;
     int tag ;
     int state ;

     int l_tag ;
     int l_flag ;
     int l_state ;

}VertexData;



struct _point {};
struct _pointdata {};
struct _face {};
struct _facedata{};
struct _edge{};
struct _edgedata{};

typedef bimap<
tagged<Point,_point>,
     tagged<VertexData*,_pointdata>
     > Point_input;
typedef bimap<  
tagged< Edge, _edge > ,  
     tagged<EdgeData*,_edgedata>
     > Edge_input;
typedef bimap<  
tagged< Face, _face >,  
     tagged< FaceData*,_facedata >  
     > Face_input;
	
typedef Point_input::value_type datain;
typedef Edge_input::value_type edgein;
typedef Face_input::value_type facein;
//

FaceData *new_FaceData();

EdgeData *new_EdgeData();

VertexData *new_VertexData();

void coco_facedata_free(std::vector<FaceData*> *face_list);
void coco_edgedata_free(std::vector<EdgeData*> *edge_list);
void coco_pointdata_free(std::vector<VertexData*> *point_list);

void coco_facedataCopy(FaceData* target, FaceData* src);

#endif
