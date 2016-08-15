#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <iostream>
#include <fstream>
#include <list>
#include <CGAL/IO/io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <boost/iterator/transform_iterator.hpp>
#include <vector>
#include <cmath>
#include <boost/bimap.hpp>
//#include "DataInput.h"
#define PI 3.1415926
#define ANGLE PI*0.125

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



class PointCloud {
public:
        int pc; 
        typedef std::pair<Point,unsigned> Ver;
        typedef std::pair<Point, int> Pole;
        typedef std::pair<Pole, Pole> TwoPoles;

        typedef std::pair<Vertex_handle, TwoPoles> AmentaNeedle;

        PointCloud();
        PointCloud(std::string file_path) ;
	PointCloud(std::vector<Point> *knn);

        void delaunay() {D.insert(data.begin(), data.end());};

        bool is_on_convex_hull_vertex (Vertex_handle v);
	bool is_on_convex_hull_facet (Facet f);
        void voronoi_cell_vertices(Vertex_handle v, std::list<Point> &stars);
        void amenta_needle(Vertex_handle v, AmentaNeedle &needle);
        void all_amenta_needles();
        void output_poles();
	void facet_filter();
	bool is_candidate_facet(Facet f);
	int check_coco_angle(AmentaNeedle &needle,Point f_vorn1,Point f_vorn2);

	
	void output_facet();
	Point facet_normal(Point p1, Point p2, Point p3);
	Point facet_normal_orient(Point n, Facet face);

        std::vector<Facet> face_list;

      
private:
	
        std::list<Ver> data;
        Delaunay D;
        std::vector<AmentaNeedle> needles;

};


#endif
