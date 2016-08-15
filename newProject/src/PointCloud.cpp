#include "PointCloud.h"

using namespace std;


PointCloud::PointCloud(string file_path)
{
  
        ifstream in(file_path);
        if (!in) {
                cerr << "error: unable to open input file: "
                     << in
                     << endl;
        }
        string line, word;
        double x, y, z;
	int i = 0;
        while (getline(in, line)) {
	        istringstream stream(line);
	
		stream >> x >> y >> z;
                data.push_back(std::make_pair(Point(x, y, z),i++));
        }
        in.close();
}

PointCloud::PointCloud(std::vector<Point> *knn)
{
     
        int i = 0;
        if (knn->size() == 0){

	  cerr << "error: the input point list is NULL: "
	       << endl;

	}
	else{

	      for (vector<Point>::iterator it = knn->begin(); it != knn->end(); it++){
		Point p = *it;
		data.push_back(std::make_pair(p,i++));

	  }

	}
}


bool PointCloud::is_on_convex_hull_vertex(Vertex_handle v)
{
        bool res = false;
        list<Cell_handle> cells;
        D.incident_cells(v, back_inserter(cells));
        for (list<Cell_handle>::iterator it = cells.begin(); it != cells.end(); it++) {
                if (D.is_infinite(*it)) {
                        res = true;
                        break;
                }
        }
        return res;
}

void PointCloud::voronoi_cell_vertices(Vertex_handle v, list<Point> &stars)
{
        list<Delaunay::Cell_handle> cells;
        D.finite_incident_cells(v, back_inserter(cells));

        for(list<Cell_handle>::iterator it = cells.begin(); it != cells.end(); it++) {
                stars.push_back(D.dual(*it));
        }
}

void PointCloud::amenta_needle(Vertex_handle v, AmentaNeedle &needle)
{
        needle.first = v;

        //计算正极点
        list<Point> stars;
        voronoi_cell_vertices(v, stars);
        if (is_on_convex_hull_vertex(v)) {
                Vector norm = CGAL::NULL_VECTOR;
                list<Facet> F;
                D.finite_incident_facets(v, back_inserter(F));
                for (list<Facet>::iterator it = F.begin(); it != F.end(); it++) {
                        Facet f = *it;
                        Facet _f = D.mirror_facet(f);
                        if (D.is_infinite(f.first) || D.is_infinite(_f.first)) {
                                if (D.is_infinite(f.first)) {
                                        f = _f;
                                }
                                Plane s = D.triangle(f).supporting_plane();
                                Cell_handle cell = f.first;
                                Point p = cell->vertex(f.second)->point();
                                Point q = s.projection(p);
                                norm = norm + (q - p);
                        }
                }
                norm = norm / sqrt(norm.squared_length());
                needle.second.first.first = CGAL::ORIGIN + norm;
		//该极点标记为0,表示此时该极点为向量。
                needle.second.first.second = 0;
        } else {
                Point r = v->point();
                auto comp_func = [&](const Point &p, const Point &q){
		  return CGAL::squared_distance(p, r) < CGAL::squared_distance(q, r);
                };
		needle.second.first.first = *max_element(stars.begin(), stars.end(), comp_func);
		//该极点标记为1,表示此时该极点为极点坐标
                needle.second.first.second = 1;
                //cout << needle.second.first.first << endl;
        }

        //计算负极点
        Point pos_pole;
        if (needle.second.first.second == 0) {
                pos_pole = v->point() + (needle.second.first.first - CGAL::ORIGIN);
        } else {
                pos_pole = needle.second.first.first;
        }
	//构造直线，过点v，且方向为pos_pole到v。
        Line L(v->point(), pos_pole - v->point());
        auto comp_func = [&](const Point &p, const Point &q){
                Point _p = L.projection(p);
                Point _q = L.projection(q);
                return CGAL::squared_distance(_p, pos_pole) < CGAL::squared_distance(q, pos_pole);
        };
        needle.second.second.first = *max_element(stars.begin(), stars.end(), comp_func);
        needle.second.second.second = 1;
}


void PointCloud::all_amenta_needles()
{
        Delaunay::Finite_vertices_iterator it;
	int i = 0;
        for (it = D.finite_vertices_begin(); it != D.finite_vertices_end(); it++) {
	        
                AmentaNeedle needle;

		it->info() = i++;
		
		//cout<< it->info() << endl; 
		
                amenta_needle(it, needle);
                needles.push_back(needle);
        }
}

void PointCloud::output_poles()
{


         ofstream  output_pole;
	 output_pole.open("../data/output/out_pole");

	 /*
        auto lambda = [](AmentaNeedle needle){
	  if (needle.second.first.second == 0) {    //当该点位于凸包上时，只有一个极点
	    //output_pole << needle.first->point() <<" " << needle.second.second.first << endl;
	  } else {   //当该点不位于凸包上时，有两个极点
	    //cout << needle.first->point() <<" " << needle.second.first.first << endl;
                }
        };
        for_each(needles.begin(), needles.end(), lambda);
	 */


	 std::vector<AmentaNeedle>::iterator it;
	 for( it = needles.begin(); it != needles.end(); it++){

	       AmentaNeedle needle = *it;
	       Vector norm =  CGAL::NULL_VECTOR;
	       if (needle.second.first.second == 0)
		 {
		   norm = needle.second.second.first-needle.first->point();
		   norm = (-1)*norm / sqrt(norm.squared_length());
		   output_pole << needle.first->point() <<" " << needle.first->point()+norm << endl;

		 }
	       else{
		 norm = needle.second.first.first-needle.first->point();
		 norm = norm / sqrt(norm.squared_length());
		 output_pole << needle.first->point() <<" " << needle.first->point()+norm << endl;

	       }
	  


	}


	
}

//面片过滤
void PointCloud::facet_filter (){

	Delaunay::Finite_facets_iterator it;
	
	for (it = D.finite_facets_begin(); it != D.finite_facets_end(); it++){
	  
	  Facet f = *it;
	  Facet _f = D.mirror_facet(f);
	  

	  if (is_candidate_facet(f))
	    face_list.push_back(f);

	}
}


//判断面片是否位于凸包上
bool PointCloud::is_on_convex_hull_facet(Facet f){

      bool res = false;
      Facet face = f;
      Facet _face = D.mirror_facet(face);
      
      if (D.is_infinite(face.first) || D.is_infinite(_face.first))
	res = true;
      return res;
      
}

//判断是否为候选三角面片
bool PointCloud::is_candidate_facet (Facet f){

      bool res = false;
      Facet face = f;
      Facet _face = D.mirror_facet(face);


      //输出面片的三个顶点
      Cell_handle cell = face.first;
      Point opp_p = cell->vertex(face.second)->point();
      // cout <<"    opp vertex is : " << opp_p << endl;
      // cout << "    vertex of facet is :" << endl;
      int j = 0;
      std::vector<Point> p;
      std::vector<AmentaNeedle> needle_f;
      for (int i = 0; i <= 3; i++)
	    {
	      if(cell->vertex(i) != cell->vertex(face.second))
		{
		     Point p_tmp = cell->vertex(i)->point();
		     p.push_back(p_tmp);
		     int in_d = cell->vertex(i)->info();
		     //cout <<"    " << p_tmp << endl;
		     // cout <<"    "<< in_d << endl;
		     needle_f.push_back(needles[in_d]);
		     //cout <<"    the vertex of needle is : " << needle_f[in_d].first->point() << endl;
		}
	    }
	 
     
      if (is_on_convex_hull_facet(face)) {  //当面片位于凸包上时
        	if (D.is_infinite(face.first))   face = _face;
		   
		Point f_vorn1 = D.dual(face.first);
		//cout << "    facet_voronoi_1  is "<< f_vorn1 << endl;

		Point f_vorn2 = facet_normal(p[0], p[1], p[2]);
		f_vorn2 = facet_normal_orient(f_vorn2, face);
		
		Vector norm = f_vorn2-CGAL::ORIGIN;
		
		double norm_n = sqrt(norm.squared_length());
		
		norm = norm*10000/norm_n;
		f_vorn2 = f_vorn1 + norm;
		
		
		int k1 = check_coco_angle (needle_f[0],f_vorn1,f_vorn2);
		int k2 = check_coco_angle (needle_f[1],f_vorn1,f_vorn2);
		int k3 = check_coco_angle (needle_f[2],f_vorn1,f_vorn2);

		if ((k1+k2+k3) == 0)
		  res = true;
		else
		  res = false;
      }
      else{
	
	        Point f_vorn1 = D.dual(face.first);
		Point f_vorn2 = D.dual(_face.first);
		//	cout << "    f_voronoi_1  is "<< f_vorn1 << endl;
		//	cout << "    f_voronoi_2  is "<< f_vorn2 << endl;
		int k1 = check_coco_angle (needle_f[0],f_vorn1,f_vorn2);
		int k2 = check_coco_angle (needle_f[1],f_vorn1,f_vorn2);
		int k3 = check_coco_angle (needle_f[2],f_vorn1,f_vorn2);

	
		if ((k1+k2+k3) == 0)
		  res = true;
		else
		  res = false;
      }
      return res;
      
}

//夹角计算
int PointCloud::check_coco_angle (AmentaNeedle &needle, Point f_vorn1, Point f_vorn2){

     
       Vector pv = CGAL::NULL_VECTOR;
       Vector pp1 = CGAL::NULL_VECTOR;
       Vector pp2 = CGAL::NULL_VECTOR;
       Point p_pole;
       Point point =  needle.first->point();
       //Point n_pole = needle.second.second.first;
       //当该点为向量时。
       if (needle.second.first.second == 0)
	 //pv =  needle.second.second.first - point;
       	 pv =  needle.second.first.first - CGAL::ORIGIN ;
       else{
	 //当该点为极点坐标时
	 p_pole = needle.second.first.first;
	 //pv = p_pole-needle.second.second.first;
	 pv = p_pole-point;
       }
       
       pp1 = f_vorn1-point;
       pp2 = f_vorn2-point;

       int ret =0;

       double ang_min = cos(PI*0.5 - ANGLE);
       double ang_max = cos(PI*0.5 + ANGLE);

       pv =  pv / sqrt(pv.squared_length());
       pp1 =  pp1 / sqrt(pp1.squared_length());
       pp2 =  pp2 / sqrt(pp2.squared_length());
       
       
       double v1, v2, v3, p11, p12, p13, p21, p22, p23;

       v1 = pv.x();
       v2 = pv.y();
       v3 = pv.z();

       p11 = pp1.x();
       p12 = pp1.y();
       p13 = pp1.z();
     

       p21 = pp2.x();
       p22 = pp2.y();
       p23 = pp2.z();
     
       double norm_pv = sqrt(pv.squared_length());
       double norm_pp1 = sqrt(pp1.squared_length());
       double norm_pp2 = sqrt(pp2.squared_length());

       
       double angle1 = ((v1*p11+v2*p12+v3*p13)*1.0/(norm_pv*norm_pp1));
       double angle2 = ((v1*p21+v2*p22+v3*p23)*1.0/(norm_pv*norm_pp2));
       
       if (((angle1 < ang_max)&&(angle2 < ang_max))||((angle1 > ang_min)&&(angle2 > ang_min)))
     	 ret = 1;
       else
	 ret = 0;
       
       return ret;
}

void PointCloud::output_facet(){

       ofstream outfacet ("./out_fecet.stl");

       if (!outfacet) {
                cerr << "error: unable to open outfacet file: "
                     << outfacet
                     << endl;
		exit(1);
        }

  
       // output_normal.open("./tmp/result/out_normal.stl");
       //output_voronoi.open("./tmp/result/out_voronoi.stl");
     
       outfacet << "solid SURFACE" <<endl;
              
       for (auto it = face_list.begin(); it != face_list.end(); it++){

	      Facet face = *it;
	      Cell_handle cell = face.first;
	      Point opp_p = cell->vertex(face.second)->point();
	     
	      int j = 0;
	      std::vector<Point> p;
	      std::vector<AmentaNeedle> needle_f;
	      for (int i = 0; i <= 3; i++)
		{
		     if(cell->vertex(i) != cell->vertex(face.second))
		             {
			       Point p_tmp = cell->vertex(i)->point();
			       p.push_back(p_tmp);
			       int in_d = cell->vertex(i)->info();
			       // cout <<"    " <<p[j++].y()<< endl;
			       // cout <<"    "<< in_d << endl;
			       needle_f.push_back(needles[in_d]);
			       // cout <<"    the vertex of needle is : " << needle1.first->point() << endl;

		
			     }
		}

	      Point p0 = needle_f[0].first->point();
	      Point p1 = needle_f[1].first->point();
	      Point p2 = needle_f[2].first->point();

	      Vector norm = needle_f[0].second.first.first-p0;
	      
	      outfacet << "facet normal " << norm << endl;
	      outfacet << "outer loop" << endl;
	      outfacet << "vertex " << p0 << endl;
	      outfacet << "vertex " << p1 << endl;
	      outfacet << "vertex " << p2 << endl;
   
	      outfacet << "endloop" << endl;
	      outfacet << "endfacet" << endl;
       }
       outfacet << "ensolid SURFACE" <<endl;

       outfacet.close();
}

//计算面片的法向
Point PointCloud::facet_normal(Point p1, Point p2, Point p3){

 
       Vector v1 = CGAL::NULL_VECTOR;
       Vector v2 = CGAL::NULL_VECTOR;

       v1 = p2-p1;
       v2 = p3-p1;

       double a1, a2, a3, b1, b2, b3, c1, c2, c3;

       a1 = v1.x();
       a2 = v1.y();
       a3 = v1.z();

       b1 = v2.x();
       b2 = v2.y();
       b3 = v2.z();

       c1 = a2*b3-a3*b2;
       c2 = a3*b1-a1*b3;
       c3 = a1*b2-a2*b1;


       
       Point n = Point(c1,c2,c3);

       return n;
     
}




//使凸包上面片的法向指向外侧
Point PointCloud::facet_normal_orient(Point n, Facet face){

      Cell_handle cell = face.first;
      std::vector<Vector> p;
          
      // Point a1 = cell->vertex(0)->point();
      //Point b1 = cell->vertex(1)->point();
      //Point c1 = cell->vertex(2)->point();
      //Point d1 = cell->vertex(3)->point();

      Vector a = cell->vertex(0)->point()-CGAL::ORIGIN;
      Vector b = cell->vertex(1)->point()-CGAL::ORIGIN;
      Vector c = cell->vertex(2)->point()-CGAL::ORIGIN;
      Vector d = cell->vertex(3)->point()-CGAL::ORIGIN;
      
      Vector tetra_center = a+b+c+d;
      tetra_center = tetra_center/4;

        
      for (int i = 0; i <= 3; i++)
	{
	  if(cell->vertex(i) != cell->vertex(face.second))
	    {
	      Vector p_tmp = cell->vertex(i)->point()-CGAL::ORIGIN;
      
	      p.push_back(p_tmp);
	      
	    }
	}

      Vector ft_center = p[0]+p[1]+p[2];
      ft_center = ft_center/3;
      
      tetra_center = tetra_center-ft_center;
      
      Vector norm = n - CGAL::ORIGIN;

      tetra_center =  tetra_center / sqrt(tetra_center.squared_length());
      norm =  norm / sqrt(norm.squared_length());
      
      double a1, a2, a3, b1, b2, b3, c1, c2, c3;

      a1 = tetra_center.x();
      a2 = tetra_center.y();
      a3 = tetra_center.z();

      b1 = norm.x();
      b2 = norm.y();
      b3 = norm.z();

      double norm_1 = sqrt(tetra_center.squared_length());
      double norm_2 = sqrt(norm.squared_length());

      double angle = ((a1*b1+a2*b2+a3*b3)*1.0/(norm_1*norm_2));
      if (angle > 0)
	norm = norm*(-1);

      Point n_tmp = CGAL::ORIGIN + norm;
      return n_tmp;
}

