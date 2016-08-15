#include "border.h"


void
coco_border_kboundary_extract_boundary (vector<Point> *boundary, vector<Point> *pl, guint k, gdouble e, Tree *kd_tree)
{
        cout << "\t Start to extract the boundary!\n" << endl;

        int pl_size = pl->size();

        assert(pl_size != 0);

        cout << pl_size << endl;
        
        for(int i = 0; i < pl_size; i++){    
                Point point = pl->at(i);
                //   cout << point << endl;
                vector<Point> neighbors;
                Neighbor_search search(*kd_tree, point, k);
                for(Neighbor_search::iterator it = search.begin();
                    it != search.end(); ++it){
                        neighbors.push_back(it->first);
                }

                // cout <<"neighbor size is :  "<<neighbors.size()<<endl;
                if (is_boundary(point, &neighbors, e) == TRUE)
                        boundary->push_back(point);
                //cout <<"ok a " << endl;
                neighbors.clear();
                vector<Point>().swap(neighbors);
               
        }
  
        
        //  return boundary; //
}
//利用函数重载，输入为自定义的点数据容器
void
coco_border_kboundary_extract_boundary (vector<Point>* boundary, vector<VertexData*> *pl, guint k, gdouble e, Tree *kd_tree)
{
        cout << "Start to extract the boundary!\n" << endl;
        // cout <<"pl size is :  "<<pl->size()<<endl;
        //gint count = 1;
        // vector<Point> *boundary = new vector<Point>();
        for(auto it = (*pl).begin(); it != (*pl).end(); it++) {
                                
                VertexData *vd = *it;

                Point point = *(vd->point);
            
                vector<Point> neighbors;
                Neighbor_search search(*kd_tree, point, k);
                for(Neighbor_search::iterator it = search.begin();
                    it != search.end(); ++it){
                        neighbors.push_back(it->first);
                }

                // cout <<"neighbor size is :  "<<neighbors.size()<<endl;
                if (is_boundary(point, &neighbors, e) == TRUE)
                        boundary->push_back(point);
                //cout <<"ok a " << endl;
                neighbors.clear();
                vector<Point>().swap(neighbors);
               
        }
    
        // return boundary; //
}


Point coco_border_get_square_plane(vector<Point> *kn){

        gsl_vector *x = gsl_vector_alloc (3); // 线性方程组的解，亦即微切平面的系
        gdouble a_data[9] = {0};
        gdouble b_data[3] = {0};
        int kn_size = kn->size();

        for(int i = 0; i < kn_size; i++){

                Point p = kn->at(i);
                a_data[0] += p.x()*p.x();
                a_data[1] += p.x()*p.y();
                a_data[2] += p.x();
                a_data[3] += p.x()*p.y();
                a_data[4] += p.y()*p.y();
                a_data[5] += p.y();
                a_data[6] += p.x();
                a_data[7] += p.y();
                b_data[0] += p.x()*p.z();
                b_data[1] += p.y()*p.z();
                b_data[2] += p.z();

        }
        a_data[8] = kn_size;
        gsl_matrix_view m = gsl_matrix_view_array (a_data, 3, 3);
        gsl_vector_view b = gsl_vector_view_array (b_data, 3);
        gint s;
        gsl_permutation * pe = gsl_permutation_alloc (3);
        gsl_linalg_LU_decomp (&m.matrix, pe, &s);
        gsl_linalg_LU_solve (&m.matrix, pe, &b.vector, x);
        gsl_permutation_free (pe);
        Point p_tmp = Point(gsl_vector_get(x,0),gsl_vector_get(x,1),gsl_vector_get(x,2));

        gsl_vector_free(x);
        
        // cout <<"least plane point is get " << endl;
        
        return p_tmp;
}


void coco_border_get_rejection_points ( vector<Point> *rpoints, vector<Point> *kn, Point func)
{
        //  vector<Point> *rpoints  = new vector<Point>(); // 投影点链表

        //设平面的一般方程为：ax+by+cz+d=0，其中 c=-1，其它各系数如下：
        
        gdouble a = func.x();
        gdouble b = func.y();
        gdouble d = func.z();
        gdouble e = a * a + b * b + 1;
        int kn_size = kn->size();
        // cout <<"POINT   "<< kn->at(0) << endl;
        for(int i = 0; i < kn_size; i++){

                Point p = kn->at(i);
                gdouble x = p.x();
                gdouble y = p.y();
                gdouble z = p.z();
                gdouble rx = ((b*b+1)*x - a*b*y + a*z - a*d) / e;
                gdouble ry = ((-1)*a*b*x + (a*a+1)*y + b*z - b*d) / e;
                gdouble rz = (a*x + b*y + (a*a+b*b)*z + d) / e;

                Point p_tmp = Point(rx,ry,rz);
                rpoints->push_back(p_tmp);
                // cout << p_tmp << endl;

        }
        //  cout <<" rejection points list  is  got" << endl;

        //  coco_point_save_to_file_2th(rpoints);
      

}



gboolean is_boundary (Point point, vector<Point> *neighbors, gdouble e)
{

        gboolean bln = FALSE;

        //求最小二乘平面
        Point func = coco_border_get_square_plane (neighbors);
        //获得在最小二乘平面上的投影点
        vector<Point> *rps = new vector<Point>();
        coco_border_get_rejection_points (rps, neighbors, func);
        
        vector<double> *ang_l = new vector<double>();
        coco_border_sort_neighbors_angle(ang_l,rps);
        
        if (ang_l->at(0) > e) {
               
                bln = TRUE;
                // cout << "the max ang is  " << ang_l->at(0) << endl;
                //  g_printf ("A boundary point!\n");
        }
        // cout << "bangle is  "<< bangle.at(0)  << endl;
        // cout <<"is_boundary "  << bln<< endl;

             
        //对ang_l vector 释放内存。
        (*ang_l).clear();
        vector<double>().swap(*ang_l);
        delete ang_l;

        //对rps vector进行内存释放;
        rps->clear();
        vector<Point>().swap(*rps);
        delete rps;
        
        return bln;
}

void selection_sort (vector<double> *angle)
{
        gint i, j, k;
        gdouble t;
        for (i = 0; i < angle->size()-1;i++){
                k = i;
                for (j = i + 1; j < angle->size();j++)
                        if (angle->at(k) < angle->at(j))
                                k = j;
                if (k != i) {
                        t = angle->at(i);
                        angle->at(i) = angle->at(k);
                        angle->at(k) = t;
                }
        }
}





double coco_border_calculate_ang_b(Vector n1, Vector n2){
        

        double v1, v2, v3, p1, p2, p3;
        
        v1 = n1.x();
        v2 = n1.y();
        v3 = n1.z();
        
        p1 = n2.x();
        p2 = n2.y();
        p3 = n2.z();
        
        double norm_v = sqrt(n1.squared_length());
        double norm_p = sqrt(n2.squared_length());

        double ang = (v1*p1+v2*p2+v3*p3)/(norm_v*norm_p);

        // cout << "ang is  " <<acos(ang) << endl;
        //  cout << "M_PI is " << M_PI_2 << endl;
        return acos(ang);
}

void coco_point_save_to_file(vector<Point>* pl){

        ofstream  output_point;
        output_point.open("./data.asc");
        /*
        for(auto it = (*pl).begin(); it != (*pl).end(); it++){

                Point point = *it;
                output_point << point << endl;
                

        }
        */
        int size = pl->size();

        for(int i = 0; i < size; i++){

                output_point<<pl->at(i)<<endl;

        }

}


void coco_point_save_to_file_2th(vector<Point>* pl){

        ofstream  output_point;
        output_point.open("./data-border.asc");

        for(auto it = (*pl).begin(); it != (*pl).end(); it++){

                Point point = *it;
                output_point << point << endl;
                

        }

}

void
coco_border_sort_neighbors_angle(vector<double> *ang_l, vector<Point>*kn){

        if (kn->size() == 0)
                cerr << "the border detect neighbors is NULL" << endl;
        Point p0 = kn->at(0);
        kn->erase(kn->begin());

        // cout << kn->size() << endl;
        
        Point f_normal = coco_facet_normal(p0,kn->at(1),kn->at(2));
        Point p1 = kn->back();
        // kn->erase(kn->end());

        Vector base = p1 - p0;
      
      
        Point pi;
        int size = kn->size();
        int i = 1;
        while(size-->0){
                double ang = 2*M_PI;
                //auto itra = (*kn).begin();
                for(auto it = (*kn).begin(); it != (*kn).end();it++){
                
                        Point p_tmp = *it;
                        Vector pn = p_tmp - p0;
                        double ang_tmp = coco_border_calculate_ang_b(pn, p1-p0);

                        //cout << p0 <<"  "<< p1 <<"  "<< p_tmp <<"  " << ang_tmp << endl;
 
                        if ((ang_tmp < ang)&&(ang_tmp > 0.001) ){
                                Point norm = coco_facet_normal(p0,p1,p_tmp);
                                double ang_norm = coco_border_calculate_ang_b(norm-CGAL::ORIGIN,f_normal-CGAL::ORIGIN);
                                if(ang_norm < 0.5*M_PI){
                                        ang = ang_tmp;
                                        pi = p_tmp;
                                        //   itra = it;
                                }
                        
                        }
              
                }
                
                //cout <<"ang is  " << ang<< endl;
                ang_l->push_back(ang);
                if(ang > 6.28)
                        {
                                //  cout << "break" << endl;
                                break;
                        }
                p1 = pi;
                //  cout << p1 << endl;

        
        }
        selection_sort(ang_l);
        double ang_a = 0.0;
        for(auto it = (*ang_l).begin(); it != (*ang_l).end(); it++){

                ang_a = ang_a+ (*it);
        }
       
     
        
}

//实现点集间的转化。
void coco_pl_to_vl(vector<Point>*pl, vector<VertexData*>* vl,Point_input *pointmap ){

        int pl_size = pl->size();
        for(int i = 0; i < pl_size; i++){
                Point p = pl->at(i);
                VertexData *vd = coco_get_pointdata(p,pointmap);

                vl->push_back(vd);
  
        }
}

//对待重建点进行标记
void coco_mark_vertexdata_recon(vector<VertexData*> *vl){

        for(auto it = vl->begin(); it != vl->end(); it++){

                VertexData *vd = *it;
                vd->tag = 2; //tag 为 2 ，表示该点为进行面片选择的点
                vd->state = 1;//state 为 1 ，表示该点为已重建的点

        }
        
}

//边界点扩增，实现边界样点正确重建。
void coco_border_extend_for_assure(vector<Point> *pl, vector<Point> *pl_extend, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap){

        vector<Point> knn;
        for(auto it = pl->begin(); it != pl->end(); it++){

                Point p = *it;
                Neighbor_search search(*kd_tree, p, BOR_PRO_SIZE);
                for(Neighbor_search::iterator itra = search.begin(); itra != search.end(); ++itra){
                        knn.push_back(itra->first);
                }
        }
        
        vector<VertexData*>vl;
        
        coco_pl_to_vl(&knn,&vl,pointmap);

        //knn内存释放
        knn.clear();
        vector<Point>().swap(knn);

        vector<int> id_list;
        for(auto itrc = vl.begin(); itrc != vl.end(); itrc++){

                VertexData *vd = *itrc;
                //  vd->flag = 1; //将边界点的 近邻点进行标记，在下次搜索重建点时，避免对该点进行搜索。
                int vd_id = vd->id;
                id_list.push_back(vd_id);
        }
        
        //vl 内存释放
        vl.clear();
        vector<VertexData*>().swap(vl);
        
        sort(id_list.begin(),id_list.end());
        id_list.erase( unique(id_list.begin(), id_list.end() ), id_list.end() );

        for(auto it = id_list.begin(); it != id_list.end(); it++){

                int v_id = *it;
                Point p  = *(point_list->at(v_id)->point);
                if(point_list->at(v_id)->tag != 2)
                        pl_extend->push_back(p);

        }
        // cout << "border extend size , after sort " << pl_extend->size() << endl;
    
        
}


//边界点扩增，实现重建区域的扩展。
void coco_border_extend_for_recons(vector<Point> *pl, vector<Point> *pl_extend, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap){

        //求取边界各个样点的k近邻。
        // vector<Point> knn;
        int pl_size = pl->size(); 

        
        for(int i = 0; i < pl_size ;i++){
                Point p = pl->at(i);
                VertexData* vd = coco_get_pointdata(p,pointmap);

                if (vd->flag == 1)
                        continue;
                //  vd->flag = 1;
                
                //  cout << vd->tag << endl;
                if(vd->tag == 3)
                        continue;
                Neighbor_search search(*kd_tree, p, EXTENDING_SIZE);
                for(Neighbor_search::iterator itra = search.begin(); itra != search.end(); ++itra){
                        //检测该点的标记。
                        if(coco_border_extend_point_test(itra->first, pointmap)){
                                pl_extend->push_back((itra->first));
                        }
                }
                
        }
 
        cout << "\t the reconstruction point size is   " << pl_extend->size() << endl;
    
        
}



