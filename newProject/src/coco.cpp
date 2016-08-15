#include "coco.h"
using namespace std;


/*将CGAL 中的Facet数据结构转化为自己定义的Face数据结构*/
void coco_facet_to_face (vector<Facet> *facet_list,vector<FaceData*> *face_list, Point_input *Pointmap){

        
        for (auto it = (*facet_list).begin(); it != (*facet_list).end(); it++){

                Facet face = *it;
                Cell_handle cell = face.first;
                Point opp_p = cell->vertex(face.second)->point();
	     
                int j = 0;
                vector<Point> p;
                //std::vector<AmentaNeedle> needle_f;
                for (int i = 0; i <= 3; i++)
                        {
                                if(cell->vertex(i) != cell->vertex(face.second))
                                        {
                                                Point p_tmp = cell->vertex(i)->point();
                                                p.push_back(p_tmp);
     
		
                                        }
                        }

                Point p0 = p[0];
                Point p1 = p[1];
                Point p2 = p[2];


                Vector norm = coco_facet_normal(p0,p1,p2) - CGAL::ORIGIN; 
                /***************/
                /*
                cout << p0 << endl;
                cout << p1 << endl;
                cout << p2 << endl;
                cout <<"\n" <<endl;
                */
                /***************/

                
                //std::cout << "search vertexdata by Point : " << std::endl;

                

                VertexData* v0 = coco_get_pointdata(p0,Pointmap);
                VertexData* v1 = coco_get_pointdata(p1,Pointmap);
                VertexData* v2 = coco_get_pointdata(p2,Pointmap);
                
                //对三个点的id按有大到小的序就行排序
                int v0_id = v0->id;
                int v1_id = v1->id;
                int v2_id = v2->id;

                // cout << "\tbefore sort "<< v0_id << "  " << v1_id << "  " << v2_id << endl;
                
                coco_sort_triangle_point(v0_id,v1_id,v2_id);

                // cout << "\tafter sort  "<<v0_id << "  " << v1_id << "  " << v2_id << endl;
                //  Face face_tmp = make_tuple((v0_id),(v1_id),(v2_id));
               
                FaceData *face_data = new_FaceData();
                *(face_data->face) = make_tuple((v0_id),(v1_id),(v2_id));
                *(face_data->normal) = norm;
                // face_data->points.push_back(p0);
                // face_data->points.push_back(p1);
                // face_data->points.push_back(p2);
                //int s = face_list->size();
                face_data->id = face_list->size();

                (*face_list).push_back(face_data);

        }
                
}

/*获取点对应的点结点信息*/
VertexData *coco_get_pointdata(Point point, Point_input *Pointmap){


        
        const auto myvd = (*Pointmap).by<_point>().equal_range(point);
        auto iter = myvd.first;
        if (iter != myvd.second)
                {
                        VertexData *vd = iter->get<_pointdata>();
                        // Edges_list edgelist = facedata1->edges;
                        //Edge *edge = edgelist[0];
                        //std::cout << (*edge).first<< std::endl;
                        return vd;
                        // std::cout << vd->id << std::endl;
                }
        else
                std::cerr << "!! there is no such record in the pointmap !! " << std::endl;
        
}

/*获取边对于的边结点信息*/
EdgeData *coco_get_edgedata(Edge edge, Edge_input *edgemap){


        
        const auto myed = (*edgemap).by<_edge>().equal_range(edge);
        auto iter = myed.first;
        if (iter != myed.second)
                {
                        EdgeData *ed = iter->get<_edgedata>();
                        // Edges_list edgelist = facedata1->edges;
                        //Edge *edge = edgelist[0];
                        //std::cout << (*edge).first<< std::endl;
                        return ed;
                        // std::cout << vd->id << std::endl;
                }
        else
                return nullptr;
                //std::cout << "!! there is no such edgedata in the table !! " << std::endl;
        
}
/*获取面片对应的面片结点信息*/
FaceData *coco_get_facedata(Face face, Face_input *facemap){

        const auto myfd = (*facemap).by<_face>().equal_range(face);
        auto iter = myfd.first;
        if (iter != myfd.second)
                {
                        FaceData *fd = iter->get<_facedata>();
                        // Edges_list edgelist = facedata1->edges;
                        //Edge *edge = edgelist[0];
                        //std::cout << (*edge).first<< std::endl;
                        return fd;
                        // std::cout << vd->id << std::endl;
                }
        else
                {
               
                        //std::cerr << "!! there is no such facedata in the table !! " << std::endl;
                return nullptr;
                }
        
}

/*对面，边进行相关性解析*/
/*
void coco_association_analysis(vector<FaceData*> *face_list,
                               vector<EdgeData*>*edge_list,
                               vector<VertexData*>*point_list,
                               Point_input *pointmap,
                               Edge_input *edgemap,
                               Face_input *facemap){
        

        int f_size = 0;
        //int e_size = 0;
        for (auto it = (*face_list).begin(); it != (*face_list).end(); it++){

                FaceData *facedata = *it;
                Face *face = facedata->face;
              

                //建立Face和FaceData的双向映射
                 (*facemap).insert(facein(*face,facedata));
                int p0_id = get<0>(*face);
                int p1_id = get<1>(*face);
                int p2_id = get<2>(*face);

                // Point p0 = pl.at(0);
                //   Point p1 = pl.at(1);
                //   Point p2 = pl.at(2);
                        


                
                //cout << p0 << endl;
                //cout << p1 << endl;
                //cout << p2 << endl;
                
             
                
                //  VertexData* v0 = coco_get_pointdata(p0,pointmap);
                // VertexData* v1 = coco_get_pointdata(p1,pointmap);
                // VertexData* v2 = coco_get_pointdata(p2,pointmap);
                VertexData* v0 = point_list->at(p0_id);
                VertexData* v1 = point_list->at(p1_id);
                VertexData* v2 = point_list->at(p2_id);
                
                //将face添加进顶点结点的相关信息中
                (v0->faces)->push_back(facedata->id);
                (v1->faces)->push_back(facedata->id);
                (v2->faces)->push_back(facedata->id);

           
                Edge edge0 = make_pair((v0->id),(v1->id));
                Edge edge1 = make_pair((v0->id),(v2->id));
                Edge edge2 = make_pair((v1->id),(v2->id));


               
                //将三条边添加进面片的边链表中
                //  facedata->edges.push_back(edge0);
                //   facedata->edges.push_back(edge1);
                //  facedata->edges.push_back(edge2);
                
                //对边进行解析
                coco_association_analysis_edge(&edge0,facedata,point_list,edge_list,edgemap);
                coco_association_analysis_edge(&edge1,facedata,point_list,edge_list,edgemap);
                coco_association_analysis_edge(&edge2,facedata,point_list,edge_list,edgemap);
        }

                
                   
}
*/
/*对边进行解析*/
/*
void coco_association_analysis_edge( Edge *edge,
                                     FaceData* fd,
                                     vector<VertexData*>*point_list,
                                     vector<EdgeData*>*edge_list,
                                     Edge_input *edgemap){


        int p1_id = edge->first;
        int p2_id = edge->second;
        VertexData* v1 = point_list->at(p1_id);
        VertexData* v2 = point_list->at(p2_id);
        
        EdgeData *ed = coco_get_edgedata(*edge,edgemap);
        
        if(ed == nullptr){   //*该边在还未添加进边链表中//
                EdgeData *edge_data = new_EdgeData();  //*创建新边/
                *(edge_data->edge) = *edge;
                edge_data->id = edge_list->size();
                edge_data->faces->push_back(fd->id);

                edgemap->insert(edgein(*edge,edge_data));
                edge_list->push_back(edge_data);
                
                v1->edges->push_back(edge_data->id);
                v2->edges->push_back(edge_data->id);
                        
        }else{

                (ed->faces)->push_back(fd->id);
                // v1->edges->push_back(ed->id);
                //v2->edges->push_back(ed->id);
                
        }            
}
*/
void coco_facelist_print(vector<VertexData*> *point_list, vector<FaceData*> *face_list){


        ofstream outfacet ("./out_fecet.stl");
        if (!outfacet) {
                cerr << "error: unable to open outfacet file: "
                     << outfacet
                     << endl;
		exit(1);
        }

        outfacet << "solid SURFACE" <<endl;

        cout <<"the capacity of face_list to print is   " << face_list->capacity() << "   size is"<<  face_list->size() << endl;
        
        for (auto it = (*face_list).begin(); it != (*face_list).end();it++){

                FaceData *facedata = *it;
                Face face = *(facedata->face);
                // Points_list fl = facedata->points;
                
                //Point p0 = get<0>(*face);
                //Point p1 = get<1>(*face);
                //Point p2 = get<2>(*face);
                // Point p0 = fl.at(0);
                //Point p1 = fl.at(1);
                //Point p2 = fl.at(2);
                
                //Point norm = Point(0,0,0);
                /**************生成STL文件**************/
                if (1){
                        outfacet << "facet normal " << Point(0,0,0) << endl;
                        outfacet << "outer loop" << endl;
                        outfacet << "vertex " << *(point_list->at(get<0>(face))->point) << endl;
                        outfacet << "vertex " << *(point_list->at(get<1>(face))->point) << endl;
                        outfacet << "vertex " << *(point_list->at(get<2>(face))->point) << endl;
                        
                        outfacet << "endloop" << endl;
                        outfacet << "endfacet" << endl;
                        // (*fl_tmp).push_back(facedata);
                        
                }
                else {
                        //  (*face_list).erase(it++);
                        
                }



                
                
                /************************************/
                //cout << facedata->tag << endl;
                //cout << p1 << endl;
                //cout << p2 << endl;
        }
        

        outfacet << "ensolid SURFACE" <<endl;
}

void coco_edgelist_print(vector<EdgeData*> *edge_list){

        /*
   for (auto it = (*point_list).begin(); it != (*point_list).end(); it++){

                VertexData *pointdata = *it;
                Point p = pointdata->point;

                cout << p << endl;
                cout << pointdata->faces.size() << endl;
                cout << pointdata->id << endl;
                cout <<endl;
   }
        */


        int ed_len = edge_list->size();
        for(int i = 0; i < ed_len; i++){

                EdgeData *edgedata = (*edge_list)[i];
                
                // cout << edgedata->edge.first << endl;
                // cout << edgedata->id << endl;
                //cout << edgedata->faces->size() << endl;
              
                //cout <<endl;

        }
}

void coco_pointlist_print(vector<VertexData*> *point_list){

        /*
   for (auto it = (*point_list).begin(); it != (*point_list).end(); it++){

                VertexData *pointdata = *it;
                Point p = pointdata->point;

                cout << p << endl;
                cout << pointdata->faces.size() << endl;
                cout << pointdata->id << endl;
                cout <<endl;
   }
        */


        int pt_len = point_list->size();
        for(int i = 0; i < pt_len; i++){

                VertexData *pointdata = (*point_list)[i];
                
                //cout << pointdata->point << endl;
                // cout << pointdata->id << endl;
                //cout << pointdata->faces.size() << endl;
              
                // cout <<endl;

        }
}

/*将面片的三个顶点按id有大到小进行排序*/
void coco_sort_triangle_point(int &id0, int &id1, int &id2){

        int id_tmp;
        if(id0<0 || id1<0 || id2<0)
                cerr << "the triangle point to sort is not exist !!" << endl;

        if(id0 > id1 && id1 > id2)
                {}
        
        if(id0 > id2 && id2 > id1){
                id_tmp = id1;
                id1 = id2;
                id2 = id_tmp;
        }
        
        if(id1 > id0 && id0 > id2){
                id_tmp = id0;
                id0 = id1;
                id1 = id_tmp;
        }        

        if(id1 > id2 && id2 > id0){
                id_tmp = id0;
                id0 = id1;
                id1 = id2;
                id2 = id_tmp;
        }

        if(id2 > id0 && id0 > id1){
                id_tmp = id0;
                id0 = id2;
                id2 = id1;
                id1 = id_tmp;
        }

        if(id2 > id1 && id1 > id0){
                id_tmp = id0;
                id0 = id2;
                id2 = id_tmp;
        }
                        
  
}
//计算面片的法向
Point coco_facet_normal(Point p1, Point p2, Point p3){

 
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

/*流形提取操作*/
/*
void coco_extract_manifold(vector<VertexData*> *point_list,
                           vector<EdgeData*> *edge_list,
                           vector<FaceData*> *face_list,
                           Point_input *pointmap,
                           Edge_input *edgemap,
                           Face_input *facemap){

        int ed_len = edge_list->size();
        cout << "edge_list_size   "<< edge_list->size() << endl;
        //该此循环对面进行第一次标记
        for(int i = 0; i < ed_len; i++){
                
                EdgeData *edgedata = (*edge_list)[i];
                // cout << edgedata->edge.first << endl;
                //cout << edgedata->faces->size() << endl;
             
                if((edgedata->faces->size()) > 2){
                        Data_list fl = *(edgedata->faces);
                        for (auto it = (fl).begin(); it != (fl).end(); it++){
                                int f_id = *it;
                                FaceData *fd = face_list->at(f_id);
                                fd->flag++;
                        }
                        //Point p1 = edgedata->edge.first;
                        //Point p2 = edgedata->edge.second;

                        // VertexData *v1 = coco_get_pointdata(p1,pointmap);
                        // VertexData *v2 = coco_get_pointdata(p2,pointmap);

                        //coco_mark_face(v1, pointmap, facemap);
                        // coco_mark_face(v2, pointmap, facemap);
  
                        
                }
               
        }
        //对面片集合中那些孤立存在的，非四面体面片进行标记
        //coco_mark_single_face(face_list,edge_list,point_list,pointmap,edgemap,facemap);
        //对面进行第三次标记。
        for(int i = 0; i < ed_len; i++){

                EdgeData *edgedata = (*edge_list)[i];
                
                // cout << edgedata->edge.first << endl;
                // cout << edgedata->id << endl;
             
                if((edgedata->faces->size()) > 2){
                        //Faces_list fl = edgedata->faces;
                        
                     
                       
                        int p1_id = edgedata->edge->first;
                        int p2_id = edgedata->edge->second;

                        VertexData *v1 = point_list->at(p1_id);
                        VertexData *v2 = point_list->at(p2_id);

                        //已经流形提取过的点不再进行提取。
                        if (v1->state != 1){
                                coco_mark_face(v1, point_list,edge_list, face_list,pointmap, edgemap, facemap);
                        }
                        // if (v2->state != -1)
                        //      coco_mark_face(v2, point_list,face_list,pointmap, edgemap, facemap);
  
                        
                }
        }

        //创建一个新的存储FaceData的vector，并销毁原先的vector，释放内存。
        vector<FaceData*> *face_list_tmp = new vector<FaceData*>();
        for(auto it = face_list->begin(); it != face_list->end(); it++){

                FaceData *fd = *it;
                FaceData *fd_tmp = new_FaceData();
                *fd_tmp = *fd;
                if(fd->state != 1)
                        face_list_tmp->push_back(fd_tmp);
        }

      
        coco_facedata_free(face_list);
      
        *face_list = *face_list_tmp;
     
        (*face_list_tmp).clear();
        vector<FaceData*>().swap(*face_list_tmp);
               
}
*/

/*

void coco_mark_face(VertexData* v, vector<VertexData*> *point_list, vector<EdgeData*> *edge_list, vector<FaceData*> *face_list,Point_input *pointmap,  Edge_input *edgemap, Face_input *facemap){

        int p_id = v->id;
        Data_list pl_tmp;
        //提取出与点v想关联的边，得到点v的邻点
        Data_list el = *(v->edges);
        for (auto it = el.begin(); it != el.end(); it++){

                int e_id = *it;
                Edge *e = edge_list->at(e_id)->edge;
                int e_p1 = e->first;
                int e_p2 = e->second;
                
                cout << e_p1 << "  " << e_p2 << endl;
                
                if (p_id == e_p1)
                        pl_tmp.push_back(e_p2);
                else
                        pl_tmp.push_back(e_p1);
                
        }

        
        
        //对查询点的非直接关联的面片进行标记
        vector<int> id_list(pl_tmp);
        id_list.push_back(v->id);
        coco_mark_relate_face(&id_list,face_list,point_list,pointmap,facemap);


        
        //对流形提取的点进行state标记，避免以后再次进行检验;并进行tag标记初始化
        v->state = 1;
        for (auto it = (pl_tmp).begin(); it != (pl_tmp).end();it++){
                 int id = *it;
                 
                 VertexData *vd = (*point_list)[id];
                 vd->state = 1;
                 vd->tag = -1;
                 
                 cout << id << endl;
        }

        
        // cout << endl;
        Point p = *(v->point);
        Point p_1 = *(((*point_list)[pl_tmp[0]])->point);
        (*point_list)[pl_tmp[0]]->tag = 0;
        int p1_id = ((*point_list)[pl_tmp[0]])->id;
        Point p_2;
        Point p_tmp;
        int id_tmp;
       
        vector<Edge> el_tmp;
        int pl_tmp_size = pl_tmp.size();
        //定义初始拓扑圆盘法向
        Point f_normal =  coco_facet_normal(p, p_1, *(((*point_list)[pl_tmp[1]])->point));
        
        cout << "the point test is "<< v->id << endl;
        
        while(pl_tmp_size-- > 1){
                
                double ang = -1;
                //
                for (auto it = (pl_tmp).begin(); it != (pl_tmp).end();it++){

                        int id = *it;
                        //if ((*point_list)[id]->tag != 0)
                        //        continue;
                        cout << id << endl;
                        p_2 = *(point_list->at(id)->point);

                        Vector n1 = p_1 - p;
                        Vector n2 = p_2 - p;
                        
                        double ang_tmp =  coco_calculate_ang(n1,n2);
                        
                        //cout <<"id is :"<< id << endl;
                        // cout <<"ang_tmp"<<ang_tmp<< endl;
                        // cout << (*point_list)[id]->tag << endl;

                        if (ang_tmp > ang && ang_tmp < 0.999 && (*point_list)[id]->tag != 0){
                                Point normal_tmp =  coco_facet_normal(p,p_1,p_2);
                                double ang_normal = coco_calculate_ang(normal_tmp-CGAL::ORIGIN,f_normal-CGAL::ORIGIN);
                                if(ang_normal > 0){  
                                        // cout << "pass" << endl;
                                        // cout << "ang_tmp"<< ang_tmp<< endl;
                                        ang = ang_tmp;
                                        p_tmp = p_2;
                                        id_tmp = id;
                                }
                        }
                }
                
                Edge edge = Edge (p1_id,id_tmp);
                p_1 = p_tmp;
                p1_id = id_tmp;
                (*point_list)[id_tmp]->tag = 0;
                el_tmp.push_back(edge);
                
                cout << "the choise point is :  " << id_tmp<< endl;
         
        }
        Edge e_final = Edge(el_tmp[el_tmp.size()-1].second,el_tmp[0].first);
        el_tmp.push_back(e_final);
        
        // Edge e_fir = el_tmp[0];
        // Edge e_las = el_tmp[el_tmp.size()-1];
        
        //对提取的对边链表进行检验，看是否能够构成拓扑圆盘。
     
        for(auto it = el_tmp.begin();it != el_tmp.end();it++){

                Edge e_tmp = *it;
                int e_p1 = e_tmp.first;
                int e_p2 = e_tmp.second;

                //VertexData *e_v1 = point_list.at(e_p1);
                //VertexData *e_v2 = point_list.at(e_p2);
                //VertexData *e_v3 = coco_get_pointdata(e_p3,pointmap);

                // VertexData v0_tmp = *v;
                // VertexData v1_tmp = *e_v1;
                //VertexData v2_tmp = *e_v2;

               

                int id0_tmp = v->id;
                int id1_tmp = e_p1;
                int id2_tmp = e_p2;

                coco_sort_triangle_point(id0_tmp, id1_tmp, id2_tmp);
                 
                Face f_tmp = Face(id0_tmp, id1_tmp, id2_tmp);
                coco_face_add(f_tmp,face_list);

        }
     
}

*/


double coco_calculate_ang(Vector n1, Vector n2){
        

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

        return ang;
}



double coco_calculate_ang_b(Vector n1, Vector n2){
        

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

        return acos(ang);
}




void coco_face_add(Face face,vector<FaceData*> *face_list){

        FaceData * fd = new_FaceData();
        *(fd->face) = face;
        (*face_list).push_back(fd);
       
}



/*
void coco_mark_single_face(vector<FaceData*> *face_list,
                               vector<EdgeData*>*edge_list,
                               vector<VertexData*>*point_list,
                               Point_input *pointmap,
                               Edge_input *edgemap,
                               Face_input *facemap){

         for (auto it = (*face_list).begin(); it != (*face_list).end(); it++){

                 FaceData *facedata = *it;
                 if(facedata->flag >= 2){
                         Face face = facedata->face;
                         Edges_list el = facedata->edges;
                         for(auto itr = el.begin(); itr != el.end(); itr++){

                                 Edge edge = *itr;
                                 EdgeData *ed = coco_get_edgedata(edge,edgemap);

                                 if(ed->faces.size() == 1){
                                         facedata->state = 1;
                                         // cerr << "marked!!" << endl;
                                         Point p1 = get<0>(face);
                                         Point p2 = get<1>(face);
                                         Point p3 = get<2>(face);
                                         
                                         VertexData *v1 = coco_get_pointdata(p1,pointmap);
                                         VertexData *v2 = coco_get_pointdata(p2,pointmap);
                                         VertexData *v3 = coco_get_pointdata(p3,pointmap);

                                         v1->state = -1;
                                         v2->state = -1;
                                         v3->state = -1;
                                         
                                 }
                         }
                  }
         }
}

*/

void coco_mark_relate_face(vector<int>*id_list,
                           vector<FaceData*> *face_list,
                           vector<VertexData*>*point_list,
                           Point_input *pointmap,
                           Face_input *facemap){

      
        sort((*id_list).begin(),(*id_list).end(),Comp);
        int size = (*id_list).size();
        for (int i = 0; i < size; i++){

                for (int j = i+1; j < size; j++){

                        for (int k = j+1; k < size; k++){

                                int id_i = (*id_list)[i];
                                int id_j = (*id_list)[j];
                                int id_k = (*id_list)[k];
                                //   Point pi = (*point_list)[id_i]->point;
                                //   Point pj = (*point_list)[id_j]->point;
                                //   Point pk = (*point_list)[id_k]->point;
                                // cerr << id_i << "  "<< id_j<<"  " << id_k << endl;
                                Face f = Face(id_i,id_j,id_k);
                                FaceData * fd = coco_get_facedata(f,facemap);
                                if (fd != nullptr){
                                        //cerr << "found!!!" <<endl;
                                        fd->state = 1;
                                }
                        }

                }
                

        }
}




/*
void coco_coco_association_analysis_2th(vector<FaceData*> *face_list,
                                        vector<VertexData*>*point_list,
                                        Point_input *pointmap)
{
        
      
        Edge_input *edgemap2 = new Edge_input;
        Face_input *facemap2 = new Face_input;
        vector<EdgeData*> *edge_list2 = new vector<EdgeData*>();
        
        
        for (auto it = (*point_list).begin(); it != (*point_list).end(); it++){
                VertexData *vd = *it;
                //清空边链表
                vd->edges->clear();
                vector <int>().swap(*(vd->edges));

                vd->faces->clear();
                vector <int>().swap(*(vd->faces));
                
                vd->flag = 0;
                vd->state = 0;
                
        }
   
        for (auto it = (*face_list).begin(); it != (*face_list).end(); it++){

                FaceData *fd = *it;
                //清空边链表
                // fd->points.clear();
                // vector <Point>().swap(fd->points);

                fd->edges->clear();
                vector <int>().swap(*(fd->edges));
                
                fd->flag = 0;
                fd->tag = 0;
                fd->state = -1;
                fd->ang = 1;
                
        }
        

        coco_association_analysis(face_list,edge_list2,point_list,pointmap,edgemap2,facemap2);

        coco_extract_manifold(point_list, edge_list2,face_list, pointmap, edgemap2, facemap2);
        
        //vector<FaceData*> *fl_tmp2 = new vector<FaceData*>();
        coco_facelist_print(point_list,face_list);
}


*/

bool Comp(const int &a,const int &b)
{
        return a>b;       
}


