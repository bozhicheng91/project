#include "IncLocal.h"

//初始网格重建
void IncLocal_initialMesh(Point p,
                          int initMeshNumber,
                          Tree *kd_tree,
                          Point_input *pointmap,
                          vector<VertexData*> *pointlist
                          ){


        //获取初始网格点云数据
        vector<Point> knn_1;
        Neighbor_search search(*kd_tree, p, initMeshNumber);
        for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
                knn_1.push_back(it->first);
        }

        //将数据由point转化为vertexdata
        vector<VertexData*>vl;
        IncLocal_pointTOvertexdata(&knn_1, &vl, pointmap);
        //将点集标记为重建点
        IncLocal_markReconstructionVertex(&vl);

        vector<Point> auxiliPoints;
        IncLocal_addAuxiliaryPoints(&knn_1, &auxiliPoints,kd_tree,pointmap);

        //打印重建样点和辅助点信息
        int origin = 0;
        int auxiliary = 1;
        
        //cout << knn_1.size() << endl;
        //cout << auxiliPoints.size() << endl;

        IncTestFunc_outputData(&knn_1, origin);
        IncTestFunc_outputData(&auxiliPoints, auxiliary);
        //将辅助点追加到重建集合中
        knn_1.insert(knn_1.end(), auxiliPoints.begin(), auxiliPoints.end());

        //对局部点集进行cocone重建
        vector<FaceData*> facelist;
        IncLocal_coconeReconstruction(&knn_1, &facelist, pointmap);

        
        //>>>>>>>信息打印>>>>>>>>>>>//
        cout << facelist.size() << endl;
        int originFacetNum = 0;
        IncTestFunc_outputData(pointlist,  &facelist, originFacetNum);
        //>>>>>>>>>>>>>>>>>>>>>>>>>>//
        
         //依据辅助点进行面片选择
        IncLocal_facetSelection(&facelist, pointlist, pointmap);
      
        //>>>>>>>>>信息打印>>>>>>>>>//
        cout << facelist.size() << endl;
        int seletFacetNum = 1;
        IncTestFunc_outputData(pointlist,  &facelist, seletFacetNum);
        //>>>>>>>>>>>>>>>>>>>>>//
       
        //建立面、边、点 之间映射关系
        vector<EdgeData*> edgelist;
        // vector<FaceData*> facelist;
        Edge_input edgemap;
        Face_input facemap;

        IncLocal_association_analysis(&facelist,&edgelist,pointlist,pointmap,&edgemap,&facemap);

        
        //流形提取操作
        IncLocal_extractManifold(pointlist,&edgelist,&facelist,pointmap,&edgemap,&facemap);

        //>>>>>>>>>>>>>>>>>>>>//
        int manifoldFacetNum = 2;
        IncTestFunc_outputData(pointlist,  &facelist,manifoldFacetNum);
        //>>>>>>>>>>>>>>>>>>>>>//

        //边链表和面片链表内存释放。虽然vector链表不是
        //动态分配的内存，但是内部存储的变量为动态分配,
        //因此该处需要进行释放,
        coco_edgedata_free(&edgelist);
        coco_facedata_free(&facelist);
       
}


//实现点集间的转化。
void IncLocal_pointTOvertexdata(vector<Point>*pl, vector<VertexData*>* vl,Point_input *pointmap ){

        int pl_size = pl->size();
        for(int i = 0; i < pl_size; i++){
                Point p = pl->at(i);
                VertexData *vd = coco_get_pointdata(p,pointmap);

                vl->push_back(vd);
  
        }
}

//将输入点集标记为重建点
void IncLocal_markReconstructionVertex(vector<VertexData*> *vl){

        for(auto it = vl->begin(); it != vl->end(); it++){

                VertexData *vd = *it;
                vd->tag = 2; //tag 为 2，表示该点为重建点
             

        }
        
}

//辅助点添加
void IncLocal_addAuxiliaryPoints(vector<Point>*pointInput,
                                 vector<Point>*pointOutput,
                                 Tree *kd_tree,
                                 Point_input *pointmap){

        if (pointInput->size() == 0){
                std::cerr << "Error at  IncLocal_addAuxiliaryPoints  is NULL !!!!" << std::endl;
        }

            for(auto it = pointInput->begin(); it != pointInput->end(); it++){
                    
                Point p = *it;

                
                
                Neighbor_search search(*kd_tree, p, 10);
                for(Neighbor_search::iterator itra = search.begin(); itra != search.end(); ++itra){
                        VertexData *vd = IncLocal_getVertexData(itra->first, pointmap);
                        if (vd->tag != 2 && vd->tag != 1){
                                pointOutput->push_back(itra->first);
                                vd->tag = 1;
                                // cout << vd->tag << endl;
                        }
                }
        }

}

/*获取点对应的点结点信息*/
VertexData *IncLocal_getVertexData(Point point, Point_input *Pointmap){


        
        const auto myvd = (*Pointmap).by<_point>().equal_range(point);
        auto iter = myvd.first;
        if (iter != myvd.second)
                {
                        VertexData *vd = iter->get<_pointdata>();
                        return vd;
                }
        else
                std::cerr << "Error  !! there is no such record in the pointmap !! " << std::endl;
        
}

/*获取边对于的边结点信息*/
EdgeData *IncLocal_getEdgeData(Edge edge, Edge_input *edgemap){


        
        const auto myed = (*edgemap).by<_edge>().equal_range(edge);
        auto iter = myed.first;
        if (iter != myed.second)
                {
                        EdgeData *ed = iter->get<_edgedata>();
                        return ed;
                }
        else
                return nullptr;
                //std::cout << "!! there is no such edgedata in the table !! " << std::endl;
        
}

/*获取面片对应的面片结点信息*/
FaceData *IncLocal_getFaceData(Face face, Face_input *facemap){

        const auto myfd = (*facemap).by<_face>().equal_range(face);
        auto iter = myfd.first;
        if (iter != myfd.second)
                {
                        FaceData *fd = iter->get<_facedata>();
                        return fd;
                }
        else
                {
               
                   //std::cerr << "!! there is no such facedata in the table !! " << std::endl;
                return nullptr;
                }
}





void IncLocal_coconeReconstruction(vector<Point>*pointInput, vector<FaceData*>*facelistOutput, Point_input *pointmap){


        PointCloud A = PointCloud(pointInput);
        (A).delaunay();
        //cerr << "\t Dealunay is done："<< endl;
        (A).all_amenta_needles();
        //cerr << "\t Amenta calculate is done："<< endl;
        (A).facet_filter();
        //cerr << "\t facet filter is done："<< endl;
        coco_facet_to_face(&(A.face_list),facelistOutput,pointmap);

}

void IncLocal_facetSelection(vector<FaceData*> *facelist, vector<VertexData*> *pointlist, Point_input *pointmap){

        vector<FaceData*> facelistTmp;
        int facelistSize = facelist->size();
        //for (auto it = (*face_list_in).begin(); it != (*face_list_in).end();it++){
        for(int i = 0; i < facelistSize; i++){
                FaceData *facedata = facelist->at(i);
                Face face = *(facedata->face);
                int p0_id = get<0>(face);
                int p1_id = get<1>(face);
                int p2_id = get<2>(face);
       
                VertexData* v0 = pointlist->at(p0_id);
                VertexData* v1 = pointlist->at(p1_id);
                VertexData* v2 = pointlist->at(p2_id);

                if ((v0->tag + v1->tag + v2->tag ) == 6){
                        FaceData *faceNew = new_FaceData();
                        coco_facedataCopy(faceNew, facedata);
                        facelistTmp.push_back(faceNew);

                }

        }
        coco_facedata_free(facelist);
        facelist->swap(facelistTmp);
        // coco_facedata_free(&facelistTmp);

}

/*对面，边进行相关性解析*/

void IncLocal_association_analysis(vector<FaceData*> *face_list,
                               vector<EdgeData*>*edge_list,
                               vector<VertexData*>*point_list,
                               Point_input *pointmap,
                               Edge_input *edgemap,
                               Face_input *facemap){
        

        int f_size = 0;
        //int e_size = 0;
        int f_id = 0;
        for (auto it = (*face_list).begin(); it != (*face_list).end(); it++){

                FaceData *facedata = *it;
                Face *face = facedata->face;
              
                facedata->id = f_id++;
                //建立Face和FaceData的双向映射
                 (*facemap).insert(facein(*face,facedata));

                 //获取面片的三个顶点
                int p0_id = get<0>(*face);
                int p1_id = get<1>(*face);
                int p2_id = get<2>(*face);

                VertexData* v0 = point_list->at(p0_id);
                VertexData* v1 = point_list->at(p1_id);
                VertexData* v2 = point_list->at(p2_id);
                
                //将face添加进顶点结点的相关信息中
                (v0->faces)->push_back(facedata->id);
                (v1->faces)->push_back(facedata->id);
                (v2->faces)->push_back(facedata->id);

                //构造边
                Edge edge0 = make_pair((v0->id),(v1->id));
                Edge edge1 = make_pair((v0->id),(v2->id));
                Edge edge2 = make_pair((v1->id),(v2->id));


               
                //将三条边添加进面片的边链表中
                //  facedata->edges.push_back(edge0);
                //   facedata->edges.push_back(edge1);
                //  facedata->edges.push_back(edge2);
                
                //对边进行解析
                IncLocal_association_analysis_edge(&edge0,facedata,point_list,edge_list,edgemap);
                IncLocal_association_analysis_edge(&edge1,facedata,point_list,edge_list,edgemap);
                IncLocal_association_analysis_edge(&edge2,facedata,point_list,edge_list,edgemap);
        }

                
                   
}

/*对边进行解析*/

void IncLocal_association_analysis_edge( 
                            Edge *edge,
                            FaceData* fd,
                            vector<VertexData*>*point_list,
                            vector<EdgeData*>*edge_list,
                            Edge_input *edgemap){


        int p1_id = edge->first;
        int p2_id = edge->second;
        VertexData* v1 = point_list->at(p1_id);
        VertexData* v2 = point_list->at(p2_id);
        
        EdgeData *ed = IncLocal_getEdgeData(*edge,edgemap);
        
        if(ed == nullptr){   //*该边在还未添加进边链表中//
                EdgeData *edge_data = new_EdgeData();  //*创建新边/
                *(edge_data->edge) = *edge;
                edge_data->id = edge_list->size();
                edge_data->faces->push_back(fd->id);

                edgemap->insert(edgein(*edge,edge_data));
                edge_list->push_back(edge_data);
                
                v1->edges->push_back(edge_data->id);
                v2->edges->push_back(edge_data->id);
                        
        }else{   //该边已在边链表中存在

                (ed->faces)->push_back(fd->id);
                              
        }
}



/*流形提取操作*/

void IncLocal_extractManifold(vector<VertexData*> *point_list,
                           vector<EdgeData*> *edge_list,
                           vector<FaceData*> *face_list,
                           Point_input *pointmap,
                           Edge_input *edgemap,
                           Face_input *facemap){

       

        //遍历边链表，提取非流形边，对非流形边的一个端点进行映射面片标记和重新组合
        int ed_len = edge_list->size();
        for(int i = 0; i < ed_len; i++){
                EdgeData *edgedata = (*edge_list)[i];
                if((edgedata->faces->size()) > 2){
                        int p1_id = edgedata->edge->first;
                        int p2_id = edgedata->edge->second;

                        VertexData *v1 = point_list->at(p1_id);
                        VertexData *v2 = point_list->at(p2_id);

                        if (v1->state != 1){
                                IncLocal_extractFacet(v1, point_list,edge_list, face_list,pointmap, edgemap, facemap);
                        }
                                            
                }
        }

        //剔除面片链表中 state=1 的面片
        vector<FaceData*> face_list_tmp;
        for(auto it = face_list->begin(); it != face_list->end(); it++){

                FaceData *fd = *it;
               
                // *fd_tmp = *fd;
                if(fd->state != 1)
                        {
                                FaceData *fd_tmp = new_FaceData();
                                coco_facedataCopy(fd_tmp, fd);
                                face_list_tmp.push_back(fd_tmp);
                        }        
        }

        cout<< face_list->size() << endl;
        coco_facedata_free(face_list); 
        face_list->swap(face_list_tmp);
        //coco_facedata_free(&face_list_tmp);
        cout<< face_list->size() << endl;   
}




/*对点v的映射面片进行标记，并重新生成点v的映射面片*/
void IncLocal_extractFacet(VertexData* v, 
                           vector<VertexData*> *point_list, 
                           vector<EdgeData*> *edge_list, 
                           vector<FaceData*> *face_list,
                           Point_input *pointmap,  
                           Edge_input *edgemap, 
                           Face_input *facemap){

        int p_id = v->id;
        Data_list pl_tmp;
        //通过v的映射边获取v的邻点
        Data_list el = *(v->edges);
        for (auto it = el.begin(); it != el.end(); it++){

                int e_id = *it;
                Edge *e = edge_list->at(e_id)->edge;
                int e_p1 = e->first;
                int e_p2 = e->second;
                
                // cout << e_p1 << "  " << e_p2 << endl;
                
                if (p_id == e_p1)
                        pl_tmp.push_back(e_p2);
                else
                        pl_tmp.push_back(e_p1);
                
        }

        
        
        //对映射于v的面片进行标记，此处包括不直接与v相连的面片！！！
        //标记后面片  state=1
        vector<int> id_list(pl_tmp);
        id_list.push_back(v->id);
        IncLocal_markFace_accidentToVertex(&id_list,face_list,point_list,pointmap,facemap);


        
        //对流形提取的点进行state标记，避免以后再次进行检验;并进行tag标记初始化
        v->state = 1;
        for (auto it = (pl_tmp).begin(); it != (pl_tmp).end();it++){
                 int id = *it;
                 
                 VertexData *vd = (*point_list)[id];
                 vd->state = 1;
                 vd->tag = -1;
                 
        }


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
        Point f_normal =  IncLocal_facetNormal(p, p_1, *(((*point_list)[pl_tmp[1]])->point));
        
        //将输入样点按照某一顺序排序
        while(pl_tmp_size-- > 1){
                
                double ang = -1;
                //
                for (auto it = (pl_tmp).begin(); it != (pl_tmp).end();it++){

                        int id = *it;
                        p_2 = *(point_list->at(id)->point);

                        Vector n1 = p_1 - p;
                        Vector n2 = p_2 - p;
                        
                        double ang_tmp = IncLocal_calculateAngle(n1,n2);
                        
                        if (ang_tmp > ang && ang_tmp < 0.999 && (*point_list)[id]->tag != 0){
                                Point normal_tmp = IncLocal_facetNormal(p,p_1,p_2);
                                double ang_normal = IncLocal_calculateAngle(normal_tmp-CGAL::ORIGIN,f_normal-CGAL::ORIGIN);
                                if(ang_normal > 0){  
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
                
        }
        Edge e_final = Edge(el_tmp[el_tmp.size()-1].second,el_tmp[0].first);
        el_tmp.push_back(e_final);
        
        //将排序后的样点重新组合成面片，并添加到面片链表中
     
        for(auto it = el_tmp.begin();it != el_tmp.end();it++){

                Edge e_tmp = *it;
                int e_p1 = e_tmp.first;
                int e_p2 = e_tmp.second;

                int id0_tmp = v->id;
                int id1_tmp = e_p1;
                int id2_tmp = e_p2;

                coco_sort_triangle_point(id0_tmp, id1_tmp, id2_tmp);
                 
                Face f_tmp = Face(id0_tmp, id1_tmp, id2_tmp);
                IncLocal_faceAddToList(f_tmp,face_list);

        }
     
}



/*对面链表中由输入样点构成的面片进行标记*/
void IncLocal_markFace_accidentToVertex(
                       vector<int>*id_list,
                       vector<FaceData*> *face_list,
                       vector<VertexData*>*point_list,
                       Point_input *pointmap,
                       Face_input *facemap){

        //对输入样点按照其id由大到小进行排序
        sort((*id_list).begin(),(*id_list).end(),Comp);
        int size = (*id_list).size();
        for (int i = 0; i < size; i++){
                for (int j = i+1; j < size; j++){
                        for (int k = j+1; k < size; k++){
                                int id_i = (*id_list)[i];
                                int id_j = (*id_list)[j];
                                int id_k = (*id_list)[k];
                              
                                Face f = Face(id_i,id_j,id_k);
                                FaceData * fd = IncLocal_getFaceData(f,facemap);
                                if (fd != nullptr)
                                        fd->state = 1; 
                                        
                        }
                }
        }
}



/*比较函数，判断两个int值的大小*/
bool IncLocal_Comp(const int &a,const int &b)
{
        return a>b;       
}


//计算面片的法向
Point IncLocal_facetNormal(Point p1, Point p2, Point p3){

 
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

/*计算两个向量之间夹角*/
double IncLocal_calculateAngle(Vector n1, Vector n2){
        

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

/*将面片附加到面片链表*/
void IncLocal_faceAddToList(Face face,vector<FaceData*> *face_list){

        FaceData * fd = new_FaceData();
        *(fd->face) = face;
        (*face_list).push_back(fd);
       
}

/*提取波前环*/
void IncLocal_frontExtract(vector<EdgeData*> *edgeList, 
                           vector<VertexData*> frontList){

        vector<EdgeData*> frontEdge;
        for(auto it = edgeList->begin(); it != edgeList->end(); it++){
                EdgeData *edge = *it;
                if(edge->faces->size() == 1)
                        frontEdge.push_back(edge);

        }
        
        IncLocal_sortFrontEdge(&frontEdge);

       
}

/*对波前边排序*/
void IncLocal_sortFrontEdge(vector<EdgeData*> *frontEdge){

        
        EdgeData *firstEdge;
        if(frontEdge->at(0) != nullptr)
                firstEdge =  frontEdge->at(0);
        else
                cerr << "the forntEdge list is NULL, <IncLocal_sortFrontEdge> function " << endl;
        
        vector<EdgeData*> sortedEdge;
        sortedEdge.push_back(firstEdge);

        int firEdge_v1 = firstEdge->edge->first;
        int firEdge_v2 = firstEdge->edge->second;
        
        for(auto it = frontEdge->begin(); it != frontEdge->end(); it++){

                EdgeData *edgedata = *it;
                if(IncLocal_compEdgeData(edgedata,firstEdge) == 1)
                        sortedEdge.push_back(edgedata);
  
        }
        coco_edgedata_free(frontEdge);
        frontEdge->swap(sortedEdge);
}

/*比较两条边的信息，若两边不同，返回０，若存在一个共同顶点，返回１，存在两个共同顶点，返回２*/
int IncLocal_compEdgeData(EdgeData* firstEdge, EdgeData* secondEdge){

        int FEv1 = firstEdge->edge->first;
        int FEv2 = firstEdge->edge->second;
        int SEv1 = secondEdge->edge->first;
        int SEv2 = secondEdge->edge->second;

        if(FEv1 != SEv1 && FEv1 != SEv2 && FEv2 != SEv1 && FEv2 != SEv2 )
                return 0;
        else if ((FEv1 == SEv1 && FEv2 == SEv2) || (FEv1 == SEv2 && FEv2 != SEv1))
                return 2;
        else {
                if(FEv2 == SEv2)
                        {
                                int tmp = SEv1;
                                SEv1 = SEv2;
                                SEv2 = tmp;
                        }
                        
                return 1;
        }
}
