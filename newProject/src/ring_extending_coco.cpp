#include "ring_extending_coco.h"

//点云初始化函数，实现：点云输入点表，对点云构建kd树，设定点的ID ,构建pointmap对应关系。
void ring_extending_file_input(Point *p_tmp,
                               string file_path,
                               Tree *kd_tree,
                               Point_input *pointmap,
                               vector<VertexData*> *point_list){

        
	ifstream in(file_path);
        if (!in) {
                cerr << "error: unable to open input file: "
                     << in
                     << endl;
        }
        string line, word;
        double x, y, z;
        int i =0;
        while (getline(in, line)) {
                istringstream stream(line);
                stream >> x >> y >> z;
                VertexData *vd  = new_VertexData();
                *(vd->point) = Point(x,y,z);
                kd_tree->insert(*(vd->point));
                
                vd->id = i++;
                (*point_list).push_back(vd);
                (*pointmap).insert(datain(*(vd->point),vd));
                if(p_tmp->y() <  vd->point->y())
                        *p_tmp = *(vd->point);
        }
        in.close();

        cout << "\n- - - - Data input is finished successfully !! " << endl;
        cout << "          点云文件名称为   " << file_path << endl;
        cout << "          点云文件所含点数为     " << point_list->size() << endl;
        
}
//初始网格曲面重建
void ring_extending_initial_surface(Point p,
                                    int k,
                                    Tree *kd_tree,
                                    double e,
                                    Point_input *pointmap,
                                    vector<Point> *border,
                                    vector<VertexData*> *point_list){

	vector<Point> knn_1;
     	
       	Neighbor_search search(*kd_tree, p, INITIAL_SIZE);
       	for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
                knn_1.push_back(it->first);
        }

 
        Tree kd_tree_1;
        for(auto it = knn_1.begin(); it != knn_1.end(); it++){

                kd_tree_1.insert(*it);

        }
      
        coco_point_save_to_file(&knn_1);
        //提取边界点链表
        vector<Point> *border_tmp = new vector<Point>();
        coco_border_kboundary_extract_boundary (border_tmp, &knn_1, 30, e, &kd_tree_1);
        border->swap(*border_tmp);
        border_tmp->clear();
        vector<Point>().swap(*border_tmp);
        delete border_tmp;
        
        cout << "\t border size is  " << border->size() << endl;
        coco_point_save_to_file_2th(border);

        //将待重建点进行标记,tag = 2 表示该点在后续重建步骤中进行面片选择。state = 1 表示该点为已重建点
        vector<VertexData*> vl ; 
        coco_pl_to_vl(&knn_1, &vl, pointmap);
        coco_mark_vertexdata_recon(&vl);

        //对边界点进行扩增，添加保护点
        vector<Point> border_extend;
     
        coco_border_extend_for_assure(border, &border_extend, kd_tree, point_list, pointmap);

        // vector<VertexData*>border_extend_vl ;
        //  coco_pl_to_vl(&border_extend, &border_extend_vl, pointmap);
        //  coco_vertexdata_mark_border_near(&border_extend_vl);
        
        coco_point_save_to_file_2th(&border_extend);
        
        knn_1.insert(knn_1.end(), border_extend.begin(), border_extend.end()); //将边界点扩增的样点添加进重建点链表。
        
        coco_point_save_to_file_2th(&knn_1);


        //初始网格开始重建。
  
        PointCloud A = PointCloud(&knn_1);
        
        (A).delaunay();
        cerr << "\t Dealunay is done："<< endl;
        (A).all_amenta_needles();
        cerr << "\t Amenta calculate is done："<< endl;
        (A).facet_filter();
        cerr << "\t facet filter is done："<< endl;
        //   finish = clock();
        A.output_facet();
        // cerr << "coco重建 所用时间：" << delta_time(start, finish) << " 秒" << endl;

        vector<FaceData*> *fl_global = new vector<FaceData*>();
        coco_facet_to_face(&(A.face_list),fl_global,pointmap);
        vector<FaceData*> fl_out;
        facet_selection(fl_global, &fl_out, point_list, pointmap);

        coco_vertexdata_reset_mark(&vl);
        
        coco_facelist_print_multi(0, point_list, &fl_out);
        //fl_global 需要深度释放
        coco_facedata_free(fl_global);
        fl_out.clear();
        vector<FaceData*>().swap(fl_out);
        // coco_facedata_free(&fl_out);
        cout << "- - - - Initial surface is reconstructed  !! " << endl;
}

//波前扩展重建
void ring_extending_adding(vector<Point> *border,
                           Tree *kd_tree,
                           vector<VertexData*>* point_list,
                           Point_input *pointmap,
                           double e,
                           int loop){


        cout << "\n>>>>>> loop " << loop << "  is runing...." << endl;
        //获取待重建点
        vector<Point> recons_pl;
        coco_border_extend_for_recons(border, &recons_pl, kd_tree, point_list, pointmap);
        
        cout <<"\t the point to be recons size  is  " <<recons_pl.size() << endl;
          coco_point_save_to_file(&recons_pl);

          //将上次环形重建的外环边界点保存，即为该次环形重建的内环
          vector<Point>border_inner_to_mark;
          border_inner_to_mark.swap(*border);
        //对border进行内存释放
        border->clear();
        vector<Point>().swap(*border);

        
        //对待重建点进行标记
        vector<VertexData*> vl ; 
        coco_pl_to_vl(&recons_pl, &vl, pointmap);
        coco_mark_vertexdata_recon(&vl);

     
        cout <<"\t vd mark reconstruction  is done " << endl;
 
        //对待重建点重新进行建树
        Tree kd_tree_2 ;
        for(auto it = recons_pl.begin(); it != recons_pl.end(); it++){

                kd_tree_2.insert(*it);

        }
      
        //提取待重建点的边界点。将边界点重新存入border链表
        vector<Point> *border_tmp = new vector<Point>();
        coco_border_kboundary_extract_boundary (border_tmp, &recons_pl, 30, e, &kd_tree_2);
        border->swap(*border_tmp);
        border_tmp->clear();
        vector<Point>().swap(*border_tmp);
        
        cout << "\t border extract is done" << endl;
        cout << "\t border size is " << border->size() << endl;
                
        //对边界点进行扩增，将扩增的点作为边界点的保护点
        vector<Point> border_extend;
        coco_border_extend_for_assure(border, &border_extend, kd_tree, point_list, pointmap);
        coco_point_save_to_file_2th(&border_extend);
        //cout << "border_extend size  is " << border_extend.size() << endl;

        //将边界点扩增的样点添加进待重建点链表。
        recons_pl.insert(recons_pl.end(), border_extend.begin(), border_extend.end());
        border_extend.clear();
        vector<Point>().swap(border_extend);

        cout << "\t recons_pl size after add is " << recons_pl.size() << endl;

                
        PointCloud B = PointCloud(&recons_pl);
        
        (B).delaunay();
        cerr << "\t Dealunay is done："<< endl;
        (B).all_amenta_needles();
        cerr << "\t Amenta calculate is done："<< endl;
        (B).facet_filter();
        cerr << "\t facet filter is done："<< endl;


             
        vector<FaceData*>* fl_global = new vector<FaceData*>();
                
        coco_facet_to_face(&(B.face_list),fl_global,pointmap);
                
        cout << "\t fl_global size is " << fl_global->size() << endl;
                
        // coco_facedata_free(&fl_out);
        // fl_out = new vector<FaceData*>();

        vector<FaceData*> fl_out;
        facet_selection(fl_global, &fl_out, point_list, pointmap);
        cout << "\t facet selection is done " << endl;
                
        coco_facelist_print_multi(loop, point_list, &fl_out);
        cout << "\t facelist output is done " << endl;
                
        coco_vertexdata_reset_mark(&vl);
        cout << "\t vd remark is done " << endl;

        vl.clear();
        vector<VertexData*>().swap(vl);
        coco_facedata_free(fl_global);
        // coco_facedata_free(&fl_out);
        fl_out.clear();
        vector<FaceData*>().swap(fl_out);

        //标记环形重建的内环点
        coco_border_inner_mark(&border_inner_to_mark, kd_tree,  point_list, pointmap);
                
}


//鳞式波前扩展算法
void ring_extending_advancing_adding(vector<Point> *border,
                           Tree *kd_tree,
                           vector<VertexData*>* point_list,
                           Point_input *pointmap,
                           double e,
                           int loop){



        cout << "\n>>>>>> loop " << loop << "  is  runing...." << endl;

          
        vector<Point>pl_extend;
        vector<Point>pl_recons;
        int border_size = border->size();

        if (border_size < 5)
                return ;
        //   int times = 0;
        for (int i = 0; i < border_size; i++){

                Point p = border->at(i);
                ring_extending_get_reconstruction_data(p, &pl_recons,  &pl_extend, kd_tree, point_list, pointmap);
                ring_extending_point_reconstruction(&pl_recons, kd_tree, point_list, pointmap, e, loop*1000+i);

        }
 

        //将上次环形重建的外环边界点保存，即为该次环形重建的内环
        vector<Point>border_inner_to_mark;
        border_inner_to_mark.swap(*border);
        
        //对待重建点重新进行建树
        Tree kd_tree_2 ;
        for(auto it = pl_extend.begin(); it != pl_extend.end(); it++){

                kd_tree_2.insert(*it);

        }

        //对本次环重建的点求取边界样点。
        border->clear();
        vector<Point>().swap(*border);
        coco_border_kboundary_extract_boundary (border, &pl_extend, 30, e, &kd_tree_2);

        //标记环形重建的内环点
        coco_border_inner_mark(&border_inner_to_mark, kd_tree,  point_list, pointmap);
        cout << "border size of inner to mark is :     " << border_inner_to_mark.size() << endl;

}
        
//获取波前扩展的待重建点
void ring_extending_get_reconstruction_data(Point p, vector<Point>*pl_recons,  vector<Point> *pl_extend, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap){

        pl_recons->clear();
        vector<Point>().swap(*pl_recons);
        VertexData* vd = coco_get_pointdata(p,pointmap);

        if ((vd->tag == 3) || (vd->flag == 1))
                return ;
        
        
        Neighbor_search search(*kd_tree, p, EXTENDING_SIZE);
        for(Neighbor_search::iterator itra = search.begin(); itra != search.end(); ++itra){
                //检测该点的标记。
                if(coco_border_extend_point_test(itra->first,  pointmap)){
                        pl_extend->push_back((itra->first));
                        pl_recons->push_back(itra->first);
                }
        }
}

//对波前点进行重建
void ring_extending_point_reconstruction(vector<Point>*recons_pl,
                                         Tree *kd_tree,
                                         vector<VertexData*>* point_list,
                                         Point_input *pointmap,
                                         double e,
                                         int times){


        if (recons_pl->size() < 5 )
                return;
        //对待重建点进行标记
        vector<VertexData*> vl ; 
        coco_pl_to_vl(recons_pl, &vl, pointmap);
        coco_mark_vertexdata_recon(&vl);

        cout << "reconstruvtion data size is :    " << recons_pl->size() << endl;
        cout <<"\t vd mark reconstruction  is done " << endl;

        //对待重建点重新进行建树
        Tree kd_tree_2 ;
        for(auto it = recons_pl->begin(); it != recons_pl->end(); it++){

                kd_tree_2.insert(*it);

        }
      
        //提取待重建点的边界点。
        vector<Point> *border_tmp = new vector<Point>();
        coco_border_kboundary_extract_boundary (border_tmp, recons_pl, 30, e, &kd_tree_2);
        //  border->swap(*border_tmp);
   
        cout << "\t border extract is done" << endl;
        //   cout << "\t border size is " << border->size() << endl;
                
        //对边界点进行扩增，将扩增的点作为边界点的保护点
        vector<Point> border_extend;
        coco_border_extend_for_assure(border_tmp, &border_extend, kd_tree, point_list, pointmap);
        coco_point_save_to_file_2th(&border_extend);
        //cout << "border_extend size  is " << border_extend.size() << endl;

        //将边界点扩增的样点添加进待重建点链表。
        recons_pl->insert(recons_pl->end(), border_extend.begin(), border_extend.end());
        border_extend.clear();
        vector<Point>().swap(border_extend);

        cout << "\t recons_pl size after add is " << recons_pl->size() << endl;

                
        PointCloud B = PointCloud(recons_pl);
        
        (B).delaunay();
        cerr << "\t Dealunay is done："<< endl;
        (B).all_amenta_needles();
        cerr << "\t Amenta calculate is done："<< endl;
        (B).facet_filter();
        cerr << "\t facet filter is done："<< endl;


             
        vector<FaceData*>* fl_global = new vector<FaceData*>();
                
        coco_facet_to_face(&(B.face_list),fl_global,pointmap);
                
        cout << "\t fl_global size is " << fl_global->size() << endl;
                
        // coco_facedata_free(&fl_out);
        // fl_out = new vector<FaceData*>();

        vector<FaceData*> fl_out;
        facet_selection(fl_global, &fl_out, point_list, pointmap);
        cout << "\t facet selection is done " << endl;
                
        coco_facelist_print_multi(times, point_list, &fl_out);
        cout << "\t facelist output is done " << endl;
                
        coco_vertexdata_reset_mark(&vl);
        cout << "\t vd remark is done " << endl;

        vl.clear();
        vector<VertexData*>().swap(vl);
        coco_facedata_free(fl_global);
        // coco_facedata_free(&fl_out);
        fl_out.clear();
        vector<FaceData*>().swap(fl_out);
        border_tmp->clear();
        vector<Point>().swap(*border_tmp);
        delete border_tmp;

}
