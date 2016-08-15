#include "test_single_coco.h"

using namespace std;


static double delta_time(clock_t start, clock_t finish)
{
        double a = start;
        double b = finish;
        double c = CLOCKS_PER_SEC;
        return (b - a) / c;
}



int coco_single_reconstruction()
{


        Tree kd_tree ;
 
        Point_input *Pointmap = new Point_input;
        // Edge_input *Edgemap = new Edge_input;
        // Face_input *Facemap = new Face_input;

        vector<VertexData*> *point_list = new vector<VertexData*>();
        //  vector<EdgeData*> *edge_list = new vector<EdgeData*>();
        vector<FaceData*> *fl_global = new vector<FaceData*>();
        
        Point p_tmp = Point(10000,10000,10000);

        clock_t begin, end, start, finish;
        begin = clock();
        start = clock();

        //**************点云读入内存*****************//
        string file_path = "../../test_data/input/happy-5M.asc";
      
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
                kd_tree.insert(*(vd->point));
                
                vd->id = i++;
                (*point_list).push_back(vd);
                (*Pointmap).insert(datain(*(vd->point),vd));
                if(p_tmp.z()  <  vd->point->z())
                        p_tmp = *(vd->point);
        }
        in.close();

        finish = clock();
        cerr << "点云读入内存 所用时间：" << delta_time(start, finish) << " 秒" << endl;
        /*******************************************/

   
        start = clock();
        //初次重建。初始重建网格形成。
	vector<Point> knn_1;
     	
       	Neighbor_search search(kd_tree, p_tmp, 5000);
       	for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
                knn_1.push_back(it->first);
        }

 
        Tree kd_tree_1 ;
        for(auto it = knn_1.begin(); it != knn_1.end(); it++){

                kd_tree_1.insert(*it);

        }
        cout << "ok 2 " << endl;
        coco_point_save_to_file(&knn_1);
        gdouble e = M_PI*0.7;  //设置边界样点判断区域点数量。
        vector<Point> *border = new vector<Point>();
        coco_border_kboundary_extract_boundary (border, &knn_1, 50, e, &kd_tree_1);
        coco_point_save_to_file_2th(border);

        vector<VertexData*> vl ; 
        coco_pl_to_vl(&knn_1, &vl, Pointmap);
        coco_mark_vertexdata_recon(&vl);

        vector<Point> border_extend;
        coco_border_extend_for_assure(border, &border_extend, &kd_tree, point_list, Pointmap);
        
        coco_point_save_to_file_2th(&border_extend);
        
        knn_1.insert(knn_1.end(), border_extend.begin(), border_extend.end()); //将边界点扩增的样点添加进重建点链表。
        
        coco_point_save_to_file_2th(&knn_1);


        

   
	

        start = clock();
        //初始网格开始重建。
        //  PointCloud *A = new PointCloud(file_path);
        PointCloud A = PointCloud(&knn_1);
        
        (A).delaunay();
        cerr << "Dealunay is done："<< endl;
        (A).all_amenta_needles();
        cerr << "Amenta calculate is done："<< endl;
        (A).facet_filter();
        cerr << "facet filter is done："<< endl;
        finish = clock();
        A.output_facet();
        cerr << "coco重建 所用时间：" << delta_time(start, finish) << " 秒" << endl;

        
        coco_facet_to_face(&(A.face_list),fl_global,Pointmap);
        vector<FaceData*> fl_out;
        facet_selection(fl_global, &fl_out, point_list, Pointmap);

        //  coco_pl_to_vl(&knn_1, &vl, Pointmap);
        coco_vertexdata_reset_mark(&vl);
        
        // coco_facelist_print(point_list, &fl_out);
        coco_facelist_print_multi(0, point_list, &fl_out);
        
        fl_out.clear();
        vector<FaceData*>().swap(fl_out);

     
        finish = clock();
        cerr << "初始网格重建  所用时间： " << delta_time(start, finish) << " 秒" << endl;


        //>>>>>>>>>>>>增量重建开始>>>>>>>>>>>>>>>>>//
        int loop = 0;
      
        vector<Point> recons_pl;
        while(loop++ < 30){

                
                //待重建点集
                start = clock();
                
                cout <<"\tThe loop of  " << loop <<" is running" << endl;

                //获取待重建点集
                recons_pl.clear();
                vector<Point>().swap(recons_pl);
                coco_border_extend_for_recons(border, &recons_pl, &kd_tree, point_list, Pointmap);

                cout <<"the point to be recons size  is  " <<recons_pl.size() << endl;
                
                border->clear();
                vector<Point>().swap(*border);
                

                //对待重建点进行标记
                coco_pl_to_vl(&recons_pl, &vl, Pointmap);
                coco_mark_vertexdata_recon(&vl);
                
                cout <<"vd mark reconstruction  is done " << endl;
 
                //对待重建点重新进行建树
                Tree kd_tree_2 ;
                knn_1.clear();
                vector<Point>().swap(knn_1);
                for(auto it = recons_pl.begin(); it != recons_pl.end(); it++){

                        kd_tree_2.insert(*it);

                }
      
                //提取点集的边界点。
                coco_border_kboundary_extract_boundary (border, &recons_pl, 50, e, &kd_tree_2);

                cout << "border extract is done" << endl;
                // coco_point_save_to_file(border);
                
                cout << "border size is " << border->size() << endl;
                
                border_extend.clear();
                vector<Point>().swap(border_extend);
                coco_border_extend_for_assure(border, &border_extend, &kd_tree, point_list, Pointmap);
                 coco_point_save_to_file_2th(&border_extend);
                cout << "border_extend size  is " << border_extend.size() << endl;
 
                recons_pl.insert(recons_pl.end(), border_extend.begin(), border_extend.end()); //将边界点扩增的样点添加进重建点链表。

                cout << "recons_pl size after add is " << recons_pl.size() << endl;

                
                PointCloud B = PointCloud(&recons_pl);
        
                (B).delaunay();
                cerr << "Dealunay is done："<< endl;
                (B).all_amenta_needles();
                cerr << "Amenta calculate is done："<< endl;
                (B).facet_filter();
                cerr << "facet filter is done："<< endl;


             
                fl_global = new vector<FaceData*>();
                
                coco_facet_to_face(&(B.face_list),fl_global,Pointmap);
                
                cout << "fl_global size is " << fl_global->size() << endl;
                
                // coco_facedata_free(&fl_out);
                // fl_out = new vector<FaceData*>();

                
                facet_selection(fl_global, &fl_out, point_list, Pointmap);
                cout << "facet selection is done " << endl;
                
                coco_facelist_print_multi(loop, point_list, &fl_out);
                cout << "facelist output is done " << endl;
                
                coco_vertexdata_reset_mark(&vl);
                cout << "vd remark is done " << endl;


            
                coco_facedata_free(fl_global);

                vl.clear();
                vector<VertexData*>().swap(vl);

                fl_out.clear();
                vector<FaceData*>().swap(fl_out);
                
                finish = clock();
                cerr << "\tloop  "<< loop <<" 重建" <<" 所用时间： " << delta_time(start, finish) << " 秒" << endl;
                
        }
        
        end = clock();
        cerr <<"全部重建完成  所用时间： " << delta_time(begin, end) << " 秒" << endl;
          
         coco_facedata_free(fl_global);
         // coco_edgedata_free(edge_list);
         coco_pointdata_free(point_list);
        return 0;
        
}
