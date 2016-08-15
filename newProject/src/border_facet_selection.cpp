#include "border_facet_selection.h"


void facet_selection(vector<FaceData*> *face_list_in,vector<FaceData*> *face_list_out, vector<VertexData*> *point_list, Point_input *pointmap){

        int face_list_in_size = face_list_in->size();
        //for (auto it = (*face_list_in).begin(); it != (*face_list_in).end();it++){
        for(int i = 0; i < face_list_in_size; i++){
                FaceData *facedata = face_list_in->at(i);
                Face face = *(facedata->face);
                int p0_id = get<0>(face);
                int p1_id = get<1>(face);
                int p2_id = get<2>(face);
       
                VertexData* v0 = point_list->at(p0_id);
                VertexData* v1 = point_list->at(p1_id);
                VertexData* v2 = point_list->at(p2_id);

                if (v0->tag == 2 || v1->tag == 2 || v2->tag == 2){

                        (*face_list_out).push_back(facedata);

                }

        }

}


void coco_facelist_print_multi(int i, vector<VertexData*> *point_list, vector<FaceData*> *face_list){



        string str1="../lib/output_facet_",str2=".stl";
        stringstream ss;
        ss<<i; 
        string s1 = ss.str();
        string str=str1+s1+str2;
         
        ofstream outfacet (str.c_str());
        if (!outfacet) {
                cerr << "error: unable to open outfacet file: "
                     << outfacet
                     << endl;
		exit(1);
        }

        outfacet << "solid SURFACE" <<endl;

      
        for (auto it = (*face_list).begin(); it != (*face_list).end();it++){

                FaceData *facedata = *it;
                Face face = *(facedata->face);
                
                /**************生成STL文件**************/
                if (1){
                        outfacet << "facet normal " << Point(0,0,0) << endl;
                        outfacet << "outer loop" << endl;
                        outfacet << "vertex " << *(point_list->at(get<0>(face))->point) << endl;
                        outfacet << "vertex " << *(point_list->at(get<1>(face))->point) << endl;
                        outfacet << "vertex " << *(point_list->at(get<2>(face))->point) << endl;
                        
                        outfacet << "endloop" << endl;
                        outfacet << "endfacet" << endl;
                
                }
              
        }
        

        outfacet << "ensolid SURFACE" <<endl;
        outfacet.close();

        
}

//����
void coco_vertexdata_reset_mark(vector<VertexData*> *vl){

        for(auto it = vl->begin(); it != vl->end(); it++){

                VertexData* vd = *it;
                vd->tag = -1;
                // vd->state = 1;
        }

}

void coco_vertexdata_mark__border_near(vector<VertexData*> *vl){

        for(auto it = vl->begin(); it != vl->end(); it++){

                VertexData* vd = *it;
                vd->flag = 1; //flag = 1 ��ʾ�õ�Ϊ�߽籣����
                // vd->state = 1;
        }

}



//���ڲ�ǰ���k���ڵ㼯������state �� tag ��־���������֡�k���ڵ㼯�е����ؽ����state Ϊ1, tag Ϊ 3, δ�ؽ����state Ϊ -1. tag ��Ϊ 2.
bool coco_border_extend_point_test(Point p, Point_input *pointmap){

        bool res = false;
        VertexData *vd = coco_get_pointdata(p,pointmap);
        //  vd->tag = 2;
        if (vd->state != 1 && vd->tag != 2){
                res = true;
                vd->tag = 2;
        }
        else
                vd->tag = 3;
     
        return res;
        
}


//�Ի����ؽ�������ڻ��߽���б�ǡ�ͨ����ǿ�ʹ�´λ����ؽ�ʱ��ֻ���⻷�����ؽ����������
void coco_border_inner_mark(vector<Point> *pl, Tree *kd_tree, vector<VertexData*>* point_list, Point_input *pointmap){

        //��ȡ�߽���������k���ڡ�
        int pl_size = pl->size(); 
   
        for(int i = 0; i < pl_size ;i++){
                Point p = pl->at(i);

                Neighbor_search search(*kd_tree, p, 10);
                for(Neighbor_search::iterator itra = search.begin(); itra != search.end(); ++itra){
                        VertexData* vd = coco_get_pointdata(itra->first, pointmap);
                        vd->flag = 1;
                        
                }
        }
}
