#include "DataInput.h"
#include "PointCloud.h"
using namespace std;

FaceData *
new_FaceData(){

        FaceData *fd = new FaceData;
        fd->face = new Face;
        fd->normal = new Vector;
        fd->edges = new Data_list;
        fd->id = -1;
        fd->flag = 0;
        fd->tag = 0;
        fd->state = -1;
        fd->ang = 1;
        return fd;

}


EdgeData *
new_EdgeData(){

        EdgeData *ed = new EdgeData;
        ed->edge = new Edge;
        ed->faces = new Data_list;
        ed->id = -1;
        ed->flag = 0;
        return ed;

}


VertexData *
new_VertexData(){

        VertexData *vd = new VertexData;
        vd->point = new Point;
        vd->edges = new Data_list;
        vd->faces = new Data_list;

        vd->id = -1;
        vd->flag = 0;
        vd->tag = -1;
        vd->state = 0;

        vd->l_tag = -1;
        vd->l_flag = 0;
        vd->l_state = 0;

        return vd;
}




void coco_facedata_free(std::vector<FaceData*> *face_list){
        
        for (auto it = (*face_list).begin(); it != (*face_list).end(); it++){

                if(NULL != *it)
                        {
                                delete (*it)->face;
                                delete (*it)->normal;
                                (*it)->edges->clear();
                                vector<int>().swap((*(*it)->edges));
                                delete (*it)->edges;
                                delete *it;
                                *it = NULL;
                        }
                
        }
        (*face_list).clear();
        vector<FaceData*>().swap(*face_list);
        // delete face_list;
        // face_list = NULL;
}



void coco_edgedata_free(std::vector<EdgeData*> *edge_list){
        
        for (auto it = (*edge_list).begin(); it != (*edge_list).end(); it++){

                if(NULL != *it)
                        {

                                delete (*it)->edge;
                                
                                (*it)->faces->clear();
                                vector<int>().swap((*(*it)->faces));
                                delete (*it)->faces;
                                
                                delete *it;
                                *it = NULL;
                        }
                
        }
        (*edge_list).clear();
        vector<EdgeData*>().swap(*edge_list);
        // delete edge_list;
}



void coco_pointdata_free(std::vector<VertexData*> *point_list){
        
        for (auto it = (*point_list).begin(); it != (*point_list).end(); it++){

                if(NULL != *it)
                        {

                                delete (*it)->point;
                                
                                (*it)->edges->clear();
                                vector<int>().swap((*(*it)->edges));
                                delete (*it)->edges;

                                (*it)->faces->clear();
                                vector<int>().swap((*(*it)->faces));
                                delete (*it)->faces;

                                
                                delete *it;
                                *it = NULL;
                        }
                
        }
        (*point_list).clear();
        vector<VertexData*>().swap(*point_list);
        // delete point_list;
}


void coco_facedataCopy(FaceData* target, FaceData* src){
        *(target->face) = *(src->face);
        *(target->normal) = *(src->normal);
        *(target->edges) = *(src->edges);
        target->id = src->id;
        target->flag = src->flag;
        target->tag = src->tag;
        target->state = src->state;
        target->ang = src->ang;

}
