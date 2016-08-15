#include "test_incrementalcoco.h"
extern "C"
using namespace std;

int IncTest(){

        //点表，用于存储输入的点云数据
        vector<VertexData*> *point_list = new vector<VertexData*>();
        //用于建立样点与点数据之间的链接管理，可以进行双向查找
        Point_input *Pointmap = new Point_input;
        //预设点，用于挑选输入点云中的初始点，后期应该将该数据抛弃！！
        Point p_tmp = Point(-10000,-10000,-10000);

        //设定初始网格的样点数量
        int initMeshNumber = 30;
        //kd树索引
        Tree kd_tree ;

        //输入文件的目录
        string file_path = "../../test_data/input/foot.asc";

        //将点云数据读入内存
        ring_extending_file_input(&p_tmp, file_path, &kd_tree, Pointmap, point_list);

        IncLocal_initialMesh(p_tmp, initMeshNumber,&kd_tree, Pointmap, point_list);
        
        coco_pointdata_free(point_list);

        delete point_list;
        delete Pointmap;
}
