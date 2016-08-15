#include "test_ring_extending_coco.h"
#include "ring_extending_coco.h"
using namespace std;

static double delta_time(clock_t start, clock_t finish)
{
        double a = start;
        double b = finish;
        double c = CLOCKS_PER_SEC;
        return (b - a) / c;
}

int test_ring_extending(){

        /*调用外部程序，对lib目录下次重建文件进行清空*/

        int call = system("rm -rf ../lib/*");
        
        /******************/
        cout << ">>>>>>  Test < Ring Extending Cocone > is begining ....>>>>>" << endl;
        cout << endl;
        vector<VertexData*> *point_list = new vector<VertexData*>();
     
        Point_input *Pointmap = new Point_input;
        
        Point p_tmp = Point(-10000,-10000,-10000);
        gdouble e = M_PI*0.7;  //设置边界样点判断区域点数量。
        int k = 2000; //设置初始网格样点数量
        Tree kd_tree ;

        clock_t begin, end, start, finish;
        begin = clock();
        start = clock();

        string file_path = "../../test_data/input/venus.asc";

        //将点云数据读入内存
        ring_extending_file_input(&p_tmp, file_path, &kd_tree, Pointmap, point_list);

        finish = clock();
        cout << "\t >----> 点云输入时间为   " << delta_time(start,finish) << endl;

        //初始网格重建
        vector<Point> *border = new vector<Point>();
        ring_extending_initial_surface(p_tmp, k, &kd_tree, e, Pointmap, border, point_list);

        int loop = 0;
        while(loop++ < 100){

                //波前扩展重建
                // ring_extending_adding(border, &kd_tree, point_list, Pointmap, e,loop);
                ring_extending_advancing_adding(border, &kd_tree, point_list, Pointmap,
 e, loop);
        }

        end = clock();
        cout << "全部重建时间为    " << delta_time(begin,end) << endl;
           
        coco_pointdata_free(point_list);

        border->clear();
        vector<Point>().swap(*border);
        delete border;

        delete Pointmap;
        
       
}
