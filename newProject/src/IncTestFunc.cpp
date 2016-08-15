#include "IncTestFunc.h"

using namespace std;


//数据输出到硬盘借口，利用函数重载
void IncTestFunc_outputData(vector<Point>*pl, int times){

        string str1="./outputPointdata_",str2=".asc";
        stringstream ss;
        ss<<times; 
        string s1 = ss.str();
        string str=str1+s1+str2;
         
        ofstream out (str.c_str());
        if (!out) {
                cerr << "error: unable to open outPointdata file: "
                     << out
                     << endl;
                exit(1);
        }

        for(auto it = pl->begin(); it != pl->end(); it++)
                out << *it << endl;
}

void IncTestFunc_outputData(vector<VertexData*>*pointlist,vector<FaceData*>*facelist,  int times){

 string str1="./output_facet_",str2=".stl";
        stringstream ss;
        ss<<times; 
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

      
        for (auto it = (*facelist).begin(); it != (*facelist).end();it++){

                FaceData *facedata = *it;
                Face face = *(facedata->face);
                
                /**************鐢熸垚STL鏂囦欢**************/
                if (1){
                        outfacet << "facet normal " << Point(0,0,0) << endl;
                        outfacet << "outer loop" << endl;
                        outfacet << "vertex " << *(pointlist->at(get<0>(face))->point) << endl;
                        outfacet << "vertex " << *(pointlist->at(get<1>(face))->point) << endl;
                        outfacet << "vertex " << *(pointlist->at(get<2>(face))->point) << endl;
                        
                        outfacet << "endloop" << endl;
                        outfacet << "endfacet" << endl;
                
                }
              
        }
        

        outfacet << "ensolid SURFACE" <<endl;
        outfacet.close();

}


void IncTestFunc_outputData(vector<VertexData*>*pointlist,int times){

        string str1="./pointlist-",str2=".asc";
        stringstream ss;
        ss<<times; 
        string s1 = ss.str();
        string str=str1+s1+str2;
         
        ofstream out (str.c_str());
        if (!out) {
                cerr << "error: unable to open outPointdata file: "
                     << out
                     << endl;
                exit(1);
        }



        for (auto itra = pointlist->begin(); itra != pointlist->end(); itra++){

                out << *((*itra)->point) << endl;

        }


}
