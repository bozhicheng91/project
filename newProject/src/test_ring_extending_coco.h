#ifndef TEST_RING_EXTENDING_COCO_H
#define TEST_RING_EXTENDING_COCO_H



#define  INITIAL_SIZE 100       //初始重建区域点数
#define  EXTENDING_SIZE 100     //扩展重建点数
#define  BOR_PRO_SIZE   20         //边界点检测点数

#include <ctime>

#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <glib/gprintf.h>
#include <sstream>


#include "PointCloud.h"
#include "DataInput.h"
#include "coco.h"
#include "border.h"
#include "border_facet_selection.h"
#include "ring_extending_coco.h"

int test_ring_extending();

#endif
