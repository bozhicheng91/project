#ifndef TEST_RING_EXTENDING_COCO_H
#define TEST_RING_EXTENDING_COCO_H



#define  INITIAL_SIZE 100       //��ʼ�ؽ��������
#define  EXTENDING_SIZE 100     //��չ�ؽ�����
#define  BOR_PRO_SIZE   20         //�߽�������

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
