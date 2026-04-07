
#include "iir.h"


const int NL = 11;
const double NUM[11] = {
   0.001921073607887, -0.01582233398195,  0.06173168982294,  -0.1491538305649,
     0.2463093607783,  -0.2899614251408,   0.2463093607783,  -0.1491538305649,
    0.06173168982294, -0.01582233398195, 0.001921073607887
};
const int DL = 11;
const double DEN[11] = {
                   1,   -1.832786262737,   -1.224855861324,    2.826160440524,
      1.632584744955,   -2.486457890674,   -1.233681781788,    1.058111500771,
     0.5467512933005,  -0.1849415421185,  -0.1008741467245
};
typedef struct
{
    double xhist[11];  // 改为double保持精度
    double yhist[11];  // 改为double保持精度
    double y;
} Filter_t;

Filter_t filter = {
    .xhist = {0},
    .yhist = {0}
};

// /*  hz=DEN/NUM
//     使用直接型差分方程实现：
//  *  y[n] = sum_{k=0..NL-1} NUM[k]*x[n-k] - sum_{k=1..DL-1} DEN[k]*y[n-k]
//  */
double butterOrdF(float input)
{
    double input_d = (double)input;  // 转换为double
    
    // 更新历史缓冲区
    for (int i = NL-1; i > 0; i--)
    {
        filter.xhist[i] = filter.xhist[i-1];
    }
    filter.xhist[0] = input_d;
    
    for (int i = DL-1; i > 0; i--)
    {
        filter.yhist[i] = filter.yhist[i-1];
    }
    
    // 计算分子部分 (与输入历史卷积)
    double numerator = 0.0;
    for (int k = 0; k < NL; k++)
    {
        numerator += NUM[k] * filter.xhist[k];
    }
    
    // 计算分母部分 (与输出历史卷积)
    double denominator = 0.0;
    for (int k = 1; k < DL; k++)  // 从1开始，因为DEN[0]=1
    {
        denominator += DEN[k] * filter.yhist[k];
    }
    
    // 计算当前输出: y[n] = numerator - denominator
    filter.yhist[0] = numerator - denominator;
    filter.y = filter.yhist[0];
    
    return filter.y;
}