//
//  Created by kade on 2023/10/23.
//
//

/**
 * 该文件用于计算电源的环路控制，在书写的时候也要注意格式能直接调用。
 *
 * 电流和占空比的传递函数Gid(S)=Vg(1+RCs)/(R+Ls+RCLs^2);
 * 电压和电流的传递函数Gvi(S)=R/(1+RCs)
 *
 * 穿越频率设置为10分之1的开关频率
 * 一极点一零点补偿网络的低频零点在谐振频率的左侧附近。
 * 极点的选择：相位欲度=arctan(fs/fz1)-arctan(fs/fp1)，相位欲度选取为60度
 *
 * 其实buck的传递函数是已知的函数了，我需要做的就是计算出需要补偿的零极点。在buck中，使用电流内环加上电压外环的控制方式。
 * 电流环使用一个“一极点一零点”补偿网络。电压还可以只使用一个pi控制器来进行补偿。
 *
 * 在本例程中，先学会计算一极点一零点补偿网络的零点和极点，以及直流增益。
 */
#include "loop_control.h"
#include "dwpr_dcdc.h"
#include <math.h>


dcdc_data buck_cap;

//暂时假定负载为10欧

void calculate_buck_transfer_function(double Vout, double Vin, double Iout) {


    double fp;
    double fz;
    double fc = 5000;
    fp = (tan(50 * M_PI / 180.0f) + sqrt(pow(tan(50 * M_PI / 180.0f), 2) + 1.0));
    fp = fp * fc;
    fz = (fc * fc) / fp;

}


/**
 * 该代码出尝试计算电流型step_by_step 的buck电路的数字模型，计算各个系数。
 */
void __2p2zcalculate(dcdc_data *DCDC) {

}