//
// Created by kade on 2023/10/23.
//

#include <stdio.h>
#include "loop_control.h"
#include "dwpr_dcdc.h"
#include <math.h>

extern dcdc_data buck_cap;

int main() {
    double a = sqrt(4);
    init_dcdc(&buck_cap, DCDC_BUCK, dcdc_average_current, _3p3z, 12, 3.3, 2);
    calculate_controller_coff(&buck_cap);
    return 0;
}