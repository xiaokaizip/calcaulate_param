//
// Created by kade on 2023/10/27.
//

#include "dwpr_dcdc.h"
#include <math.h>

void
init_dcdc(dcdc_data *DCDC, dcdc_topology_e topology, dcdc_control_mode_e mode, dcdc_controller_type_e type, double Vin,
          double Vout,
          double Imax) {
    DCDC->currant_max = Imax;
    DCDC->voltage_in = Vin;
    DCDC->voltage_out = Vout;
    DCDC->topology = topology;
    DCDC->control_mode = mode;
    DCDC->controller_type = type;
}

void calculate_controller_coff(dcdc_data *DCDC) {
    double Wcp0, Wcp1, Wcp2, Wcz0, Wcz1, Wcz2, R1, R2, Q;
    double Fcp0, Fcp1, Fcp2, Fcz0, Fcz1, Fcz2;
    double a;
    double boost = 0;//相位提升

    if (DCDC->controller_type == _2p2z) {
        /**
         * 二型环路补偿的形式为 Hc(S)=(Wcp0/s)*((1+s/Wcz1)/(1+2/Wcp1);
         * 其中，Wcp1=1/(Resr*Co);
         * Wcz1=0.4*pi*fx;
         * Wcp0=(1.23fx*Ri*(Lo+0.32*Ro*Ts)*R1*R2)/(Lo*Ro);
         * R1=sqrt(1-4*fx^2*Ts^2+16*fx^4*Ts^4)
         * R2=sqrt(1+(39.38*Co^2*fx^2*Lo^2*Ro^2)/(Lo+0.32*Ro*Ts)^2)
         *
         *
         */

        Wcp1 = 1.0f / (R_ESR * Co_CAP);
        Wcz1 = 0.4f * M_PI * Fx;
        R1 = sqrt(1 - pow(2 * Fx * Ts, 2) + pow(2 * Fx * Ts, 4));
        R2 = sqrt(1 + (39.48 * pow(Co_CAP * Fx * L * RL, 2)) / pow(L + 0.32 * RL * Ts, 2));
        Wcp0 = (1.23f * Fx * Ri * (L + 0.32 * RL * Ts) * R1 * R2) / (L * RL);

        double temp = 2 + Ts * Wcp1;
        DCDC->filter_coffB[0] = (Ts * Wcp0 * Wcp1 * (2 + Ts * Wcz1)) / (2 * temp * Wcz1);
        DCDC->filter_coffB[1] = (Ts * Ts * Wcp0 * Wcp1) / temp;
        DCDC->filter_coffB[2] = (Ts * Wcp0 * Wcp1 * (-2 + Ts * Wcz1)) / (2 * temp * Wcz1);
        DCDC->filter_coffB[3] = 0;
        DCDC->filter_coffA[0] = 4 / temp;
        DCDC->filter_coffA[1] = (-2 + Ts * Wcp1) / temp;
        DCDC->filter_coffA[2] = 0;
    } else if (DCDC->controller_type == _3p3z) {
        /**
         * 三型环路补偿的形式为 Hc(S)=(Wcp0/s)*((1+s/Wcz1)*(1+s/Wcz2)/((1+s/Wcp1)*(1+s/Wcp2));
         * 其中，Wcp1=1/(Resr*Co);
         * Wcz1=0.4*pi*fx;
         * Wcp0=(1.23fx*Ri*(Lo+0.32*Ro*Ts)*R1*R2)/(Lo*Ro);
         * R1=sqrt(1-4*fx^2*Ts^2+16*fx^4*Ts^4)
         * R2=sqrt(1+(39.38*Co^2*fx^2*Lo^2*Ro^2)/(Lo+0.32*Ro*Ts)^2)
         *
         *
         */

        Q = RL * sqrt((double) Co_CAP / L);

        Wcz0 = 1 / (R_ESR * Co_CAP);

        Wcp1 = Wcz0;
        Wcp2 = 10 * 2 * M_PI * Fx;
        Wcz2 = Wcz1 = (1 / sqrt((double) L * Co_CAP));

//        Wcp1 = 10300 * (2 * M_PI);
//        Wcp2 = 100000 * (2 * M_PI);
//        Wcz1 = 1200 * (2 * M_PI);
//        Wcz2 = 1200 * (2 * M_PI);


        double temp_1 = Wc / Wcp1;
        double temp_2 = Wc / Wcp2;
        double temp_3 = Wcz1 / Wc;
        double temp_4 = Wc / Wcz2;


        Wcp0 = Wcz1 * pow(10, -3 / 20.0) * sqrt(1 + pow(temp_1, 2)) * sqrt(1 + pow(temp_2, 2)) /
               (sqrt(1 + pow(temp_3, 2)) * sqrt(1 + pow(temp_4, 2)));


        Fcp0 = Wcp0 / (2 * M_PI);
        Fcp1 = Wcp1 / (2 * M_PI);
        Fcp2 = Wcp2 / (2 * M_PI);
        Fcz0 = Wcz0 / (2 * M_PI);
        Fcz1 = Wcz1 / (2 * M_PI);
        Fcz2 = Wcz2 / (2 * M_PI);


        double temp1 = 2 + Ts * Wcp1;
        double temp2 = 2 + Ts * Wcp2;
        double denominator_B = 2 * Wcz1 * Wcz2 * temp1 * temp2;
        double denominator_A = temp1 * temp2;


        DCDC->filter_coffB[0] = (Ts * Wcp0 * Wcp1 * Wcp2 * (2 + Ts * Wcz1) * (2 + Ts * Wcz2)) / denominator_B;
        DCDC->filter_coffB[1] =
                (Ts * Wcp0 * Wcp1 * Wcp2 * (-4 + 3 * pow(Ts, 2) * Wcz1 * Wcz2 + 2 * Ts * (Wcz1 + Wcz2))) /
                denominator_B;
        DCDC->filter_coffB[2] =
                (Ts * Wcp0 * Wcp1 * Wcp2 * (-4 + 3 * pow(Ts, 2) * Wcz1 * Wcz2 - 2 * Ts * (Wcz1 + Wcz2))) /
                denominator_B;
        DCDC->filter_coffB[3] =
                (Ts * Wcp0 * Wcp1 * Wcp2 * (-2 + Ts * Wcz1) * (-2 + Ts * Wcz2)) /
                denominator_B;
        DCDC->filter_coffA[0] = -(-12 + pow(Ts, 2) * Wcp1 * Wcp2 - 2 * Ts * (Wcp1 + Wcp2)) / denominator_A;
        DCDC->filter_coffA[1] = (-12 + pow(Ts, 2) * Wcp1 * Wcp2 + 2 * Ts * (Wcp1 + Wcp2)) / denominator_A;
        DCDC->filter_coffA[2] = (-2 + Ts * Wcp1) * (-2 + Ts * Wcp2) / denominator_A;
    }
    while (1) {

    }
}

double calculate_result_controller(dcdc_data *DCDC, double new_data) {
//y[n]=B2*x[n-2]+B1*x[n-1]+B0*x[n]+A2*y[n-2]+A1*y[n-1]
    for (int i = 0; i < 3; i++) {
        DCDC->data_input[i] = DCDC->data_input[i + 1];
        DCDC->data_output[i] = DCDC->data_output[i + 1];
    }
    DCDC->data_input[3] = new_data;


    DCDC->data_output[3] =
            DCDC->filter_coffB[3] * DCDC->data_input[0] + DCDC->filter_coffB[2] * DCDC->data_input[1] +
            DCDC->filter_coffB[1] * DCDC->data_input[2] + DCDC->filter_coffB[0] * DCDC->data_input[3]
            + DCDC->filter_coffA[2] * DCDC->data_output[0] + DCDC->filter_coffA[1] * DCDC->data_output[1] +
            DCDC->filter_coffA[0] * DCDC->data_output[2];

    return DCDC->data_output[3];
}