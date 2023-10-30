//
// Created by kade on 2023/10/27.
//

#ifndef C_DWPR_DCDC_H
#define C_DWPR_DCDC_H


#define L (6.8f*0.000001f)
#define Co_CAP (440.0f*0.000001f)
#define R_ESR (35*0.001f)         //电容内阻
#define RL 2.5f                   //负载电阻
#define Ri 0.48f                //电流采样电阻
#define Fs  200000.0f              //开关频率
#define Fx  10000.0f               //穿越频率
#define Ts  (1.0f/Fs)              //开关周期
#define Wc (2*M_PI*Fx)
/**
 * @brief      Lists the supported DC/DC topologies
 */
typedef enum dcdc_topology_e {
    DCDC_BUCK,              /**< Non-syncrhonous Buck converter (high side switch only) */
    DCDC_BOOST,             /**< Boost converter */
    DCDC_HALF_BRIDGE,       /**< Half-bridge type converter (HRTIM Timerx Tx1 and Tx2 active and complimentary) */
    DCDC_FULL_BRIDGE        /**< Full-bridge type converter (HRTIM Timerx Tx1 and Tx2 active and complimentary, HRTIM Timery Ty1 and Ty2 active and complimentary) */
} dcdc_topology_e;


/**
 * @brief      Lists the supported DC/DC control mode
 */
typedef enum dcdc_control_mode_e {
    dcdc_voltage,               /**< Voltage mode control */
    dcdc_average_current,       /**< Average current mode control */
    dcdc_peak_current           /**< Peak current mode control */
} dcdc_control_mode_e;


/**
 * @brief      Lists the supported DC/DC control type
 */
typedef enum dcdc_ontroller_type_e {
    _3p3z,
    _2p2z
} dcdc_controller_type_e;
/**
 * @brief      DC/DC data structure content
 */
typedef struct dcdc_data {
    dcdc_topology_e topology;                     /**< The topology of the DCDC */
    dcdc_control_mode_e control_mode;
    dcdc_controller_type_e controller_type;/**< The control mode of the DCDC */
    double voltage_in;
    double voltage_out;
    double currant_max;
    double filter_coffA[3];
    double filter_coffB[4];
    double K;

    double data_output[4];
    double data_input[4];

} dcdc_data;


void
init_dcdc(dcdc_data *DCDC, dcdc_topology_e topology, dcdc_control_mode_e mode, dcdc_controller_type_e type, double Vin,
          double Vout,
          double Imax);

void calculate_controller_coff(dcdc_data *DCDC);

double calculate_result_controller(dcdc_data *DCDC, double new_data);

#endif //C_DWPR_DCDC_H
