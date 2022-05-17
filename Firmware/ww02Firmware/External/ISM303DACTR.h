

#ifndef ISM303DACTR_H_
#define ISM303DACTR_H_

/****************************************************************/
/* I2C Device Addresses */
#define ISM303DACTR_I2C_ACC_ADDR          0x1D
#define ISM303DACTR_I2C_MAG_ADDR          0x1E

/****************************************************************/
/** Device Identification (Who am I) **/
#define ISM303DAC_ID_XL                   0x43U
#define ISM303DAC_ID_MG                   0x40U

/****************************************************************/
/** Device Registers and Structs */
#define ISM303DAC_MODULE_8BIT_A           0x0CU
#define ISM303DAC_WHO_AM_I_A              0x0FU
#define ISM303DAC_CTRL1_A                 0x20U
typedef struct {
  uint8_t bdu                 : 1;
  uint8_t hf_odr              : 1;
  uint8_t fs                  : 2;
  uint8_t odr                 : 4;
} ism303dac_ctrl1_a_t;

#define ISM303DAC_CTRL2_A                 0x21U
typedef struct {
  uint8_t sim                 : 1;
  uint8_t i2c_disable         : 1;
  uint8_t if_add_inc          : 1;
  uint8_t fds_slope           : 1;
  uint8_t not_used_01         : 2;
  uint8_t soft_reset          : 1;
  uint8_t boot                : 1;
} ism303dac_ctrl2_a_t;

#define ISM303DAC_CTRL3_A                 0x22U
typedef struct {
  uint8_t pp_od               : 1;
  uint8_t h_lactive           : 1;
  uint8_t lir                 : 1;
  uint8_t tap_z_en            : 1;
  uint8_t tap_y_en            : 1;
  uint8_t tap_x_en            : 1;
  uint8_t st                  : 2;
} ism303dac_ctrl3_a_t;

#define ISM303DAC_CTRL4_A                 0x23U
typedef struct {
  uint8_t int1_drdy           : 1;
  uint8_t int1_fth            : 1;
  uint8_t int1_6d             : 1;
  uint8_t int1_tap            : 1;
  uint8_t int1_ff             : 1;
  uint8_t int1_wu             : 1;
  uint8_t int1_s_tap          : 1;
  uint8_t not_used_01         : 1;
} ism303dac_ctrl4_a_t;

#define ISM303DAC_CTRL5_A                 0x24U
typedef struct {
  uint8_t int2_drdy           : 1;
  uint8_t int2_fth            : 1;
  uint8_t not_used_01         : 3;
  uint8_t int2_on_int1        : 1;
  uint8_t int2_boot           : 1;
  uint8_t drdy_pulsed         : 1;
} ism303dac_ctrl5_a_t;

#define ISM303DAC_FIFO_CTRL_A             0x25U
typedef struct {
  uint8_t if_cs_pu_dis        : 1;
  uint8_t not_used_01         : 2;
  uint8_t module_to_fifo      : 1;
  uint8_t not_used_02         : 1;
  uint8_t fmode               : 3;
} ism303dac_fifo_ctrl_a_t;

#define ISM303DAC_OUT_T_A                 0x26U
#define ISM303DAC_STATUS_A                0x27U
typedef struct {
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t fifo_ths            : 1;
} ism303dac_status_a_t;

#define ISM303DAC_OUT_X_L_A               0x28U
#define ISM303DAC_OUT_X_H_A               0x29U
#define ISM303DAC_OUT_Y_L_A               0x2AU
#define ISM303DAC_OUT_Y_H_A               0x2BU
#define ISM303DAC_OUT_Z_L_A               0x2CU
#define ISM303DAC_OUT_Z_H_A               0x2DU
typedef struct
{
  int16_t x_axis;
  int16_t y_axis;
  int16_t z_axis;
}ism303dac_accel_out_t;

typedef struct
{
  float x_axis;
  float y_axis;
  float z_axis;
}ism303dac_accel_float_out_t;

#define ISM303DAC_FIFO_THS_A              0x2EU
#define ISM303DAC_FIFO_SRC_A              0x2FU
typedef struct {
  uint8_t not_used_01         : 5;
  uint8_t diff                : 1;
  uint8_t fifo_ovr            : 1;
  uint8_t fth                 : 1;
} ism303dac_fifo_src_a_t;

#define ISM303DAC_FIFO_SAMPLES_A          0x30U
typedef struct {
  uint8_t diff                : 8;
} ism303dac_fifo_samples_a_t;

#define ISM303DAC_TAP_6D_THS_A            0x31U
typedef struct {
  uint8_t tap_ths             : 5;
  uint8_t _6d_ths             : 2;
  uint8_t _4d_en              : 1;
} ism303dac_tap_6d_ths_a_t;

#define ISM303DAC_INT_DUR_A               0x32U
typedef struct {
  uint8_t shock               : 2;
  uint8_t quiet               : 2;
  uint8_t lat                 : 4;
} ism303dac_int_dur_a_t;

#define ISM303DAC_WAKE_UP_THS_A           0x33U
typedef struct {
  uint8_t wu_ths              : 6;
  uint8_t sleep_on            : 1;
  uint8_t single_double_tap   : 1;
} ism303dac_wake_up_ths_a_t;

#define ISM303DAC_WAKE_UP_DUR_A           0x34U
typedef struct {
  uint8_t sleep_dur           : 4;
  uint8_t int1_fss7           : 1;
  uint8_t wu_dur              : 2;
  uint8_t ff_dur              : 1;
} ism303dac_wake_up_dur_a_t;

#define ISM303DAC_FREE_FALL_A             0x35U
typedef struct {
  uint8_t ff_ths              : 3;
  uint8_t ff_dur              : 5;
} ism303dac_free_fall_a_t;

#define ISM303DAC_STATUS_DUP_A            0x36U
typedef struct {
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t ovr                 : 1;
} ism303dac_status_dup_a_t;

#define ISM303DAC_WAKE_UP_SRC_A           0x37U
typedef struct {
  uint8_t z_wu                : 1;
  uint8_t y_wu                : 1;
  uint8_t x_wu                : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state_ia      : 1;
  uint8_t ff_ia               : 1;
  uint8_t not_used_01         : 2;
} ism303dac_wake_up_src_a_t;

#define ISM303DAC_TAP_SRC_A               0x38U
typedef struct {
  uint8_t z_tap               : 1;
  uint8_t y_tap               : 1;
  uint8_t x_tap               : 1;
  uint8_t tap_sign            : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t tap_ia              : 1;
  uint8_t not_used_01         : 1;
} ism303dac_tap_src_a_t;

#define ISM303DAC_6D_SRC_A                0x39U
typedef struct {
  uint8_t xl                  : 1;
  uint8_t xh                  : 1;
  uint8_t yl                  : 1;
  uint8_t yh                  : 1;
  uint8_t zl                  : 1;
  uint8_t zh                  : 1;
  uint8_t _6d_ia              : 1;
  uint8_t not_used_01         : 1;
} ism303dac_6d_src_a_t;

#define ISM303DAC_FUNC_SRC_A              0x3EU
typedef struct {
  uint8_t not_used_01         : 1;
  uint8_t module_ready        : 1;
  uint8_t not_used_02         : 6;
} ism303dac_func_src_a_t;

#define ISM303DAC_FUNC_CTRL_A             0x3FU
typedef struct {
  uint8_t not_used_01         : 5;
  uint8_t module_on           : 1;
  uint8_t not_used_02         : 2;
} ism303dac_func_ctrl_a_t;

#define ISM303DAC_OFFSET_X_REG_L_M          0x45U
#define ISM303DAC_OFFSET_X_REG_H_M          0x46U
#define ISM303DAC_OFFSET_Y_REG_L_M          0x47U
#define ISM303DAC_OFFSET_Y_REG_H_M          0x48U
#define ISM303DAC_OFFSET_Z_REG_L_M          0x49U
#define ISM303DAC_OFFSET_Z_REG_H_M          0x4AU
#define ISM303DAC_WHO_AM_I_M                0x4FU
#define ISM303DAC_CFG_REG_A_M               0x60U
typedef struct {
  uint8_t md                     : 2;
  uint8_t odr                    : 2;
  uint8_t lp                     : 1;
  uint8_t soft_rst               : 1;
  uint8_t reboot                 : 1;
  uint8_t comp_temp_en           : 1;
} ism303dac_cfg_reg_a_m_t;

#define ISM303DAC_CFG_REG_B_M               0x61U
typedef struct {
  uint8_t lpf                    : 1;
  uint8_t set_rst                : 2; /* off_canc + set_freq */
  uint8_t int_on_dataoff         : 1;
  uint8_t off_canc_one_shot      : 1;
  uint8_t not_used_01            : 3;
} ism303dac_cfg_reg_b_m_t;

#define ISM303DAC_CFG_REG_C_M               0x62U
typedef struct {
  uint8_t int_mag                : 1;
  uint8_t self_test              : 1;
  uint8_t not_used_01            : 1;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
  uint8_t i2c_dis                : 1;
  uint8_t int_mag_pin            : 1;
  uint8_t not_used_02            : 1;
} ism303dac_cfg_reg_c_m_t;

#define ISM303DAC_INT_CRTL_REG_M            0x63U
typedef struct {
  uint8_t ien                    : 1;
  uint8_t iel                    : 1;
  uint8_t iea                    : 1;
  uint8_t not_used_01            : 2;
  uint8_t zien                   : 1;
  uint8_t yien                   : 1;
  uint8_t xien                   : 1;
} ism303dac_int_crtl_reg_m_t;

#define ISM303DAC_INT_SOURCE_REG_M          0x64U
typedef struct {
  uint8_t _int                    : 1;
  uint8_t mroi                   : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_x               : 1;
} ism303dac_int_source_reg_m_t;

#define ISM303DAC_INT_THS_L_REG_M           0x65U
#define ISM303DAC_INT_THS_H_REG_M           0x66U
#define ISM303DAC_STATUS_REG_M              0x67U
typedef struct {
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
} ism303dac_status_reg_m_t;

#define ISM303DAC_OUTX_L_REG_M              0x68U
#define ISM303DAC_OUTX_H_REG_M              0x69U
#define ISM303DAC_OUTY_L_REG_M              0x6AU
#define ISM303DAC_OUTY_H_REG_M              0x6BU
#define ISM303DAC_OUTZ_L_REG_M              0x6CU
#define ISM303DAC_OUTZ_H_REG_M              0x6DU

/****************************************************************/
/* Register Values */
#define DISABLED                0
#define nENABLED                0
#define ENABLED                 1


#define ACC_ODR_OFF             0
#define ACC_ODR_1HZ             8
#define ACC_ODR_12_5HZ          9
#define ACC_ODR_25HZ            10
#define ACC_ODR_SETTING(x)      x << 4


#define ACC_RANGE_2G            0
#define ACC_RANGE_4G            2
#define ACC_RANGE_8G            3
#define ACC_RANGE_16G           1
#define ACC_RANGE_SETTING(x)    x << 2

#define ACC_2G_CONVERSION(x)    x*0.061/1000
#define ACC_4G_CONVERSION(x)    x*0.122/1000
#define ACC_8G_CONVERSION(x)    x*0.244/1000
#define ACC_16G_CONVERSION(x)   x*0.488/1000

#define BDR_ENABLED               1
#define ENABLE_SLEEP_BIT          (1 << 6)
#define ENABLE_HPF_BIT            (1 << 3)
#define ENABLE_AUTO_ADDR_INC_BIT  (1 << 2)
#define ENABLE_LATCH_INT_BIT      (1 << 2)
#define ENABLE_WAKE_UP_INT1_BIT   (1 << 5)
#define ENABLE_FIFO_INT1_BIT      (1 << 1)

#define WAKE_UP_1x_ODR            1
#define WAKE_UP_2x_ODR            2
#define WAKE_UP_3x_ODR            3
#define WAKE_UP_DURATION(x)       ((x & 0x03) << 5)
#define SLEEP_DURATION(x)         (x & 0x07)


#define FIFO_MODE_DISABLED        0x00
#define FIFO_MODE_ENABLED         0x01


#define MAG_LP_POWER_MODE         (1 << 4)
#define MAG_SYS_MODE_IDLE         0x03

/****************************************************************/

/**
 * @brief - Returns the Acceleromter Range setting
 */
uint8_t ism303dactr_get_acc_range();

/**
 * @brief - Convert Raw Accelerometer Values to G's
 */
ism303dac_accel_float_out_t ism303dactr_convert_raw_values_acc( ism303dac_accel_out_t * accel_values );


/**
 * @brief - Read Accelerometer
 */
ism303dac_accel_out_t ism303dactr_read_acc();


/**
 * @brief - Read Temperature Register
 */
float ism303dactr_reading_temperature();


/**
 * @brief - Set Sleep Bit
 */
void ism303dactr_set_sleep_bit( bool enable );


/**
 * @brief - Soft Reset of Device
 */
void ism303dactr_reset();

/**
 * @brief - Soft Reset of Device in progress?
 */
bool ism303dactr_reset_in_progress();


void ism303dactr_who_am_i_test_mag();

void ism303dactr_who_am_i_test_acc();


/**
 * @brief - Register Read
 */
void ism303dactr_register_read( uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes );


/**
 * @brief - Register Write
 */
void ism303dactr_register_write( uint8_t register_address, uint8_t value);


void ism303dactr_init( external_device_config_t device_config,  external_device_config_t device_config_mag );

#endif // ISM303DACTR_H_

