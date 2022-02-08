
//TODO.. should be project setting?
#define LPS33HW_I2C_INST 0
#define LPS33HW_I2C_ADDR 0x5C

 /** I2C Device Address 8 bit format: if SA0=0 -> 0xB9 if SA0=1 -> 0xBB **/
#define LPS33HW_I2C_ADD_H   0xBBU
#define LPS33HW_I2C_ADD_L   0xB9U

/** Device Identification (Who am I) **/
#define LPS33HW_ID           0xB1U

/**
  * @}
  *
  */

/* Device Register and relevant Register Structs */
#define LPS33HW_INTERRUPT_CFG  0x0BU
typedef struct {
  uint8_t pe               : 2; /* ple + phe -> pe */
  uint8_t lir              : 1;
  uint8_t diff_en          : 1;
  uint8_t reset_az         : 1;
  uint8_t autozero         : 1;
  uint8_t reset_arp        : 1;
  uint8_t autorifp         : 1;
} lps33hw_interrupt_cfg_t;

#define LPS33HW_THS_P_L        0x0CU
#define LPS33HW_THS_P_H        0x0DU
#define LPS33HW_WHO_AM_I       0x0FU
#define LPS33HW_CTRL_REG1      0x10U
typedef struct {
  uint8_t sim              : 1;
  uint8_t bdu              : 1;
  uint8_t lpfp             : 2; /* en_lpfp + lpfp_cfg -> lpfp */
  uint8_t odr              : 3;
  uint8_t not_used_01      : 1;
} lps33hw_ctrl_reg1_t;

#define LPS33HW_CTRL_REG2      0x11U
typedef struct {
  uint8_t one_shot         : 1;
  uint8_t not_used_01      : 1;
  uint8_t swreset          : 1;
  uint8_t i2c_dis          : 1;
  uint8_t if_add_inc       : 1;
  uint8_t stop_on_fth      : 1;
  uint8_t fifo_en          : 1;
  uint8_t boot             : 1;
} lps33hw_ctrl_reg2_t;

#define LPS33HW_CTRL_REG3      0x12U
typedef struct {
  uint8_t int_s            : 2;
  uint8_t drdy             : 1;
  uint8_t f_ovr            : 1;
  uint8_t f_fth            : 1;
  uint8_t f_fss5           : 1;
  uint8_t pp_od            : 1;
  uint8_t int_h_l          : 1;
} lps33hw_ctrl_reg3_t;


#define LPS33HW_FIFO_CTRL      0x14U
typedef struct {
  uint8_t wtm              : 5;
  uint8_t f_mode           : 3;
} lps33hw_fifo_ctrl_t;

#define LPS33HW_REF_P_XL       0x15U
#define LPS33HW_REF_P_L        0x16U
#define LPS33HW_REF_P_H        0x17U
#define LPS33HW_RPDS_L         0x18U
#define LPS33HW_RPDS_H         0x19U

#define LPS33HW_RES_CONF       0x1AU
typedef struct {
  uint8_t lc_en            : 1;
  uint8_t not_used_01      : 7;
} lps33hw_res_conf_t;

#define LPS33HW_INT_SOURCE     0x25U
typedef struct {
  uint8_t ph               : 1;
  uint8_t pl               : 1;
  uint8_t ia               : 1;
  uint8_t not_used_01      : 4;
  uint8_t boot_status      : 1;
} lps33hw_int_source_t;

#define LPS33HW_FIFO_STATUS    0x26U
typedef struct {
  uint8_t fss              : 6;
  uint8_t ovr              : 1;
  uint8_t fth_fifo         : 1;
} lps33hw_fifo_status_t;

#define LPS33HW_STATUS         0x27U
typedef struct {
  uint8_t p_da             : 1;
  uint8_t t_da             : 1;
  uint8_t not_used_02      : 2;
  uint8_t p_or             : 1;
  uint8_t t_or             : 1;
  uint8_t not_used_01      : 2;
} lps33hw_status_t;

#define LPS33HW_PRESS_OUT_XL   0x28U
#define LPS33HW_PRESS_OUT_L    0x29U
#define LPS33HW_PRESS_OUT_H    0x2AU
#define LPS33HW_TEMP_OUT_L     0x2BU
#define LPS33HW_TEMP_OUT_H     0x2CU
#define LPS33HW_LPFP_RES       0x33U


/* Register Values */
#define LPS33HW_ODR_OFF        0
#define LPS33HW_ODR_1HZ        1
#define LPS33HW_ODR_10HZ       2
#define LPS33HW_ODR_25HZ       3
#define LPS33HW_ODR_50HZ       4
#define LPS33HW_ODR_75HZ       5




/**
 * @brief - Set ODR
 */
void lps33hwtr_set_odr( uint8_t odr );


/**
 * @brief - Enable/Disable Low Power Mode
 */
void lps33hwtr_enable_low_power_mode( bool enable );


/**
 * @brief - Enable/Disable I2C Interface
 */
void lps33hwtr_enable_i2c( bool i2c );


/**
 * @brief - Device ID verification
 */
bool lps33hwtr_who_am_i_test();


/**
 * @brief - Register Read
 */
void lps33hwtr_register_read( uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes );


/**
 * @brief - Register Write
 */
void lps33hwtr_register_write( uint8_t register_address, uint8_t value );


/**
 * @brief - Driver Communication Initialization
 */
void lps33hwtr_init( external_device_config_t device_config );


