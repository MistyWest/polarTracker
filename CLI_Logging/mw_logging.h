/*
 * nrf_log_includes.h
 *
 *  Created on: Nov 28, 2018
 *      Author: KLockwood
 */

#ifndef NRF_LOG_INCLUDES_H_
#define NRF_LOG_INCLUDES_H_

#include <stdint.h>
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nordic_common.h"

/** @def NRF_LOG_DEBUG
 *  @brief Macro for logging error messages. It takes a printf-like, formatted
 *  string with up to seven arguments.
 *
 *  @details This macro is compiled only if @ref NRF_LOG_LEVEL includes debug logs.
 */

#define MW_LOG_ERROR(...)			NRF_LOG_ERROR(__VA_ARGS__)
#define MW_LOG_WARNING(...)		NRF_LOG_WARNING( __VA_ARGS__)
#define MW_LOG_INFO(...)			NRF_LOG_INFO( __VA_ARGS__)
#define MW_LOG_DEBUG(...)			NRF_LOG_DEBUG( __VA_ARGS__)
#define MW_LOG_RAW(...)       NRF_LOG_RAW_INFO(__VA_ARGS__)

#define MW_LOG_BORDER "***********************************"


/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */
#define MW_LOG_FLOAT(val) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
                          (int32_t)(val),                                      \
                          (int32_t)((((val) > 0) ? (val) - (int32_t)(val)      \
                                                : (int32_t)(val) - (val))*1000)

#define MW_FLOAT_VALUE				"%s%d.%03d"


/**@brief Function for initializing the nrf log module.
 */
void mw_logging_init(void);




// Example ASCII Art Logo Generated on: https://www.ascii-art-generator.org/
// For best results use a width: 40, and an image with black background, white font
#define MW_LOG_MISTYWEST_WELCOME \
  MW_LOG_RAW("\r\n"); \
  MW_LOG_RAW("  ...        ..          ..        ..\r\n"); \
  MW_LOG_RAW(" ckkx;     .lkkd,      .okkd'     'xc\r\n"); \
  MW_LOG_RAW(".xx'ckc.   .kd.lk:     'Oo.ok;    ,Oc\r\n"); \
  MW_LOG_RAW(".kd. ;ko.  .ko  ,,     'Ol  ckc.  ;Oc\r\n"); \
  MW_LOG_RAW(".ko   'xx' 'Oo    .::. ,Oc   ,kd. ;O:\r\n"); \
  MW_LOG_RAW(".ko    .ok;;Ol    .;dk,:O:    .dx,cO;\r\n"); \
  MW_LOG_RAW(".d:      :xkx'      .cxkd'     .lxkd.\r\n"); \
  MW_LOG_RAW(" .        ..          ..         .. \r\n"); \


#define MW_CLI_MISTYWEST_WELCOME \
  "   ...        ..          ..        ..\r"  \
  "  ckkx;     .lkkd,      .okkd'     'xc\r"  \
  " .xx'ckc.   .kd.lk:     'Oo.ok;    ,Oc\r"  \
  " .kd. ;ko.  .ko  ,,     'Ol  ckc.  ;Oc\r"  \
  " .ko   'xx' 'Oo    .::. ,Oc   ,kd. ;O:\r"  \
  " .ko    .ok;;Ol    .;dk,:O:    .dxnncO;\r"  \
  " .d:      :xkx'      .cxkd'     .lxkd.\r"  \
  "  .        ..          ..         .. \r"


#endif /* NRF_LOG_INCLUDES_H_ */
