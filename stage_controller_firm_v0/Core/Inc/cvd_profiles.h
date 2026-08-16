/**
 * @file cvd_profiles.h
 * @brief Reusable CVD S-type configuration profiles.
 */

#ifndef CVD_PROFILES_H
#define CVD_PROFILES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cvd_driver.h"

#define CVD_PROFILE_CVD5_0P75A_100MICROSTEP_MOT_SEL        CVD5_MOT_SEL_0P75A
#define CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RESOLUTION     CVD_5PH_MICROSTEP_100
#define CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RUN_CURRENT    CVD_RUN_CURRENT_100_PERCENT
#define CVD_PROFILE_CVD5_0P75A_100MICROSTEP_STOP_CURRENT   CVD_STOP_CURRENT_50_PERCENT
#define CVD_PROFILE_CVD5_0P75A_100MICROSTEP_SETTING        CVD_SETTING_ENABLE_PIN_PLS_DIR
#define CVD_PROFILE_CVD5_0P75A_100MICROSTEP_NET_IN         CVD_NET_IN_SD_ON_FILTER_OFF

#define CVD_SETTING_PG413_ENABLE_PIN_PLS_DIR               CVD_SETTING_ENABLE_PIN_PLS_DIR
#define CVD_SETTING_PG413_ENABLE_SPI_PLS_DIR               CVD_SETTING_ENABLE_SPI_PLS_DIR

#define PG413_MOT_SEL                                      CVD_PROFILE_CVD5_0P75A_100MICROSTEP_MOT_SEL
#define PG413_RESOLUTION                                   CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RESOLUTION
#define PG413_RUN_CURRENT                                  CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RUN_CURRENT
#define PG413_STOP_CURRENT                                 CVD_PROFILE_CVD5_0P75A_100MICROSTEP_STOP_CURRENT
#define PG413_SETTING                                      CVD_PROFILE_CVD5_0P75A_100MICROSTEP_SETTING
#define PG413_NET_IN                                       CVD_PROFILE_CVD5_0P75A_100MICROSTEP_NET_IN

extern const CVD_ConfigTypeDef CVD_Profile_CVD5_0P75A_100Microstep;
extern const CVD_ConfigTypeDef CVD_Profile_PG413M_LA_C;
extern const CVD_ConfigTypeDef CVD_Profile_Default;

#define CVD_Config_PG413M_LA_C CVD_Profile_PG413M_LA_C

#ifdef __cplusplus
}
#endif

#endif /* CVD_PROFILES_H */
