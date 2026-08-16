/**
 * @file cvd_profiles.c
 * @brief Reusable CVD S-type configuration profiles.
 */

#include "../Inc/cvd_profiles.h"

const CVD_ConfigTypeDef CVD_Profile_CVD5_0P75A_100Microstep = {
    .mot_sel      = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_MOT_SEL,
    .resolution   = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RESOLUTION,
    .run_current  = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RUN_CURRENT,
    .stop_current = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_STOP_CURRENT,
    .setting      = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_SETTING,
    .net_in       = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_NET_IN
};

const CVD_ConfigTypeDef CVD_Profile_PG413M_LA_C = {
    .mot_sel      = PG413_MOT_SEL,
    .resolution   = PG413_RESOLUTION,
    .run_current  = PG413_RUN_CURRENT,
    .stop_current = PG413_STOP_CURRENT,
    .setting      = PG413_SETTING,
    .net_in       = PG413_NET_IN
};

const CVD_ConfigTypeDef CVD_Profile_Default = {
    .mot_sel      = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_MOT_SEL,
    .resolution   = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RESOLUTION,
    .run_current  = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_RUN_CURRENT,
    .stop_current = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_STOP_CURRENT,
    .setting      = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_SETTING,
    .net_in       = CVD_PROFILE_CVD5_0P75A_100MICROSTEP_NET_IN
};
