#pragma once

// PIN MOTOR KANAN
#define R_START_STOP  22
#define R_RUN_BRAKE   23
#define R_DIR         24
#define R_NT_VR       25
#define R_PWM          9
#define R_SPEED_OUT    3   // Encoder output (interrupt)
#define R_ALARM_RESET 26

// PIN MOTOR KIRI
#define L_START_STOP  28
#define L_RUN_BRAKE   29
#define L_DIR         30
#define L_NT_VR       31
#define L_PWM         10
#define L_SPEED_OUT    2   // Encoder output (interrupt)
#define L_ALARM_RESET 32

// KONSTANTA MOTOR
#define PWM_MAX   255
#define PWM_MIN   0

#define DIR_CW true
#define DIR_CCW false