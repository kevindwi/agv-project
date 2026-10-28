#pragma once

#define SERIAL_DEBUG Serial
#define SERIAL_COMM Serial1

// PIN MOTOR KANAN
#define R_START_STOP 22
#define R_RUN_BRAKE 23
#define R_DIR 24
#define R_INT_VR 25
#define R_PWM 9       // VRM
#define R_SPEED_OUT 3 // Encoder output (interrupt)
#define R_ALARM_RESET 26

// PIN MOTOR KIRI
#define L_START_STOP 28
#define L_RUN_BRAKE 29
#define L_DIR 30
#define L_INT_VR 31
#define L_PWM 10
#define L_SPEED_OUT 2 // Encoder output (interrupt)
#define L_ALARM_RESET 32

// KONSTANTA MOTOR
#define PWM_MAX 255
#define PWM_MIN 0

#define DIR_CW true
#define DIR_CCW false

// PIN TOMBOL
#define START_STOP_BTN_PIN 40
#define RESET_BTN_PIN 42
#define E_STOP_BTN_PIN 37

// WAKTU PROSES (SIMULASI)
#define LOADING_DURATION_MS 3000
#define UNLOADING_DURATION_MS 3000
#define OBSTACLE_TIMEOUT_MS 5000

// BACA KECEPATAN ENCODER
// Encoder: 30 pulsa/putaran poros motor, gearbox 30:1
// -> 900 pulsa per putaran roda (PULSES_PER_REV x GEAR_RATIO).
#define SPEED_READ_INTERVAL_MS 1000
#define PULSES_PER_REV 30
#define GEAR_RATIO 30
#define PULSES_PER_WHEEL_REV (PULSES_PER_REV * GEAR_RATIO)

// RODA
#define WHEEL_DIAMETER_M 0.15f

// PWM TASK
#define MOVING_PWM 20
#define ROTATE_PWM 15

// TIMEOUT PERINTAH SERIAL
// Motor berhenti bila tidak ada frame valid selama >= nilai ini.
#define SERIAL_COMMAND_TIMEOUT_MS 500

// Titik awal. Sesuaikan berdasarkan pengujian roda.
#define WHEEL_PID_KP 0.2f
#define WHEEL_PID_KI 0.0f
#define WHEEL_PID_KD 0.0f
#define WHEEL_PID_MAX_INTEGRAL 20.0f

#define WHEEL_SEPARATION_M 0.48f
// Interval kontrol lebih panjang agar encoder 900 tick/rev terbaca stabil.
#define CONTROL_PERIOD_MS 100
