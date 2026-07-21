#include <Arduino.h>

#define RSTART_STOP 22  //Black               // HIGH RUN, LOW STOP
#define RRUN_BRAKE  23  //White               // Accell deccel, HIGH ON, LOW Instant Stop
#define RDIR        24  //Gray                // HIGH CW, LOW CCW
#define RNT_VR      25  //Light Blue          // HIGH => pwm internal, LOW => external
#define RPWM        9                         // PWM
#define RSPEED_OUT   3
#define RALARM_RESET 26

#define LSTART_STOP 28
#define LRUN_BRAKE  29
#define LDIR        30
#define LNT_VR      31
#define LPWM        10
#define LSPEED_OUT   2
#define RALARM_RESET 32

#define CW  true
#define CCW false

bool alarm1 = false, alarm2 = false;

void motor1(bool dir, int pwme, int brake);
void motor2(bool dir, int pwme, int brake);

volatile unsigned long r_pulse_count = 0;
volatile unsigned long l_pulse_count = 0;

void r_pulse_ISR() {
  r_pulse_count++;
}

void l_pulse_ISR() {
  l_pulse_count++;
}

void setup() {
  Serial.begin(9600);

  pinMode(RSTART_STOP, OUTPUT);
  pinMode(RRUN_BRAKE, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(RNT_VR, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(RSPEED_OUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RSPEED_OUT), r_pulse_ISR, FALLING);

  pinMode(LSTART_STOP, OUTPUT);
  pinMode(LRUN_BRAKE, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(LNT_VR, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(LSPEED_OUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LSPEED_OUT), l_pulse_ISR, FALLING);

  digitalWrite(RNT_VR, LOW);
  digitalWrite(LNT_VR, LOW);

  digitalWrite(RRUN_BRAKE, HIGH);
  digitalWrite(LRUN_BRAKE, HIGH);

  delay(100);
}

void loop() {
  static unsigned long last_time = millis();

  if (millis() - last_time >= 1000)
  {
    motor1(CW, 100, 0);
    motor2(CCW, 100, 0);

    noInterrupts();
    unsigned long r_pulses = r_pulse_count;
    unsigned long l_pulses = l_pulse_count;

    r_pulse_count = 0;
    l_pulse_count = 0;
    interrupts();

    float r_frequency = r_pulses;
    float r_rpm = r_frequency * 2.0;

    float l_frequency = l_pulses;
    float l_rpm = l_frequency * 2.0;

    Serial.print("Right RPM = ");
    Serial.println(r_rpm);

    Serial.print("Left RPM = ");
    Serial.println(l_rpm);

    last_time = millis();
  }
  
  // motor1(CW, 100, 0);
  // motor2(CCW, 100, 0);

  // delay(3000);

  // motor1(CCW, 100, 0);
  // motor2(CW, 100, 0);

  // delay(3000);
}

void motor1(bool dir, int pwme, int brake)
{
  digitalWrite(RSTART_STOP, HIGH);
  //  digitalWrite(RMB_FREE, HIGH);

  if (dir)
  {
    digitalWrite(RDIR, HIGH);
  }
  else
  {
    digitalWrite(RDIR, LOW);
  }
  if (pwme > 0 && pwme < 256)
    analogWrite(RPWM, pwme);
  else
    digitalWrite(RPWM, LOW);
}

void motor2(bool dir, int pwme, int brake)
{
  digitalWrite(LSTART_STOP, HIGH);
  //  digitalWrite(LMB_FREE, HIGH);

  if (dir)
  {
    digitalWrite(LDIR, HIGH);
  }
  else
  {
    digitalWrite(LDIR, LOW);
  }

  if (pwme > 0 && pwme < 256)
    analogWrite(LPWM, pwme);
  else
    digitalWrite(LPWM, LOW);
}

void agv_berhenti()
{
  digitalWrite(RSTART_STOP, LOW);
  digitalWrite(LSTART_STOP, LOW);

  digitalWrite(RRUN_BRAKE, HIGH);
  digitalWrite(LRUN_BRAKE, HIGH);

  //  set_motor_aksel_desel = 0;

  //  motor1(CW, set_motor_aksel_desel, LOW);
  //  motor2(CCW, set_motor_aksel_desel, LOW);

  motor1(CW, 0, LOW);
  motor2(CCW, 0, LOW);

  //  digitalWrite(RFWD, LOW);
  //  digitalWrite(RREV, LOW);
  //  digitalWrite(LFWD, LOW);
  //  digitalWrite(LREV, LOW);
}
