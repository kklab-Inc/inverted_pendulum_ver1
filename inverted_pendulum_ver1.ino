//倒立振子
//#include <MPU9250_asukiaaa.h>//MPU9250のライブラリ
#include "kal/kal.h"

//debug
#define DEBUG 1

//motor
#define MOTOR_NUM 2//片方はエンコーダのみ
kal::nxtmotor motor[MOTOR_NUM];

//reference
kal::wave sin_wave(0.0, PI / 4, 1.5, SIN);

//robotdata
kal::RobotData<double> ref[MOTOR_NUM];
kal::RobotData<double> state[MOTOR_NUM];

//differentiator
kal::Diff<double> d_ref[MOTOR_NUM];
kal::Diff<double> d_st[MOTOR_NUM];
kal::Diff<double> d2_ref[MOTOR_NUM];
kal::Diff<double> d2_st[MOTOR_NUM];

#define KT 25.9//(mNm/A)
#define I2V 3.3/4 //MD換算
#define Ms 2.7*0.3*0.3*PI*15/4/1000 //アルミ棒質量(kg)
double t = 0.0;                     //time

double K[4] = { -40.6228, 60.4783, -60.9084, 140.238};

//timer関連
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {  /* this function must be placed in IRAM */
  portENTER_CRITICAL_ISR(&timerMux);
  //control---------------------------------------------------------------------------------------------------------------------------/
  t += Ts;
  //状態取得
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor[i].get_angle(state[i].q);
    d_st[i].update(state[i].q, state[i].dq);
    d2_st[i].update(state[i].dq, state[i].d2q);
  }
    //目標値計算
    //sin_wave.update();
    //ref[0].q = sin_wave.output;
    //d_ref[0].update(ref[0].q, ref[0].dq);
    //d2_ref[0].update(ref[0].dq, ref[0].d2q);
    
  ref[1].q = 0;
  double tau, u;
  
  tau = - (K[0]*state[0].q + K[1]*state[1].q + K[2]*state[0].dq + K[3]*state[1].dq);
  //tau = 30 * (ref[0].q - state[0].q) + 2 * (ref[0].dq - state[0].dq);
  u = I2V * 1.0 / KT * tau;
  motor[0].drive(u);
  
#if DEBUG
  for (int i = 0; i < MOTOR_NUM; i++) {
    Serial.print(ref[i].q * RAD2DEG);
    Serial.print(",");
    Serial.print(state[i].q * RAD2DEG);
    Serial.print(",");
    //    Serial.print(u);
    //    Serial.print(",");
    //    Serial.print(state[i].dq);
    //    Serial.print(",");
    //    Serial.print(state[i].d2q);
    //    Serial.print(",");
  }
  Serial.println();
#endif
  //-------------------------------------------------------------------------------------------------------------------------------------/
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  Serial.begin(115200);
  Serial.println("started");

  //motor1の設定
  motor[0].GPIO_setup(GPIO_NUM_25, GPIO_NUM_26); //方向制御ピン設定
  motor[0].PWM_setup(GPIO_NUM_2, 0); //PWMピン設定
  motor[0].encoder_setup(PCNT_UNIT_0, GPIO_NUM_4, GPIO_NUM_5); //エンコーダカウンタ設定
  motor[0].set_fb_param(35.0, 0, 20.0); //ゲイン設定
  motor[0].set_ff_param(0, 0, 0.3);

  //motor2//エンコーダのみ
  //  motor[1].GPIO_setup(GPIO_NUM_16,GPIO_NUM_17);//方向制御ピン設定
  //  motor[1].PWM_setup(GPIO_NUM_15,0);//PWMピン設定
  motor[1].encoder_setup(PCNT_UNIT_1, GPIO_NUM_18, GPIO_NUM_19); //エンコーダカウンタ設定
  //  motor[1].set_fb_param(30,0.0,5.0);//ゲイン設定

  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts * 1000000), true); //Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化
}
void loop() {
}
