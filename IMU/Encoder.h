#ifndef ENDCOER_H
#define ENDCOER_H

#include "Arduino.h"

#define ENCL_A 4 //encorder
#define ENCL_B 5 //encorder

#define ENCR_A 2 //encorder
#define ENCR_B 3 //encorder

#define Encoder_cpr 250.0

#define DEBUG_SERIAL  SerialUSB

int32_t pos_right = 0;
int32_t pos_left = 0;


void encoder_init()
{
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  attachInterrupt(ENCR_B, Callback_R, RISING);
  attachInterrupt(ENCL_B, Callback_L, RISING);

  //chanel 0
  REG_PMC_PCER1 |= PMC_PCER1_PID36;
  REG_PIOC_ABSR |= PIO_ABSR_P3;
  REG_PIOC_PDR |= PIO_PDR_P3;                      //disable from controlling the corres (enable perpheral)
  REG_PWM_CLK = PWM_CLK_PREB(0) | PWM_CLK_DIVB(2); //84 / 42 = 2M
  REG_PWM_CMR0 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKB;

  //chanel 1
  REG_PMC_PCER1 |= PMC_PCER1_PID36;
  REG_PIOA_ABSR |= PIO_ABSR_P19;
  REG_PIOA_PDR |= PIO_PDR_P19;                     //disable from controlling the corres (enable perpheral)
  REG_PWM_CLK = PWM_CLK_PREB(0) | PWM_CLK_DIVB(2); //84 / 42 = 2M
  REG_PWM_CMR1 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKB;
}

void enc_right_read()
{
  if(!digitalRead(ENCR_A))
  {
    pos_right++;
  }
  else
  {
    pos_right--;
  }
  DEBUG_SERIAL.print("pos_right");
  DEBUG_SERIAL.print(pos_right);
}

void enc_left_read()
{
  if(digitalRead(ENCL_A))
  {
    pos_left++;
  }
  else
  {
    pos_left--;
  }
  DEBUG_SERIAL.print("pos_left");
  DEBUG_SERIAL.print(pos_left);
}
#endif // ENDCOER_H
