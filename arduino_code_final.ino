#include "functions.h"

/* ---------------------------------------
  USe of this INO file

  Pin 10 M1 PWM
  Pin 6 M2 PWM
  Pin 12,13 M1 Direction
  Pin  7,8  M2 Direction
  Pin  3  M2 Encoder
  Pin  2  M1 Encoder

  Commands
  Blue Marked Motor is M2
  ----------------------------------------*/

unsigned char m1_encoder_pin=2;
unsigned char m2_encoder_pin=3;
unsigned char m1_pwm_pin=11;
unsigned char m2_pwm_pin=6;
unsigned char m1_direction_pin1=12;
unsigned char m1_direction_pin2=13;
unsigned char m2_direction_pin1=7;
unsigned char m2_direction_pin2=8;
float cps_m1 = 0.0;float cps_m2 = 0.0;
int m1_speed_setpoint = 2000; //Give Speed in cps
int m2_speed_setpoint = 2000; //Give Speed in cps
int nums[10];int nums2[10];
int timer1_counter;
float sum_m1 = 0;
float sum_m2 = 0;




//PID related
double error_speed_m1= 0;
double error_speed_pre_m1 = 0;  //last error of speed
double error_speed_sum_m1 = 0;  //sum error of speed
double pwm_pulse_m1 = 0;     //this value is 0~255

double error_speed_m2= 0;
double error_speed_pre_m2 = 0;  //last error of speed
double error_speed_sum_m2 = 0;  //sum error of speed
double pwm_pulse_m2 = 0;     //this value is 0~255

double kp_m1 = 0.11;
double ki_m1 = 0.08;
double kd_m1 = 0.01;

double kp_m2 = 0.36;
double ki_m2 = 0.03;
double kd_m2 = 0.06;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Motor Pin Map
  pinMode(m1_pwm_pin, OUTPUT); // M1 Power
  pinMode(m2_pwm_pin, OUTPUT); // M2 Power
  pinMode(m1_direction_pin1, OUTPUT); // M1 Direction
  pinMode(m1_direction_pin2, OUTPUT); // M1 Direction
  pinMode(m2_direction_pin1, OUTPUT) ; // M2 Direction
  pinMode(m2_direction_pin2, OUTPUT) ; // M2 Direction

  //Encoder
  pinMode(m1_encoder_pin, INPUT_PULLUP); // M2 encoder A
  pinMode(m2_encoder_pin, INPUT_PULLUP); // M1 encoder A
  attachInterrupt(digitalPinToInterrupt(m1_encoder_pin), encoder1A, RISING);
  attachInterrupt(digitalPinToInterrupt(m2_encoder_pin), encoder2A, RISING);

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)


  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
//  stop();
analogWrite(m1_pwm_pin,0);
analogWrite(m2_pwm_pin,0);

}
void loop() {

  while (Serial.available()<=0);//if nothing then wait
  char inChar = (char)Serial.read();
    if(inChar=='0')//stop the bot
  { 
    m1 = 0;
    m2 = 0;
    digitalWrite(m1_direction_pin1, LOW);
    digitalWrite(m1_direction_pin2, LOW);
    digitalWrite(m2_direction_pin1, LOW);
    digitalWrite(m2_direction_pin1, LOW);    

    }
    else if(inChar=='1'|| inChar=='2'){//We have to read speed of M! and direction
         String mySt="";
        while(1){
            while(Serial.available()<=0);
            char new_char = (char)Serial.read();
            if (new_char != '\n') {
                mySt += new_char;
                }
            else  {
                 int speed_dir=int(mySt.toFloat());
                 unsigned char direction_=speed_dir%10;

                 if(inChar=='1'){
                      m1_speed_setpoint=speed_dir/10;
                      m1 = 1;
                      if (direction_){
                      digitalWrite(m1_direction_pin1, HIGH);
                      digitalWrite(m1_direction_pin2, LOW);
                      }
                      else{
                      digitalWrite(m1_direction_pin2, HIGH);
                      digitalWrite(m1_direction_pin1, LOW);
                      }
                 }
                 else if(inChar=='2'){
                      m2_speed_setpoint=speed_dir/10;
                      m2 = 1;
                      if (direction_){
                      digitalWrite(m2_direction_pin1, HIGH);
                      digitalWrite(m2_direction_pin2, LOW);
                      }
                      else{
                      digitalWrite(m2_direction_pin2, HIGH);
                      digitalWrite(m2_direction_pin1, LOW);
                      }
                 }
                break;
                }
        }

    }
        /*else if(inChar=='2'){//We have to read speed of M2 and direction
         String mySt="";
        while(1){
            while(Serial.available()<=0);
            char new_char = (char)Serial.read();
            if (new_char != '\n') {
                mySt += new_char;
                }
            else  {
                 int speed_dir=int(mySt.toFloat());
                 m2_speed_setpoint=speed_dir/10;
                 unsigned char direction_=speed_dir%10;
                 m2 = 1;
                 if (direction_){
                    digitalWrite(m2_direction_pin1, HIGH);
                    digitalWrite(m2_direction_pin2, LOW);
                }
                else{
                    digitalWrite(m2_direction_pin2, HIGH);
                    digitalWrite(m2_direction_pin1, LOW);
                }
                break;
                }
        }

    }*/
  else if(inChar=='3'){// encoder 1 data
      Serial.println(encoder_val_m1);
 
}
  else if(inChar=='4'){// encoder 1 data
      Serial.println(encoder_val_m2);
 
}
  else if(inChar=='5'){// filtered encoder 1
      Serial.println(cps_m1);
 
}
  else if(inChar=='6'){// filtered_encoder_2
      Serial.println(cps_m2);
 
}
    
 
  }


// ISR

ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{

  TCNT1 = timer1_counter;   // set timer
 
  for (unsigned char p = 0; p < 10; p++)
  {
    sum_m1 += buff_1[p];
    sum_m2 += buff_2[p];
  }
  sum_m1= sum_m1/10;
  sum_m2 = sum_m2/10;

  if (sum_m1 > 0.00001)
    cps_m1 = 1000000.0/ sum_m1;
  else
    cps_m1 = 0.0;

  if (sum_m2 >0.00001)
    cps_m2 = 1000000 / sum_m2;
  else
    cps_m2 = 0;

  if (m1) {

    error_speed_m1 = m1_speed_setpoint - cps_m1;
    pwm_pulse_m1 = error_speed_m1 * kp_m1 + error_speed_sum_m1 * ki_m1 + (error_speed_m1 - error_speed_pre_m1) * kd_m1;
    error_speed_pre_m1 = error_speed_m1;  //save last (previous) error
    error_speed_sum_m1 += error_speed_m1; //sum of error
    if (error_speed_sum_m1 > 4000) error_speed_sum_m1 = 4000;
    if (error_speed_sum_m1 < -4000) error_speed_sum_m1 = -4000;
  }
  else {
    error_speed_m1 = 0.0;
    error_speed_pre_m1 = 0.0;
    error_speed_sum_m1 = 0.0;
    pwm_pulse_m1 = 0.0;
    for (int i = 0 ; i < 10 ; i++)
      buff_1[i] = 0;
  }
      //Motor 1
  if (pwm_pulse_m1 <255 & pwm_pulse_m1 >0) 
    analogWrite(m1_pwm_pin, pwm_pulse_m1); //set motor speed
  
  else if (pwm_pulse_m1 > 255)
      analogWrite(m1_pwm_pin, 255);
   else
      analogWrite(m1_pwm_pin, 0);
//Motor2

  if (m2) {
    error_speed_m2 = m2_speed_setpoint - cps_m2;
    pwm_pulse_m2 = error_speed_m2 * kp_m2 + error_speed_sum_m2 * ki_m2 + (error_speed_m2 - error_speed_pre_m2) * kd_m2;
    error_speed_pre_m2 = error_speed_m2;  //save last (previous) error
    error_speed_sum_m2 += error_speed_m2; //sum of error
    if (error_speed_sum_m2 > 4000) error_speed_sum_m2 = 4000;
    if (error_speed_sum_m2 < -4000) error_speed_sum_m2 = -4000;
  }
  else {
    error_speed_m2 = 0.0;
    error_speed_pre_m2 = 0.0;
    error_speed_sum_m2 = 0.0;
    pwm_pulse_m2 = 0.0;
    for (int i = 0 ; i < 10 ; i++)
      buff_2[i] = 0;
  }

  //Serial Plotter
  // Serial.print("Error in Speed M2");
  // Serial.print(e_speed2);
  // // Serial.println(",Min:0,Max:100");
  // Serial.print("\t");
  // Serial.print(defaultSpeed2);
  // Serial.print("\t");
  // Serial.println(cps2);

  // Motor 2

  if (pwm_pulse_m2 <255 & pwm_pulse_m2 >0) 
    analogWrite(m2_pwm_pin, pwm_pulse_m2); //set motor speed
  
  else if (pwm_pulse_m2 > 255)
      analogWrite(m2_pwm_pin, 255);
   else
      analogWrite(m2_pwm_pin, 0);


}
