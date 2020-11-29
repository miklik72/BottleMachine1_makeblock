#include <Arduino.h>
#include <Wire.h>
#include <MeMegaPi.h>

MeEncoderOnBoard Encoder_1(SLOT1);     // left wheel
MeEncoderOnBoard Encoder_2(SLOT2);     // right wheel
MeMegaPiDCMotor Bottle_motor(PORT3B);  // bottle tilt
MeUltrasonicSensor sonic(PORT_6);      // tilt control by distanc
MeGyro gyro(PORT_7);                   // gyro sensor
//Me7SegmentDisplay disp(PORT_8);

//bool bottle_up = true;                
float x_up = -15.0;                   // bottle tilt up
float x_down = 20.0;                  // bottle tilt down
float x;                              // bottle tilt real
float x_target;                       // bottle tilt target
uint16_t distanc_down = 5;            // distanc bottom limit 
uint16_t distanc_up = 15;             // distanc upper limit
uint16_t distanc;                     // distance real
float c = (x_down-x_up)/(distanc_up-distanc_down);  // coefficient distanc to tilt
long lasttime = millis();             // millis counter             

void isr_process_encoder1(void)       // interupt routine for motor encoder 1
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)       // interupt routine for motor encoder 2
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void bottle_init() {                 // set bottle to up position
  bool x_ok = true;
  float x;
  uint8_t bottle_speed = 50;
  while (x_ok) {
    gyro.update();
    x = gyro.getAngleX();
    if (x > x_up+1 ) {
      Bottle_motor.run(bottle_speed);
    }
    else if (x < x_up-1) {
      Bottle_motor.run(-bottle_speed);
    }
    else
    {
      Bottle_motor.stop();
      x_ok = false;
    }
  }
  Serial.println(x);
  Serial.println("Bottle is ready.");
}

void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  gyro.begin();
  //disp.init();
  //disp.set(BRIGHTNESS_2);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);//PIN12
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
  TCCR2B = _BV(CS21);

  TCCR3A = _BV(WGM30);//PIN9
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
   
  TCCR4A = _BV(WGM40);//PIN5
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);
  
  Encoder_1.setMotionMode(PID_MODE);
  Encoder_2.setMotionMode(PID_MODE);
  Encoder_1.setPulse(7);
  Encoder_2.setPulse(7);
  Encoder_1.setRatio(26.9);
  Encoder_2.setRatio(26.9);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
  Encoder_1.setPulsePos(0);
  Encoder_2.setPulsePos(0);
  bottle_init();
}

void loop() {
    //uint8_t ListDisp[4];
    distanc = (uint16_t)sonic.distanceCm();  // update sonic sensor distanc
    gyro.update();                           // update gyro position
    x = gyro.getAngleX();                    // current tilt position
    x_target = x_down - c * (distanc - distanc_down);
    if (x_target > x_up) x_target = x_up;
    else if (x_target < x_down) x_target = x_down;
    //disp.display(distanc);      
  if (millis() - lasttime > 500) {
    Serial.print(" distance: ");
    Serial.print(distanc);
    Serial.print(" c: ");
    Serial.print(c);
    Serial.print(" XT: ");
    Serial.print(x_target);
    Serial.print(" X: ");
    Serial.print(x);
    Serial.println("");
    lasttime = millis();
  }
  if (lasttime > millis())  lasttime = millis();
  Encoder_1.loop();
  Encoder_2.loop();
}