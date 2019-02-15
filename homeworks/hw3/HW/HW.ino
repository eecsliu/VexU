#include <NewPing.h>

#define motor_pin_1 2
#define motor_pin_2 3
#define motor_pin_3 4
#define led_pin 13
#define echo_pin 11
#define trig_pin 12
#define max_dist 200

char bluetooth_value;
float time_elapsed = 0;
float motor_timer = 0;
float motor_state = LOW;
bool connected = false;
float led_state = LOW;

NewPing sonar(trig_pin, echo_pin, max_dist);

void setup() {
  pinMode(motor_pin_1, OUTPUT);
  pinMode(motor_pin_2, OUTPUT);
  pinMode(motor_pin_3, OUTPUT);
  pinMode(led_pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  delay(50);
  time_elapsed += 50;
  motor_timer += 50;
  if (Serial.available())
  {
    time_elapsed = 0;
    connected = true;
    bluetooth_value = Serial.read();
  }
  if (bluetooth_value == 'n')
  {
    if (motor_timer > 1000)
    {
      motor_timer = 0;
      if (motor_state == HIGH) motor_state = LOW;
      else motor_state = HIGH;
    }
  }
  else if (bluetooth_value == 'f'){
    motor_state = LOW;
  }
  if (time_elapsed > 2000)
  {
    connected = false;
    Serial.write("l");
    motor_state = LOW;
  }
  
  float distance = sonar.ping_cm();
  if (distance < 10 and distance != 0){
    motor_state = LOW;
  }
  if (motor_state == HIGH)
  {
    led_state = HIGH;
  }
  else
  {
    led_state = LOW;
  }
  if (not connected){
    if (time_elapsed > 2500)
    {
      time_elapsed = 2000;
      if (led_state == HIGH) led_state = LOW;
      else led_state = HIGH;
    }
  }
  digitalWrite(led_pin, led_state);
  set_motor(motor_state);
}

void set_motor(float state){
  digitalWrite(motor_pin_1, state);
  digitalWrite(motor_pin_2, state);
  digitalWrite(motor_pin_3, state);
}
