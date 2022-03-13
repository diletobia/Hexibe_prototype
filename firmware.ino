#include "SAMDTimerInterrupt.h"
#include "MyFIFO.hpp"
#include <Servo.h>
#include <SPI.h>
#include <WiFiNINA.h>

MyFIFO<char, 1024> fifo_queue;

char ssid[] = "hexibe";        // your network SSID (name)
char pass[] = "gabbianimpilati";    // your network password (use for WPA, or use as key for WEP)


struct Motor {
  Motor(int step, int dir, int enn) : step(step), dir(dir), enn(enn) {}
  const int step;
  const int dir;
  const int enn;
};

/************
 * I/O pins *
 ************/
Motor motor1(9,A5,8);
Motor motor2(6,10,A3);
const int fan_pin = 4;
const int servo_pin = 2;
const int led_R_pin = 7;
const int butt1_pin = A0;
const int RGB_pin[] = {3, 5, A2};

uint8_t RGB_val_buffer[3] = {0,0,0};
uint8_t RGB_cnt = 0;

uint8_t servo_pos = 50;

int status = WL_IDLE_STATUS;
WiFiServer server(69);

Servo ball_servo;


void TimerHandler0(void) {
  // Structure:

  // Motors
  // 0bSCSSUUBB
  //   ||||||\\----- Bottom motor
  //   ||||\\------- Upper motor
  //   ||\\--------- Servo 
  //   ||
  //   |\----------- Color bit
  //   \------------ Sync

   
  if(!fifo_queue.empty()) {
    
    char x = fifo_queue.pop();
    if (x & 0b10) {
      digitalWrite(motor1.dir, x&1);

      digitalWrite(motor1.step, HIGH);
      delayMicroseconds(1);
      digitalWrite(motor1.step, LOW);
    }

    if (x & 0b1000) {
      digitalWrite(motor2.dir, x&0b100);

      digitalWrite(motor2.step, HIGH);
      delayMicroseconds(1);
      digitalWrite(motor2.step, LOW);
    }

    //ball_servo.write(50 + ((x >> 4) & 0b11) * (50/3));

    if (x & 0b10000000) {
      ball_servo.write(servo_pos);
      for (unsigned int i = 0; i < 3; i++) analogWrite(RGB_pin[i], RGB_val_buffer[i]);
      RGB_cnt = 0;
    }
    if (RGB_cnt < 8) {
      servo_pos = servo_pos << 1;
      servo_pos = servo_pos | ((x >> 5) & 0b1);
    }
    if (RGB_cnt < 8*3) {
      RGB_val_buffer[RGB_cnt/8] = (RGB_val_buffer[RGB_cnt/8] << 1) | ((x >> 6) & 1);
      RGB_cnt++;
    }
    
    
  } else {
    digitalWrite(led_R_pin, HIGH);
  }
}



SAMDTimer ITimer0(TIMER_TC3);



void setup() {
  // Initialize the serial com at 2M and wait until its available
  Serial.begin(2000000);
  delay(100);
 
  pinMode(motor1.step, OUTPUT);
  pinMode(motor1.dir, OUTPUT);
  pinMode(motor1.enn, OUTPUT);

  pinMode(motor2.step, OUTPUT);
  pinMode(motor2.dir, OUTPUT);
  pinMode(motor2.enn, OUTPUT);
  
  pinMode(fan_pin, OUTPUT);

  pinMode(led_R_pin , OUTPUT);

  pinMode(butt1_pin, INPUT);

  pinMode(RGB_pin[0], OUTPUT);
  pinMode(RGB_pin[1], OUTPUT);
  pinMode(RGB_pin[2], OUTPUT);

  analogWrite(RGB_pin[0], 0);
  analogWrite(RGB_pin[1], 0);
  analogWrite(RGB_pin[2], 10);
  
  ball_servo.attach(servo_pin);
  ball_servo.write(50);

  ITimer0.attachInterruptInterval(50, TimerHandler0); //uS
  digitalWrite(motor1.enn, LOW);

  digitalWrite(fan_pin,HIGH);

    // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true) {
      digitalWrite(led_R_pin, !digitalRead (led_R_pin));
      delay(100);
    }
  }
  server.begin();
}

WiFiClient client;
 

void loop() {
  
  if (client) {
    while (client.available()){
      if(!digitalRead(butt1_pin))
        digitalWrite(led_R_pin, LOW);
      if(!fifo_queue.full())
        fifo_queue.push(client.read());
    }
  } else {
    client = server.available();   // listen for incoming clients
  }
  
  while(Serial.available()) {
    if(!fifo_queue.full())
      fifo_queue.push(Serial.read());
    if(!digitalRead(butt1_pin))
      digitalWrite(led_R_pin, LOW);
  }

  if(!digitalRead(butt1_pin))
    digitalWrite(led_R_pin, LOW);
}
