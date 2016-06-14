
/*
Copyright 2016 Sebastien Lacoste-Seris (sebastien at lacoste-seris.net)
This program is free software: you can redistribute it 
and/or modify it under the terms of the GNU General Public 
License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any 
later version.

This code uses pin change interrupts and timer 1 to measure the 
time between the rise and fall of 3 channels of PPM 
(Though often called PWM, see http://forum.arduino.cc/index.php/topic,14146.html)
on a typical RC car receiver.  It could be extended to as
many channels as you like.  It uses the PinChangeInterrupt library
to notice when the signal pin goes high and low, and the 
Timer1 library to record the time between.

Based on Read Receiver from Lex Talionis (Lex.V.Talionis at gmail) http://playground.arduino.cc/Code/ReadReceiver

*/

#include <PinChangeInterrupt.h> // https://github.com/NicoHood/PinChangeInterrupt
#include <TimerOne.h>        // http://playground.arduino.cc/Code/Timer1

#define PIN_COUNT 2    //number of channels attached to the receiver
#define RC_FWD A2    //Arduino pins attached to the receiver
#define RC_FIRE A3    //Arduino pins attached to the receiver

byte pin[] = {RC_FIRE, RC_FWD };    //for maximum efficiency these pins should be attached
volatile unsigned int time[] = {0,0,0};                // to the receiver's channels in the order listed here

#define LIGHT_PINS 4
byte light_pin[] = {5, 6, 7, 8};   // Using 4 Leds for front lights

#define STOP_PINS 4
byte stop_pin[] = {9, 10, 11, 12};   // Using 4 Leds for stop lights

#define CH4_TRIGGER_LVL 1750 // receiver PWM level to trigger lights on CH4 for receiver R4EH-G
#define CH2_REVERSE 1470 // receiver PWM level to trigger lights on CH2 for receiver R4EH-G
boolean light_triggered = false;
boolean stop_light_on = false;

volatile byte state=0;

byte cmd=0;     // a place to put our serial data
volatile byte currPin=0;       // global counter for tracking what pin we are on
boolean light_on=false;

unsigned long time_switch_light=0;
#define delay_switch_light 250

unsigned long timer;
boolean led_status=false;
#define BLINK_DELAY 500
#define PIN_LED 13

void setup() {
    Serial.begin(115200);
    Serial.println("Receiver Init");

    pinMode(PIN_LED, OUTPUT); 
    digitalWrite(PIN_LED, LOW);
    timer = millis();

    Timer1.initialize(2200);    //longest pulse in PPM is usually 2.1 milliseconds,
                                //pick a period that gives you a little headroom.
    Timer1.stop();                //stop the counter
    Timer1.restart();            //set the clock to zero

    for (int light=0; light<LIGHT_PINS; light++){
       pinMode(light_pin[light], OUTPUT);     //set the pin to input
       digitalWrite(light_pin[light], LOW);
     }

     for (int light=0; light<STOP_PINS; light++){
       pinMode(stop_pin[light], OUTPUT);     //set the pin to input
       digitalWrite(stop_pin[light], LOW);
     }
     
    for (byte i=0; i<PIN_COUNT; i++)
    {
        Serial.print("Setting interupt pin : ");
        Serial.println(pin[i]);
        pinMode(pin[i], INPUT);     //set the pin to input
        digitalWrite(pin[i], HIGH); //use the internal pullup resistor
    }

    // Ready to go, setup initial interupt
    attachPCINT(digitalPinToPinChangeInterrupt(pin[currPin]), rise,RISING); // attach a PinChange Interrupt to our first pin
}

void loop() {

    if ( millis() < timer || (millis()-timer) >BLINK_DELAY){
      led_status = !led_status;
      digitalWrite(PIN_LED, (led_status ? HIGH : LOW ));
      timer = millis();
    }
  
    cmd=Serial.read();        //while you got some time gimme a systems report
    if (cmd=='p')
    {
        Serial.print("i:\t");
        Serial.println(currPin);
        Serial.print("time:\t");
        for (byte i=0; i<PIN_COUNT;i++)
        {
            Serial.print(i,DEC);
            Serial.print(":");
            Serial.print(time[i],DEC);
            Serial.print("\t");
        }

        Serial.println();
    }
    cmd=0;


    switch (state)
    {
        case FALLING: //we just saw a falling edge
            //PCintPort::detachInterrupt(pin[currPin]);
           state=0; // Reset interupt state

          //Serial.println(time[currPin]);
           
           switch (pin[currPin]){
            case RC_FIRE: // Channel 4 - light switch
              if (time[currPin] > CH4_TRIGGER_LVL && (millis() - time_switch_light) > delay_switch_light){
                if (! light_triggered){
                  light_triggered = true;
                  switch_light();
                  time_switch_light=millis();
                } else {
                  light_triggered = false;
                }
              }
              break;
            case RC_FWD: // Channel 2 - Throttle
              if (time[currPin] < CH2_REVERSE){
                // Switch stop light
                if (!stop_light_on){
                  // turn Stop on
                  //digitalWrite(13, HIGH);
                  stop_light_on = true;
                  switch_stop_light(stop_light_on);
                }
              }else{
                if (stop_light_on){
                  // turn Stop off
                  stop_light_on = false;
                  switch_stop_light(stop_light_on);
                }
              }
              break;
           }

            currPin++;                //move to the next pin
            currPin = currPin % PIN_COUNT;  //i ranges from 0 to PIN_COUNT
            attachPCINT(digitalPinToPinChangeInterrupt(pin[currPin]), rise,RISING);
            break;
        /*default:
            //do nothing
            break;*/
    }

}

void rise()        //on the rising edge of the currently interesting pin
{
    detachPCINT(digitalPinToPinChangeInterrupt(pin[currPin]));
    
    Timer1.restart();        //set our stopwatch to 0
    Timer1.start();            //and start it up

    //Serial.print('r');
    attachPCINT(digitalPinToPinChangeInterrupt(pin[currPin]), fall,FALLING);  
}

void fall()        //on the falling edge of the signal
{

  detachPCINT(digitalPinToPinChangeInterrupt(pin[currPin]));

  time[currPin]=Timer1.read();    // The function below has been ported into the
                            // the latest TimerOne class, if you have the
                            // new Timer1 lib you can use this line instead    
  Timer1.stop();
  //Serial.print('f');
  state=FALLING;
}

void switch_light(){
  int light;
  
  switch (light_on){
    case true:
        Serial.println("Light Off");
        // turn light Off
        for (light=0; light<LIGHT_PINS; light++){
         digitalWrite(light_pin[light], LOW);
        }
        light_on = false;    
    break;
    default:
         Serial.println("Light On");
        // turn light On
        for (light=0; light<LIGHT_PINS; light++){
         digitalWrite(light_pin[light], HIGH);
        }
        light_on = true;
    break;
  }
}

void switch_stop_light(boolean state){
  Serial.print("Stop ");
  Serial.println((state ? "On" : "Off"));
  
  // set stop light
  for (int light=0; light<STOP_PINS; light++){
   digitalWrite(stop_pin[light], (state ? HIGH : LOW));
  } 
}

