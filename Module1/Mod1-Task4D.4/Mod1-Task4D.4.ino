#include <avr/interrupt.h>
//#include <PinChangeInterrupt.h>   //Library from NicoHood

//Sensors
int motion_pin = 2;     // PIR motion sensor
int signal_pin = 3;     // Ultrasonic distance sensor
int trimpot_pin = A0;   // Potentiometer

//LEDs
int orange_pin = 7;     // PIR
int red_pin = 13;       // Distance close
int green_pin = 12;     // Distance medium
int blue_pin = 11;      // Distance far
int yellow_pin = 6;     // Trimpot

//Variables
volatile int orange_state = 0;
int o_Pstate = 0;

volatile unsigned long start_time = micros();
volatile unsigned long end_time = micros();
volatile bool receive_pulse = false;
volatile bool send_pulse = false;
bool new_dist = false;
int distance = 0;

const uint16_t t1_start = 0;
const uint16_t cmr = 6250;              // = (16*10^6) / (10*256) - 1

volatile int p_meter = 0;
int prev_pm = 0;

void setup()
{
  Serial.begin(9600);

  //Motion LED, lights up when movement detected.
  pinMode(motion_pin, INPUT_PULLUP);
  //DDRD &= !(1 << motion_pin);       // Set to input
  pinMode(orange_pin, OUTPUT);
  //DDRD |= (1 << orange_pin);        // Set to output

  // Distance RGB LED.
  pinMode(red_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);

  // Dimmer LED
  //pinMode(yellow_pin, OUTPUT); //not required for analogWrite()
  //pinMode(trimpot_pin, INPUT_PULLUP); // Set below on PCMSK1

  //Event driven interrupts, pins 2,3 motion and distance.
  attachInterrupt(digitalPinToInterrupt(motion_pin), alert, CHANGE);
  attachInterrupt(digitalPinToInterrupt(signal_pin), measurePulse, CHANGE);

  //attachPCINT(digitalPinToPCINT(trimpot_pin), readPot, CHANGE);

  cli();

  //set timer1 interrupt at 10Hz
  TCCR1A = 0;                   // set entire TCCR1A register to 0
  TCCR1B &= ~(1 << WGM13);      // Clear TCCR1B
  TCCR1B |= (1 << WGM12);       // Enable CTC

  TCNT1  = t1_start;            //initialize counter value to 0
  OCR1A = cmr;                  // set compare match register for 10hz increments

  TCCR1B |= (1 << CS12);        // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TIMSK1 |= (1 << OCIE1A);      // enable timer compare interrupt

  //Set Port C data direction for input on physical pin A0
  //DDRC &= ~(1 << DDC6);
  //DDRC |= (1 << DDC0);
  DDRC &= !B0000001;            //Set PortC pin A0 for data input

  //Set Pin change interrupt on physical pin A0
  //PCICR |= (1 << PCIE1);
  //PCMSK1 |= (1 << PCINT8);

  PCICR |= B00000010;         //Enables PCMSK1 PCINT8 - 14
  PCMSK1 |= B00000001;        //Sets PCINT8 for interrupt on input

  sei();                      //Enable global interrupts

}

//Triggers 10 micro second signal from pin 3.
void trigger()
{
   //Serial.println("triggered");
  pinMode(signal_pin, OUTPUT);        // Set pin for output

  digitalWrite(signal_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(signal_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(signal_pin, LOW);

  pinMode(signal_pin, INPUT);         // Reset for input on measurePulse
}

//Calculates distance in cm from start and end times.
void distCalc()
{
  distance = (end_time - start_time) * 0.01723;
  receive_pulse = false;
  new_dist = true;
}

//RGB Distance LED. Sets light temp, dependent on distance var set in
//distCalc(). Close => red  far => blue.
void colTemp()
{
  if ((distance < 50) && (distance >= 2))
  {
    digitalWrite(red_pin, HIGH);
    digitalWrite(green_pin, LOW);
    digitalWrite(blue_pin, LOW);
  }
  else if ((distance < 100) && (distance >= 50))
  {
    digitalWrite(red_pin, LOW);
    digitalWrite(green_pin, HIGH);
    digitalWrite(blue_pin, LOW);
  }
  else if (distance >= 100)
  {
    digitalWrite(red_pin, LOW);
    digitalWrite(green_pin, LOW);
    digitalWrite(blue_pin, HIGH);
  }
  new_dist = false;
}

// Adjust brightness of yellow LED
void yelLumen()
{
  analogWrite(yellow_pin, (p_meter -470) *2);   //Trimpot likely faulty, not supplying full 0-1023 range
}

void loop()
{
  //Orange Motion LED. Toggles LED, dependent on flag set in alert().
  if (orange_state == HIGH && o_Pstate == LOW)
  {
    digitalWrite(orange_pin, orange_state);
    Serial.print("                        Motion Detected! ");
    Serial.println(orange_state);
    o_Pstate = orange_state;
  }
  else if (orange_state == LOW && o_Pstate == HIGH)
  {
    digitalWrite(orange_pin, orange_state);
    Serial.print("                        It's quiet, too quiet... ");
    Serial.println(orange_state);
    o_Pstate = orange_state;
  }

  //Distance Sensor
  if (send_pulse)
  {
    trigger();
    send_pulse = false;
  }

  if (receive_pulse)
  {
    distCalc();
  }

  if ((new_dist) && (distance != 0))
  {
    colTemp();
    Serial.print("Distance: ");
    Serial.println(distance);
  }

  //Yellow LED dimmer
  if ((p_meter > prev_pm + 2) || (p_meter < prev_pm - 2))
  {
    yelLumen();
    prev_pm = p_meter;
    Serial.print("               Pot: ");
    Serial.println( p_meter);
  }
}


// [ISR] Changes flag orange LED state, executes in main.
void alert()
{
  if (digitalRead(motion_pin) == HIGH)
    //if (PIND & B0000100)
  {
    orange_state = HIGH;
  }

  else
  {
    orange_state = LOW;
  }
}

// [ISR] As a result of running output (trigger) and input (echo) signals off the same pin,
//(due to not having the SR04 sensor) measurePulse external interrupt is being queued
// off of the trigger signal resulting in a 0 distance value, before then receiving the
// echo signal.
void measurePulse()
{
  if (digitalRead(signal_pin) == HIGH)
    //if (PIND & B00001000)
  {
    start_time = micros();
  }
  else
  {
    end_time = micros();
    receive_pulse = true;
  }
}

// [ISR] Timer Interrupt, sets flag to run trigger function on next cycle.
ISR(TIMER1_COMPA_vect)
{
  send_pulse = true;
}

//For Pin Change Interrupt using Library from NicoHood
//void readPot()
//{
  //p_meter = analogRead(trimpot_pin);
//}

// [ISR] Pin Change Interrupt. Activated on change from port C pin A0, refreshes trimpot value.
ISR(PCINT1_vect)          //ISR vector for pins PCINT8 - 14
{
  p_meter = analogRead(trimpot_pin);
}
