//Sensors
int motion_pin = 2;
int echo_pin = 3;
int trig_pin = 4;

//LEDs
int orange_pin = 6;
int red_pin = 11;
int blue_pin = 10;
int green_pin = 9;

//Variables
volatile int orange_state = 0;
int o_Pstate = 0;

volatile unsigned long start_time = micros();
volatile unsigned long end_time = micros();
volatile bool new_pulse = false;
int distance = 0;
int pre_dist = 0;
int count = 0;


void setup()
{
  Serial.begin(9600);
  
  //Motion LED, lights up when movement detected.
  pinMode(motion_pin, INPUT_PULLUP);
  pinMode(orange_pin, OUTPUT);
  
  // Distance RGB LED.
  pinMode(echo_pin, INPUT_PULLUP);
  pinMode(trig_pin, OUTPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  
  
  attachInterrupt(digitalPinToInterrupt(motion_pin), alert, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo_pin), measurePulse, CHANGE);

}

//Changes flag orange LED state, executes in main.
void alert()
{
  if (digitalRead(motion_pin) == HIGH)
  {
    orange_state = HIGH;
  }
  
  else
  {
    orange_state = LOW;
  }
}

//Resets distance variable to value read from pin 3.
void measurePulse()
{
  if (digitalRead(echo_pin) == HIGH)
  {
  	start_time = micros();
  }
  else
  {
  	end_time = micros();
    new_pulse = true;
  }
}

//Triggers 10 micro second signal from pin 4.
void trigger()
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
}

//Calculates distance in cm from start and end times.
void distCalc()
{
  distance = (end_time - start_time) * 0.01723;
  new_pulse = false;
}

//RGB Distance LED. Sets light temp, dependent on distance var set in
//distCalc(). Close => red  far => blue.
void colTemp()
{
  if(distance < 100)
  {
  	digitalWrite(red_pin, HIGH);
  	digitalWrite(blue_pin, LOW);
  	digitalWrite(green_pin, LOW);
  }
  else if(distance > 200)
  {
  	digitalWrite(red_pin, LOW);
  	digitalWrite(blue_pin, HIGH);
  	digitalWrite(green_pin, LOW);
  }
  else if(distance > 100 && distance < 200)
  {
  	digitalWrite(red_pin, LOW);
  	digitalWrite(blue_pin, LOW);
  	digitalWrite(green_pin, HIGH);
  }
}

void loop()
{
  //Orange Motion LED. Toggles LED, dependent on flag set in alert().
  if (orange_state == HIGH && o_Pstate == LOW)
  {
    digitalWrite(orange_pin, orange_state);
  	Serial.print("Motion Detected! ");
    Serial.println(orange_state);
    o_Pstate = orange_state;
  }
  else if (orange_state == LOW && o_Pstate == HIGH)
  {
    digitalWrite(orange_pin, orange_state);
    Serial.print("It's quiet, too quiet... ");
    Serial.println(orange_state);
    o_Pstate = orange_state;
  }
  
 
  if (count == 10)
  {
  	trigger();
    count = 0;
  }
  count += 1;
  
  if (new_pulse)
  {
    distCalc();
  }
  
  if ((pre_dist < distance -1)||(pre_dist > distance +1))
  {
    colTemp();
  	Serial.print("Distance: ");
  	Serial.println(distance);
    pre_dist = distance;
  }
}