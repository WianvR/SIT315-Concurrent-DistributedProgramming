// C++ code
int sensor_pin = 2;
int led_pin = 13;

volatile int led_state = LOW;
int p_state = LOW;

void setup()
{
  pinMode(sensor_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  
  Serial.begin(9600);
  
  attachInterrupt(digitalPinToInterrupt(sensor_pin), alert, CHANGE);

}

void alert()
{
  if (digitalRead(sensor_pin) == HIGH)
  {
    led_state = HIGH;
  	digitalWrite(led_pin, led_state);
  }
  
  else
  {
    led_state = LOW;
  	digitalWrite(led_pin, led_state);
  }
}

void loop()
{
  if (led_state == HIGH && p_state == LOW)
  {
  	Serial.print("Motion Detected! ");
    Serial.println(led_state);
    p_state = led_state;
  }
  else if (led_state == LOW && p_state == HIGH)
  {
    Serial.print("It's quiet, too quiet... ");
    Serial.println(led_state);
    p_state = led_state;
  }
}