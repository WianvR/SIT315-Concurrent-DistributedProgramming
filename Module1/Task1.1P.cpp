// C++ code
void setup()
{
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

int motionSensor = 0;

void ledOn()
{
	Serial.println("LED on");
 	delay(500);
}

void loop()
{
  motionSensor = digitalRead(2);
  
  if (motionSensor == HIGH)
  {
  	digitalWrite(13, HIGH);
    Serial.println("Motion Detected!");
    ledOn();
  }
  else
  {
  	digitalWrite(13, LOW);
    Serial.println("It's quiet, too quiet...");
  }
}
