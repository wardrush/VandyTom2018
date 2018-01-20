int quarter_revolutions = 0; //initialize quarter revolutions variable to zero
int t = 0; //start time
int rpm; //angular velocity variable

void setup() 
{
  
  Serial.begin(9600); //run at 9600 baud
}

void loop() 
{
  int raw = analogRead(0); //read from hall effect sensor in A5 port
  //Serial.println(raw);
  if (raw < 500 || raw > 600) // if the sensor senses a magnet
    {
      quarter_revolutions = quarter_revolutions + 1; //increment counter by 1
      delay(100); //wait in this loop for 100 ms, so it doesn't count the magnet crossing the sensor more than once
      Serial.println("quarter rev");
      Serial.println(quarter_revolutions);
    }

    if (quarter_revolutions == 8)
      {
        rpm = 60000*2/(millis() - t); //two revolutions/ elapsed time
        t = millis(); //restart timer
        quarter_revolutions = 0;
        Serial.println("rpm");
        Serial.println(rpm);
      }
}
