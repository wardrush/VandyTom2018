int counter = 0; //initialize counter variable to zero
  
void setup() 
{
  
  Serial.begin(9600); //run at 9600

}

void loop() 
{
  int raw = analogRead(0); //read from hall effect sensor in A5 port
  //Serial.println(raw);
  if (raw < 500 || raw > 600) // if the sensor senses a magnet
    {
      counter = counter + 1; //increment counter by 1
      delay(50); //wait in this loop for 50 ms, so it doesn't count the magnet crossing the sensor more than once
      Serial.println("count");
      Serial.println(counter);
    }
    
}
