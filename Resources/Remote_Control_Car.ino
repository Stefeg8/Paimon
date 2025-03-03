#include<IRremote.h>
const int RemotePin=8;
IRrecv irrecv(RemotePin);
decode_results results;
int in1=3;
int in2=5;
int in3=6;
int in4=9;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
    
}

void loop() {
    if (Serial.available() > 0) {
    char command = Serial.read(); // Read command from Raspberry Pi
    if (command == 'F') {    
      Serial.println("Acknowledged: F");    // Forward
      Forward();
    } else if (command == 'B') { 
      Serial.println("Acknowledged: B");   // Backward
      Backward();
    } else if (command == 'L') {
      Serial.println("Acknowledged: L");    // Left
      Left();
    } else if (command == 'R') {
      Serial.println("Acknowledged: R");    // Right
      Right();
    } else if (command == 'S') {  // Stop
      Stop();
    }
  }
    if(irrecv.decode(&results))
      {
        if (results.value==0xFF18E7)//Press UP Button
        { 
          Forward();
        }
        else if (results.value==0xFF4AB5)//Press Down Button
        { 
          Backward();
        }
        else if (results.value==0xFF10EF)//Press Left Button
        { 
          Left();
        }
        else if (results.value==0xFF5AA5)//Press Right Button
        { 
          Right();
        }
        else if (results.value==0xFF38C7)//Stop
        { 
          Stop();
        }
      irrecv.resume();
    }
    }
   
 
void Backward()
  {
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  }
 void Forward()
  {
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  }
 void Stop()
  {
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  }
  int Left()
    {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    }
  int Right()
    {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    }
