// ---------------------------------------------------------------------------
// This example shows how to use NewPing's ping_timer method which uses the Timer2 interrupt to get the
// ping time. The advantage of using this method over the standard ping method is that it permits a more
// event-driven sketch which allows you to appear to do two things at once. An example would be to ping
// an ultrasonic sensor for a possible collision while at the same time navigating. This allows a
// properly developed sketch to multitask. Be aware that because the ping_timer method uses Timer2,
// other features or libraries that also use Timer2 would be effected. For example, the PWM function on
// pins 3 & 11 on Arduino Uno (pins 9 and 11 on Arduino Mega) and the Tone library. Note, only the PWM
// functionality of the pins is lost (as they use Timer2 to do PWM), the pins are still available to use.
// NOTE: For Teensy/Leonardo (ATmega32U4) the library uses Timer4 instead of Timer2.
// ---------------------------------------------------------------------------
#include <NewPing.h>

#define TRIGGER_PIN   3 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN      4 // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 150 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
unsigned int distance = 0;
unsigned int setDistance = 0;
unsigned long trigTimer;
unsigned int trigSpeed = 100;
unsigned int prevTrig = 0;



void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pingTimer = millis(); // Start now.
      pinMode(5, OUTPUT);
}

void loop() {

  setDistance = map(analogRead(A0), 0, 1023, 30, 150);

  
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }
          if (distance > setDistance) 
          {
                digitalWrite(5, 1);
               
                Serial.print("HIGH");
                } else  if (distance < setDistance) 
          {
                digitalWrite(5, 0);
                Serial.print("LOW");
                }
                                   
          
                
      Serial.print(distance);
    Serial.print("  --   ");
    Serial.print(setDistance);
    Serial.println("  .");

        //delay(1000);
}

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
     distance = sonar.ping_result / US_ROUNDTRIP_CM; }
     else {distance = MAX_DISTANCE;}
}
