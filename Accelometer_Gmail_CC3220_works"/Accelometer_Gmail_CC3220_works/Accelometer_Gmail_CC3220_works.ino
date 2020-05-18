#include <Wire.h>
#include <BMA222.h>

BMA222 mySensor;
  bool running = true;


//--------------------------------------------------------------------------
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Temboo.h>
#include "TembooAccount.h" // contains Temboo account information
                           // as described in the footer comment below

/*** SUBSTITUTE YOUR VALUES BELOW: ***/

// note that for additional security and reusability, you could
// use #define statements to specify these values in a .h file.

// your Gmail username, formatted as a complete email address, e.g., "john.bonham@gmail.com"
const String GMAIL_USER_NAME = "...";

// your application specific password (see instructions above)
const String GMAIL_APP_PASSWORD = "...";

// the email address you want to send the email to, e.g., "johnpauljones@temboo.com"
const String TO_EMAIL_ADDRESS = "...";

// a flag to indicate whether we've tried to send the email yet or not
//boolean attempted = true; 
boolean attempted = false; 

WiFiClient client;

//------------------------------------------------------------------------

void setup()
{

 // Serial.begin(9600);
    Serial.begin(115200);
  
  mySensor.begin();
  uint8_t chipID = mySensor.chipID();
  Serial.print("chipID: ");
  Serial.println(chipID);

// digitalWrite(RED_LED, LOW);    // turn the LED off by making the voltage LOW
//   digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
//    digitalWrite(YELLOW_LED, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);               // wait for a second
//  digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
//  digitalWrite(GREEN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
//   digitalWrite(YELLOW_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);               // wait for a second
//  digitalWrite(RED_LED, LOW);    // turn the LED off by making the voltage LOW
//   digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
//    digitalWrite(YELLOW_LED, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000);               // wait for a second
}

void loop()
{

  int8_t datax = mySensor.readXData();
  Serial.print("X: ");
  Serial.print(datax);

 int8_t datay = mySensor.readYData();
  Serial.print(" Y: ");
  Serial.print(datay);

  int8_t dataz = mySensor.readZData();
  Serial.print(" Z: ");
  Serial.println(dataz);

//if ( abs(datax)>100 || abs(datay)>100 || abs(dataz-64)>62) {
  if ( abs(datax)>120)  {

  Serial.println ("Accident********************************");
  running = false;
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
 int wifiStatus = WL_IDLE_STATUS;

  // determine if the WiFi Shield is present.
  Serial.print("\n\nShield:");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("FAIL");
    
    // if there's no WiFi shield, stop here.
    while(true);
  }
 
  while(wifiStatus != WL_CONNECTED) {
    Serial.print("WiFi:");
    wifiStatus = WiFi.begin((char *)WIFI_SSID, (char *)WIFI_PASSWORD);

    if (wifiStatus == WL_CONNECTED) {
      Serial.println("OK");
    } else {
      Serial.println("FAIL");
    }
    delay(5000);
  }
  
  // only try to send the email if we haven't already tried
  if (!attempted) {

    Serial.println("Running SendAnEmail...");
  
    TembooChoreo SendEmailChoreo(client);

    // invoke the Temboo client
    // NOTE that the client must be reinvoked, and repopulated with
    // appropriate arguments, each time its run() method is called.
    SendEmailChoreo.begin();
    
    // set Temboo account credentials
    SendEmailChoreo.setAccountName(TEMBOO_ACCOUNT);
    SendEmailChoreo.setAppKeyName(TEMBOO_APP_KEY_NAME);
    SendEmailChoreo.setAppKey(TEMBOO_APP_KEY);

    // identify the Temboo Library Choreo to run (Google > Gmail > SendEmail)
    SendEmailChoreo.setChoreo("/Library/Google/Gmail/SendEmail");
 
    // set the required Choreo inputs
    // see https://www.temboo.com/library/Library/Google/Gmail/SendEmail/ 
    // for complete details about the inputs for this Choreo

    // the first input is your Gmail email address    
    SendEmailChoreo.addInput("Username", GMAIL_USER_NAME);
    // next is your application specific password
    SendEmailChoreo.addInput("Password", GMAIL_APP_PASSWORD);
    // next is who to send the email to
    SendEmailChoreo.addInput("ToAddress", TO_EMAIL_ADDRESS);
    // then a subject line
    SendEmailChoreo.addInput("Subject", "ALERT: Sam's car crashed");

    // next comes the message body, the main content of the email   
    SendEmailChoreo.addInput("MessageBody", "Hey! your buddy might be dead due to a car accident. Call 911.");

    // tell the Choreo to run and wait for the results. The 
    // return code (returnCode) will tell us whether the Temboo client 
    // was able to send our request to the Temboo servers
    unsigned int returnCode = SendEmailChoreo.run();

    // a return code of zero (0) means everything worked
    if (returnCode == 0) {
        Serial.println("Success! Email sent!");
    } else {
      // a non-zero return code means there was an error
      // read and print the error message
      while (SendEmailChoreo.available()) {
        char c = SendEmailChoreo.read();
        Serial.print(c);
      }
    } 
    SendEmailChoreo.close();

    // set the flag showing we've tried
    attempted = false;
  }
   digitalWrite(RED_LED, LOW);    // turn the LED off by making the voltage LOW
   digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(YELLOW_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
  digitalWrite(RED_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(GREEN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
   digitalWrite(YELLOW_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(RED_LED, LOW);    // turn the LED off by making the voltage LOW
   digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(YELLOW_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second    
}


//---------------------------------------------------------------------------------------  

  if ( !running ) {
    while(1){
      delay(10000);
      }
  }
  delay(100);
 
  
}
