 
 // This program works with MSP430
 //***************************//
// HC04 connect to:
// HC05 TxD to booster 1.3 (RxD) 
// HC05 RxD to booster 1.4 (TxD)
// No need to remove UATR jumpers as we are using serial1 and not serial for Bluetooth
// Energia use COM port that corresponds to MSP Application UART1 and not the MSP Debug Interface
 //*****************************//
 // Car drive
 // Connected ENA=P2.4; ENB=P2.5, In1=P8.2, In2=P3.7, In3=P4.0, In4=P4.3
 
 //     Right motor truth table
//Here are some handy tables to show the various modes of operation.
//  ENB         IN3             IN4         Description  
//  LOW   Not Applicable   Not Applicable   Motor is off
//  HIGH        LOW             LOW         Motor is stopped (brakes)
//  HIGH        LOW             HIGH        Motor is on and turning forwards
//  HIGH        HIGH            LOW         Motor is on and turning backwards
//  HIGH        HIGH            HIGH        Motor is stopped (brakes)   
//************************************//
String voice; // for BT voice
#define GREEN GREEN_LED
#define RED RED_LED

//**************************//
// define IO pin

#define ENA P2_4
#define ENB P2_5
#define IN1 P8_2
#define IN2 P3_7
#define IN3 P4_0
#define IN4 P4_3
//
////set car speed
#define CAR_SPEED 200
//typedef unsigned char uint8_t;  //Change the name of unsigned char to uint8_t
//
void forward(uint8_t car_speed)
{

  analogWrite(ENA, car_speed);
  analogWrite(ENB, car_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
//
//
void back(uint8_t car_speed)
{

  analogWrite(ENA, car_speed);
  analogWrite(ENB, car_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void left(uint8_t car_speed)
{

  analogWrite(ENA, car_speed);
  analogWrite(ENB, car_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void right(uint8_t car_speed)
{
  analogWrite(ENA, car_speed);
  analogWrite(ENB, car_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

//*/
void stop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}


void setup() {
  // For car
  pinMode(IN1,OUTPUT);//before useing io pin, pin mode must be set first 
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
// For BT
  Serial1.begin(9600);
  Serial.begin(9600);
   pinMode(GREEN, OUTPUT);
  Serial.println("starting");
  pinMode(RED, OUTPUT);
   analogWrite(RED,0); 
   analogWrite(GREEN,0);         
}

 int redVal;
  int greenVal;

void loop() {

   while (Serial1.available())   //Check if there is an available byte to read
  {                            
  delay(10);                   //Delay added to make thing stable
  char c = Serial1.read();      //Conduct a serial read
  if (c == '#') {break;}       //Exit the loop when the # is detected after the word
  voice += c;                  //Shorthand for voice = voice + c
  } 

  if (voice.length() > 0) {
    Serial.println(voice);


       if(voice == "*forward")//                                FOR RED COLOUR OF THE LED 
     {
     analogWrite(RED,255); 
     analogWrite(GREEN,0);
  forward(CAR_SPEED);  //go forward
 
//  delay(1000);//delay 1000 ms
     }  
   else if(voice == "*back")//                              FOR GREEN COLOUR OF THE LED !
     {
    analogWrite(GREEN,255);
    analogWrite(RED,0);
    back(CAR_SPEED);     //go back

//  delay(1000);
     }   
  else if(voice == "*left")//                                FOR BLUE COLOUR OF THE LED !
     {
    analogWrite(RED,255);
    analogWrite(GREEN,255);
    left(CAR_SPEED);     //turning left


     }
 else if(voice == "*right" )//                               FOR WHITE COLOUR OF THE LED !
     {
    analogWrite(RED,255);
    analogWrite(GREEN,255);
 right(CAR_SPEED);    //turning right

 // delay(1000);
     }
   else if(voice == "*stop")//                          FOR TURNING OFF LED !
     {
    analogWrite(RED,0);
    analogWrite(GREEN,0);
 stop();

     }
  voice="";
 // while(1);
  }
}
