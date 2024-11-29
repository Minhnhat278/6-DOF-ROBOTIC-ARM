
/*
Bui Quoc An     2010098
Tran Minh Nhat  2014007
*/
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>



#define PI 3.1415926535897932384626433832795

//driver for the axis 1
#define PUL1_PIN 39
#define DIR1_PIN 37
//driver for the axis 2
#define PUL2_PIN 43
#define DIR2_PIN 41
//driver for the axis 3
#define PUL3_PIN 47
#define DIR3_PIN 45
//driver for the axis 4
#define PUL4_PIN 46
#define DIR4_PIN 48
//driver for the axis 5
#define PUL5_PIN A6
#define DIR5_PIN A7
//driver for the axis 6
#define PUL6_PIN A0
#define DIR6_PIN A1
//MAGNETIC PIN
#define MAG_PIN 31
#define ENA_PIN 33
#define DISA_PIN 35
//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38



double curPos1 = 0.0;
double curPos2 = -78.51;
double curPos3 = 73.90;
double curPos4 = 0.0;
double curPos5 = -90.0;
double curPos6 = 0.0;



boolean PULstat1 = 0;
boolean PULstat2 = 0;
boolean PULstat3 = 0;
boolean PULstat4 = 0;
boolean PULstat5 = 0;
boolean PULstat6 = 0;

//robot geometry
const double dl1 = 360.0/200.0/32.0/4.8;
const double dl2 = 360.0/200.0/32.0/4.0;
const double dl3 = 360.0/200.0/32.0/5.0;
const double dl4 = 360.0/200.0/32.0/2.8;
const double dl5 = 360.0/200.0/32.0/2.1;
const double dl6 = 360.0/200.0/32.0/1.0;
const double r1 = 47.0;
const double r2 = 110.0;
const double r3 = 26.0; 
const double d1 = 133.0;
const double d3 = 0.0;
const double d4 = 117.50;
const double d6 = 28.0;


String dataIn = ""; 
double futPos1 = 0.0;
double futPos2 = 0.0;
double futPos3 = 0.0;
double futPos4 = 0.0;
double futPos5 = 0.0;
double futPos6 = 0.0;
double curSpeed = 0.4e-4;
double curFinalSpeed = 0.0;

double futX;
double futY;
double futZ;
double futRx;
double futRy;
double futRz;

int index = 0; //index corresonding to the robot position
float Joint1[50], Joint2[50], Joint3[50], Joint4[50], Joint5[50], Joint6[50], MaxSpeed[50], InSpeed[50], FinSpeed[50], Magnic[50];
float Jinverse[6] = {0,0,0,0,0,0};

int EMER = 0;
int DONE = 0;
double curX;
double curY;
double curZ;
double curRx;
double curRy;
double curRz;
float Xsend[6]={0,0,0,0,0,0};
String ab ="";

void emergencyStopInterrupt()
{
  EMER = 1;
}
ISR (TIMER1_OVF_vect) 
{
  //String ab = "";
  ab = String(curPos1)+" "+String(curPos2)+" "+String(curPos3)+" "+String(curPos4)+" "+String(curPos5)+" "+String(curPos6)+" "+String(index)+" "+String(EMER)+" "+String(DONE)+" "+String(Xsend[0])+" "+String(Xsend[1])+" "+String(Xsend[2])+" "+String(Xsend[3])+" "+String(Xsend[4])+" "+String(Xsend[5])+" "+String(Jinverse[0])+" "+String(Jinverse[1])+" "+String(Jinverse[2])+" "+String(Jinverse[3])+" "+String(Jinverse[4])+" "+String(Jinverse[5]);
  Serial.println(ab);
}
void setup()
{
  cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    
    //Setup Timer/Counter1 
    TCCR1B = (1 << CS12) |(0 << CS11) | (0 << CS10);    // prescale = /8
    TCNT1 = 40536;
    TIMSK1 = (1 << TOIE1);                // Overflow interrupt enable   
  sei();     
          
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(2, emergencyStopInterrupt, FALLING);
  Serial.begin(115200);
  //Serial1.setTimeout(3);
  delay(20);

  pinMode(PUL1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(PUL5_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PUL6_PIN, OUTPUT);
  pinMode(DIR6_PIN, OUTPUT);

  pinMode(EN321_PIN, OUTPUT);
  pinMode(EN4_PIN, OUTPUT);
  pinMode(EN5_PIN, OUTPUT);
  pinMode(EN6_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DISA_PIN, OUTPUT);
  pinMode(MAG_PIN, OUTPUT);
  
  digitalWrite(PUL1_PIN, LOW); // gear ratio = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW); //LOW = negative direction
  
  digitalWrite(PUL2_PIN, LOW); // gear ratio = 4
  digitalWrite(DIR2_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL3_PIN, LOW); // gear ratio = 5
  digitalWrite(DIR3_PIN, LOW); //LOW = negative direction
  
  digitalWrite(PUL4_PIN, LOW); // gear ratio = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL5_PIN, LOW); // gear ratio = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL6_PIN, LOW); // gear ratio = 1
  digitalWrite(DIR6_PIN, LOW); //LOW = positive direction

  digitalWrite(MAG_PIN,LOW);

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH);
  digitalWrite(DISA_PIN, HIGH);


  //Serial.begin(9600);
}
int checkD = 0;
void loop()
{  
  DONE = 0;
  // Check for incoming data
  if (Serial.available() > 0) {
    dataIn = Serial.readString();  // Read the data as string
    
    if (dataIn == "enable") {
      digitalWrite(EN321_PIN, LOW);
      digitalWrite(EN4_PIN, LOW);
      digitalWrite(EN5_PIN, LOW);
      digitalWrite(EN6_PIN, LOW);
      digitalWrite(ENA_PIN, HIGH);
      digitalWrite(DISA_PIN, LOW);
      EMER = 0;
      DONE = 0;
      index = 0;
    }
    
    if (dataIn == "disable") {
      digitalWrite(EN321_PIN, HIGH);
      digitalWrite(EN4_PIN, HIGH);
      digitalWrite(EN5_PIN, HIGH);
      digitalWrite(EN6_PIN, HIGH);
      digitalWrite(ENA_PIN, LOW);
      digitalWrite(DISA_PIN, HIGH);
      EMER = 0;
      DONE = 0;
      index = 0;
    }

    if (dataIn.startsWith("s6")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos6 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, curPos3, curPos4, curPos5, futPos6};
      ForwardK(Jfinal,Xsend);
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s5")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos5 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, curPos3, curPos4, futPos5, curPos6};
      ForwardK(Jfinal,Xsend);
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s4")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos4 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, curPos3, futPos4, curPos5, curPos6};
      ForwardK(Jfinal,Xsend);
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s3")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos3 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, curPos2, futPos3, curPos4, curPos5, curPos6};
      ForwardK(Jfinal,Xsend);
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    if (dataIn.startsWith("s2")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos2 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={curPos1, futPos2, curPos3, curPos4, curPos5, curPos6};
      ForwardK(Jfinal,Xsend);
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }
    
    if (dataIn.startsWith("s1")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futPos1 = dataInS.toFloat();
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfinal[6]={futPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      ForwardK(Jfinal, Xsend);
      goStrightLine(Jinitial, Jfinal, curSpeed, 0.75e-10, 0.0, 0.0);
    }

    // set the maximum speed for the move
    if (dataIn.startsWith("ss")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      curSpeed = (dataInS.toFloat()/100)*1e-4;
    }

    // If button "SAVE" is pressed
    if (dataIn.startsWith("save")) {
      String dataInS = dataIn.substring(4, dataIn.length());
      float velG=0.25e-4;
      Joint1[index] = curPos1;  // save position into the array
      Joint2[index] = curPos2;
      Joint3[index] = curPos3;
      Joint4[index] = curPos4;
      Joint5[index] = curPos5;
      Joint6[index] = curPos6;
      Magnic[index]= digitalRead(MAG_PIN);
      index++;  
                            // Increase the array index                 // Increase the array index
    }
    if (dataIn =="CALIB HOME")
    {
      curPos1 = 0.0;
      curPos2 = -78.51;
      curPos3 = 73.90;
      curPos4 = 0.0;
      curPos5 = -90.0;
      curPos6 = 0.0;
    }
    if ( dataIn == "HOME") {
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      digitalWrite(MAG_PIN, LOW);
      float Jhome[6]={0,-78.51,73.9,0,-90,0};
      float velG=0.25e-4;
      ForwardK(Jhome, Xsend);
      goStrightLine(Jinitial, Jhome,0.4e-4, 0.5e-10,0, 0.0 );
    }

     if ( dataIn == "HOMEHAI") {
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      digitalWrite(MAG_PIN, LOW);
      float Jhome[6]={0,0,0,0,0,0};
      float velG=0.25e-4;
      ForwardK(Jhome, Xsend); 
      goStrightLine(Jinitial, Jhome,0.4e-4, 0.5e-10,0, 0.0 );    
    }

    if (dataIn.startsWith("run")) {
      float velG=0.25e-4;
      float Jinit[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Jfin[6]={Joint1[0], Joint2[0], Joint3[0], Joint4[0], Joint5[0], Joint6[0]};
      goStrightLine(Jinit, Jfin, 0.4e-4, 0.75e-10, velG, 0.0 );
      for (int i = 0; i <= index-2; i++) {  // Run through all steps(index)
        if(EMER == 0)
        {
        float Jinit[6]={Joint1[i], Joint2[i], Joint3[i], Joint4[i], Joint5[i], Joint6[i]};
        float Jfin[6]={Joint1[i+1], Joint2[i+1], Joint3[i+1], Joint4[i+1], Joint5[i+1], Joint6[i+1]};
        digitalWrite(MAG_PIN, Magnic[i]);
        delay(500);
        goStrightLine(Jinit, Jfin, 0.4e-4, 0.75e-10, velG, 0 );
        }
        else{loop();}
        digitalWrite(MAG_PIN, Magnic[index]);
      }
    }
    if (dataIn == "ON_MAGNETIC"){
        digitalWrite(MAG_PIN,HIGH);  
      }

    if (dataIn == "OFF_MAGNETIC"){
        digitalWrite(MAG_PIN,LOW);  
    }

    if (dataIn.startsWith("EMER")){
        EMER = 1;
        index = 0;
    }

    if(dataIn.startsWith("1")){
      DONE = 0;
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.801, 0.0, 172 , 180};
      float X12[6]={-3.813, 158.473, 176.402, 90.98, 186.95, 180.0};
      float X13[6]={-30.797, 154.467, 126.730, -179.2, 104.93, -12.96};
      float X14[6]={-68.51, 150.336, 107.34, 178.190, 96.65, -3.43};
      float X15[6]={-95.341, 145.663, 107.297, -174.98, 96.2, -5.88};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      goStrightLine(Jhome, J1, 0.4e-4, 0.75e-10, 0, 0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, Jhome, 0.4e-4, 0.75e-10, 0, 0);
      ForwardK(Jhome, Xsend);
      goStrightLine(Jhome, J14, 0.4e-4, 0.75e-10, velG, velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J15, 0.3e-4, 0.5e-10, velG,0);
      delay(1000);
      ForwardK(J15, Xsend);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, Jhome, 0.4e-4, 0.75e-10, 0, 0);
      ForwardK(Jhome, Xsend);
      DONE = 123;
      delay(500);
      DONE = 123;
      delay(500);
      DONE = 123;
      }

    if(dataIn.startsWith("set1")){
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.801, 0.0, 172 , 180};
      float X12[6]={-3.813, 158.473, 176.402, 90.98, 186.95, 180.0};
      float X13[6]={-30.797, 154.467, 126.730, -179.2, 104.93, -12.96};
      float X14[6]={-68.51, 150.336, 107.34, 178.190, 96.65, -3.43};
      float X15[6]={-95.341, 145.663, 107.297, -174.98, 96.2, -5.88};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      float J0[6]={0,0,0,0,0,0};
      goStrightLine(Jhome, J0, curSpeed, 0.75e-10, 0, velG);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J1, curSpeed, 0.75e-10, velG, 0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, J0, curSpeed, 0.75e-10, 0, velG);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J14, curSpeed, 0.75e-10, velG, velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J15, curSpeed, 0.75e-10, velG,0);
      delay(1000);
      ForwardK(J15, Xsend);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J0, curSpeed, 0.75e-10, 0, velG);
      ForwardK(J14, Xsend);
      goStrightLine(J0, Jhome, curSpeed, 0.75e-10, velG, 0);
      ForwardK(Jhome, Xsend);
      delay(500);
      }

    if(dataIn.startsWith("2")){
      DONE = 0;
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.801, 0.0, 172 , 180};
      float X12[6]={-3.813, 258.473, 176.402, 90.98, 186.95, 180.0};
      float X13[6]={-30.797, 254.467, 126.730, -179.2, 104.93, -12.96};
      float X14[6]={-68.51, 250, 107.34, 178.190, 96.65, -3.43};
      float X15[6]={-95.341, 245.663, 107.34, -174.98, 96.2, -5.88};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      goStrightLine(Jhome, J1, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, Jhome, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(Jhome, Xsend);
      goStrightLine(Jhome, J15, 0.38e-4, 0.8e-10, velG,0);
      ForwardK(J15, Xsend);
      delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      //goStrightLine(J15, J13,0.4e-4, 0.75e-10, 0,velG);
      //ForwardK(J13, Xsend);
      goStrightLine(J15, Jhome, 0.4e-4, 0.75e-10, velG,velG);
      ForwardK(Jhome, Xsend);
      DONE = 123;
      delay(500);
      DONE = 123;
      delay(500);
      DONE = 123;
    }

    if(dataIn.startsWith("set2")){
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.801, 0.0, 172 , 180};
      float X12[6]={-3.813, 258.473, 176.402, 90.98, 186.95, 180.0};
      float X13[6]={-30.797, 254.467, 126.730, -179.2, 104.93, -12.96};
      float X14[6]={-68.51, 250, 107.34, 178.190, 96.65, -3.43};
      float X15[6]={-95.341, 245.663, 107.34, -174.98, 96.2, -5.88};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      float J0[6]={0,0,0,0,0,0};
      goStrightLine(Jhome, J0, curSpeed, 0.75e-10, 0, 0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J1, curSpeed, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, J0, curSpeed, 0.75e-10, 0,0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J15, curSpeed, 0.75e-10, 0,velG);
      ForwardK(J15, Xsend);
      delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J0,curSpeed, 0.75e-10, 0,velG);
      ForwardK(J0, Xsend);
      goStrightLine(J0, Jhome, curSpeed, 0.75e-10, velG,0);
      ForwardK(Jhome, Xsend);
      delay(500);
    }

      if(dataIn.startsWith("3")){
      DONE = 0;
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.5, 0.0, 172 , 180};
      float X12[6]={65.898, 180.891, 269.0, 70, 90, 180};
      float X13[6]={59.912, 164.608, 169.392, 70, 130, 180};
      float X14[6]={-56.832, 148.052, 159.82, 111.940, 170, 180.0};
      float X15[6]={-60.673, 158.059, 90.623, 111.097, 172.46, 180};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      goStrightLine(Jhome, J1, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, Jhome, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(Jhome, Xsend);
      goStrightLine(Jhome, J12, 0.4e-4, 0.75e-10, 1.2*velG, 0);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J13, 0.4e-4, 0.75e-10, 0,1.2*velG);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J14, 0.4e-4, 0.75e-10, 1.2*velG,1.2*velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J15,0.4e-4, 0.75e-10, 1.2*velG,0);
      ForwardK(J15, Xsend);
      delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J14, 0.4e-4, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J12, 0.4e-4, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, Jhome, 0.4e-4, 0.75e-10, 1.2*velG, 0);
      ForwardK(Jhome, Xsend);
      DONE = 123;
      delay(500);
      DONE = 123;
      delay(500);
      DONE = 123;
    }

    if(dataIn.startsWith("set3")){
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.5, 0.0, 172 , 180};
      float X12[6]={65.898, 180.891, 269.0, 70, 90, 180};
      float X13[6]={59.912, 164.608, 169.392, 70, 130, 180};
      float X14[6]={-56.832, 148.052, 159.82, 111.940, 170, 180.0};
      float X15[6]={-60.673, 158.059, 90.623, 111.097, 172.46, 180};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      float J0[6]={0,0,0,0,0,0};
      goStrightLine(Jhome, J0, curSpeed, 0.75e-10, 0, 0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J1, curSpeed, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, J0, curSpeed, 0.75e-10, 0,0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J12, curSpeed, 0.75e-10, 1.2*velG, 0);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J13, curSpeed, 0.75e-10, 0,1.2*velG);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J14, curSpeed, 0.75e-10, 1.2*velG,1.2*velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J15,curSpeed, 0.75e-10, 1.2*velG,0);
      ForwardK(J15, Xsend);
      delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J14, curSpeed, 0.75e-10, 0, 1.2*velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J12, curSpeed, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J0, curSpeed, 0.75e-10, 1.2*velG, 0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, Jhome, curSpeed, 0.75e-10, 1.2*velG, 0);
      ForwardK(Jhome, Xsend);

      delay(500);
    }

    if(dataIn.startsWith("4")){
      DONE = 0;
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.5, 0.0, 172 , 180};
      float X12[6]={-36.731,188.963,269.0,101.0,90,180};
      float X13[6]={-43.64, 224.5, 163.28, 101.0, 157.0, 180.0};
      float X14[6]={-46.55, 239.47, 234.38, 101.0, 127.0, 180.0};
      float X15[6]={-50.85, 261.59, 103.08, 101.0, 162.0, 180};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      goStrightLine(Jhome, J1, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, Jhome, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(Jhome, Xsend);
      goStrightLine(Jhome, J12, 0.4e-4, 0.75e-10, 1.2*velG, 0);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J13,0.4e-4, 0.75e-10, 0,1.2*velG);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J15, 0.4e-4, 0.75e-10, 1.2*velG,0);
      ForwardK(J15, Xsend);
      //goStrightLine(J14, J15, curSpeed, 0.9e-10, 1.2*velG,1.2*velG);
      delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J14, 0.4e-4, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J12, 0.4e-4, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, Jhome, 0.4e-4, 0.75e-10, 1.2*velG, 0);
      ForwardK(Jhome, Xsend);
      DONE = 123;
      delay(500);
      DONE = 123;
      delay(500);
      DONE = 123;
    }

    if(dataIn.startsWith("set4")){
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.5, 0.0, 172 , 180};
      float X12[6]={-36.731,188.963,269.0,101.0,90,180};
      float X13[6]={-43.64, 224.5, 163.28, 101.0, 157.0, 180.0};
      float X14[6]={-46.55, 239.47, 234.38, 101.0, 127.0, 180.0};
      float X15[6]={-50.85, 261.59, 103.08, 101.0, 162.0, 180};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      float J0[6]={0,0,0,0,0,0};
      goStrightLine(Jhome, J0, curSpeed, 0.75e-10, 0, 0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J1, curSpeed, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, J0, curSpeed, 0.75e-10, 0,0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J12, curSpeed, 0.75e-10, 1.2*velG, 0);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J13,curSpeed, 0.75e-10, 0,1.2*velG);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J15, curSpeed, 0.75e-10, 1.2*velG,0);
      ForwardK(J15, Xsend);
      //goStrightLine(J14, J15, curSpeed, 0.9e-10, 1.2*velG,1.2*velG);
      delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J14, curSpeed, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J0, curSpeed, 0.75e-10, 1.2*velG, 1.2*velG);
      ForwardK(J0, Xsend);
      goStrightLine(J0, Jhome, curSpeed, 0.75e-10, 1.2*velG, 0);
      ForwardK(Jhome, Xsend);
      delay(500);
    }

    if(dataIn.startsWith("6")){
      DONE = 0;
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 78, 0.0, 172 , 180};
      float X12[6]={30.11, 190.13, 269.0, 81.0, 90, -90};
      float X13[6]={35.75, 225.69, 149.96, 81.0, 165.0, -90};
      float X14[6]={38.54, 243.35, 194.87, 81.0, 145.0, -90};
      float X15[6]={39.71, 250.72, 104.82, 81.0, 170.0, -90};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      goStrightLine(Jhome, J1, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, Jhome,0.4e-4, 0.75e-10, 0,0);
      ForwardK(Jhome, Xsend);
      goStrightLine(Jhome, J12, 0.4e-4, 0.75e-10, 0, velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J13, 0.4e-4, 0.75e-10, velG,velG);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J15, 0.4e-4, 0.75e-10, velG,0);
      ForwardK(J15, Xsend);
      //goStrightLine(J14, J15, curSpeed, 0.9e-10, 1.2*velG,1.2*velG);
      //delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J14, 0.4e-4, 0.75e-10, 0, velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J12, 0.4e-4, 0.75e-10, velG, velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, Jhome, 0.4e-4, 0.75e-10, velG, 0);
      ForwardK(Jhome, Xsend);
      DONE = 123;
      delay(500);
      DONE = 123;
      delay(500);
      DONE = 123;
    }

    if(dataIn.startsWith("set6")){
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 78, 0.0, 172 , 180};
      float X12[6]={30.11, 190.13, 269.0, 81.0, 90, -90};
      float X13[6]={35.75, 225.69, 149.96, 81.0, 165.0, -90};
      float X14[6]={38.54, 243.35, 194.87, 81.0, 145.0, -90};
      float X15[6]={39.71, 250.72, 104.82, 81.0, 170.0, -90};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      InverseK(X15, J15);
      float velG=0.25e-4;
      float J0[6]={0,0,0,0,0,0};
      goStrightLine(Jhome, J0, curSpeed, 0.75e-10, 0, 0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J1, curSpeed, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, J0,curSpeed, 0.75e-10, 0,0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J12, curSpeed, 0.75e-10, 0, velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, J13, curSpeed, 0.75e-10, velG,velG);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J15, curSpeed, 0.75e-10, velG,0);
      ForwardK(J15, Xsend);
      //goStrightLine(J14, J15, curSpeed, 0.9e-10, 1.2*velG,1.2*velG);
      //delay(1000);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J15, J14, curSpeed, 0.75e-10, 0, velG);
      ForwardK(J14, Xsend);
      goStrightLine(J14, J0, curSpeed, 0.75e-10, velG, velG);
      ForwardK(J0, Xsend);
      goStrightLine(J0, Jhome, curSpeed, 0.75e-10, 0, 0);
      ForwardK(Jhome, Xsend);
      delay(500);
    }

    if(dataIn.startsWith("5")){
      DONE = 0;
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.5, 0.0, 172 , 180};
      float X12[6]={65.84, 180.89, 269.0, 70.0, 90, 139.0};
      float X13[6]={49.69, 136.51, 135.35, 70.0, 160.0, 139.0};
      float X14[6]={52.28, 143.65, 89.49, 70.0, 170.0, 139.0};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      float velG=0.25e-4;
      goStrightLine(Jhome, J1, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, Jhome, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(Jhome, Xsend);
      goStrightLine(Jhome, J12, 0.4e-4, 0.75e-10, velG, 0);
     ForwardK(J12, Xsend);
      goStrightLine(J12, J13, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J14, 0.4e-4, 0.75e-10, 0,0);
      ForwardK(J14, Xsend);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J14, J12, 0.4e-4, 0.75e-10, 0, velG);
      ForwardK(J12, Xsend);
      goStrightLine(J12, Jhome,0.4e-4, 0.75e-10, velG, 0);
      ForwardK(Jhome, Xsend);
      DONE = 123;
      delay(500);
      DONE = 123;
      delay(500);
      DONE = 123;
    }

    if(dataIn.startsWith("set5")){
      float Jhome[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6}; //{x, y, z, ZYZ Euler angles}//home
      float X1[6]={228.926, 0.0, 80.5, 0.0, 172 , 180};
      float X12[6]={65.84, 180.89, 269.0, 70.0, 90, 139.0};
      float X13[6]={49.69, 136.51, 135.35, 70.0, 160.0, 139.0};
      float X14[6]={52.28, 143.65, 89.49, 70.0, 170.0, 139.0};
      float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
      InverseK(X1, J1);
      InverseK(X12, J12);
      InverseK(X13, J13);
      InverseK(X14, J14);
      float velG=0.25e-4;
      float J0[6]={0,0,0,0,0,0};
      goStrightLine(Jhome, J0, curSpeed, 0.75e-10, 0, 0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J1, curSpeed, 0.75e-10, 0,0);
      ForwardK(J1, Xsend);
      digitalWrite(MAG_PIN, HIGH);
      delay(1000);
      goStrightLine(J1, J0, curSpeed, 0.75e-10, 0,0);
      ForwardK(J0, Xsend);
      goStrightLine(J0, J12, curSpeed, 0.75e-10, velG, 0);
     ForwardK(J12, Xsend);
      goStrightLine(J12, J13, curSpeed, 0.75e-10, 0,0);
      ForwardK(J13, Xsend);
      goStrightLine(J13, J14, curSpeed, 0.75e-10, 0,0);
      ForwardK(J14, Xsend);
      digitalWrite(MAG_PIN, LOW);
      delay(1000);
      goStrightLine(J14, J0, curSpeed, 0.75e-10, 0, velG);
      ForwardK(J0, Xsend);
      goStrightLine(J0, Jhome,curSpeed, 0.75e-10, velG, 0);
      ForwardK(Jhome, Xsend);
      delay(500);
    }

    if (dataIn.startsWith("ix")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futX = dataInS.toFloat();
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
    }

    if (dataIn.startsWith("iy")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futY = dataInS.toFloat();
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
    }

    if (dataIn.startsWith("iz")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      futZ = dataInS.toFloat();
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
    }

    if (dataIn.startsWith("irx")) {
      String dataInS = dataIn.substring(3, dataIn.length());
      futRx = dataInS.toFloat();
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
    }

    if (dataIn.startsWith("iry")) {
      String dataInS = dataIn.substring(3, dataIn.length());
      futRy = dataInS.toFloat();
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
    }

    if (dataIn.startsWith("irz")) {
      String dataInS = dataIn.substring(3, dataIn.length());
      futRz = dataInS.toFloat();
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
    }

    if (dataIn.startsWith("re")) { 
      futX = 0;
      futY = 0;
      futZ = 0;
      futRx = 0;
      futRy = 0;
      futRz = 0;
      //float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      Jinverse[0] = 0;
      Jinverse[1] = 0;
      Jinverse[2] = 0;
      Jinverse[3] = 0;
      Jinverse[4] = 0;
      Jinverse[5] = 0;
    }

   if (dataIn.startsWith("go")) {
      String dataInS = dataIn.substring(2, dataIn.length());
      float Jinitial[6]={curPos1, curPos2, curPos3, curPos4, curPos5, curPos6};
      float Xfinal[6]={futX, futY, futZ, futRx, futRy, futRz};
      InverseK(Xfinal,Jinverse);
      goStrightLine(Jinitial, Jinverse, curSpeed, 0.75e-10, 0.25e-4, 0);
    }
  }
}


void goStrightLine(float* xfi, float* xff, float vel0, float acc0, float velini, float velfin){
  //

  float lmax = max(abs(xff[0]-xfi[0]),abs(xff[1]-xfi[1]));
  lmax = max(lmax,abs(xff[2]-xfi[2]));
  lmax = max(lmax,abs(xff[3]-xfi[3]));
  lmax = max(lmax,abs(xff[4]-xfi[4]));
  lmax = max(lmax,abs(xff[5]-xfi[5]));
  unsigned long preMil = micros();
  double l = 0.0;
  vel0 = min(vel0,sqrt(lmax*acc0+0.5*velini*velini+0.5*velfin*velfin));
  unsigned long curMil = micros();
  unsigned long t = 0;
  double tap = vel0/acc0-velini/acc0;
  double lap = velini*tap+acc0*tap*tap/2.0;
  double lcsp = lmax-(vel0*vel0/2.0/acc0-velfin*velfin/2.0/acc0);
  double tcsp = (lcsp-lap)/vel0+tap;
  double tfin = vel0/acc0-velfin/acc0+tcsp;

  while (curMil-preMil<=tfin){ 

    t = curMil-preMil;
    //acceleration phase
    if (t<=tap) {
      l = velini*t+acc0*t*t/2.0;
    }
    //contant maximum speed phase
    if (t>tap && t<=tcsp) {
      l = lap+vel0*(t-tap);
    }
    //deceleration phase
    if (t>tcsp) {
      l = lcsp+vel0*(t-tcsp)-acc0*(t-tcsp)*(t-tcsp)/2.0;
    }

    if (EMER == 1) {break;}
    
    //trajectory x and y as a function of l
    float Xx[6];
    Xx[0]=xfi[0]+(xff[0]-xfi[0])/lmax*l;
    Xx[1]=xfi[1]+(xff[1]-xfi[1])/lmax*l;
    Xx[2]=xfi[2]+(xff[2]-xfi[2])/lmax*l;
    Xx[3]=xfi[3]+(xff[3]-xfi[3])/lmax*l;
    Xx[4]=xfi[4]+(xff[4]-xfi[4])/lmax*l;
    Xx[5]=xfi[5]+(xff[5]-xfi[5])/lmax*l;

    goTrajectory(Xx);
    curMil = micros();

  }
}



void goTrajectory(float* Jf){
  
  //execution
  int delF=0;
  // joint #1
  if (Jf[0]-curPos1>0.0) { // positive direction of rotation
    digitalWrite(DIR1_PIN, HIGH);
    while (Jf[0]-curPos1>dl1/2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 + dl1/2.0;
      if (Jf[0]-curPos1>dl1/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR1_PIN, LOW);
    while (-Jf[0]+curPos1>dl1/2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 - dl1/2.0;
      if (-Jf[0]+curPos1>dl1/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #2
  if (Jf[1]-curPos2>0.0) { // positive direction of rotation
    digitalWrite(DIR2_PIN, HIGH);
    while (Jf[1]-curPos2>dl2/2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 + dl2/2.0;
      if (Jf[1]-curPos2>dl2/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR2_PIN, LOW);
    while (-Jf[1]+curPos2>dl2/2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 - dl2/2.0;
      if (-Jf[1]+curPos2>dl2/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #3
  if (Jf[2]-curPos3>0.0) { // positive direction of rotation
    digitalWrite(DIR3_PIN, LOW);
    while (Jf[2]-curPos3>dl3/2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 + dl3/2.0;
      if (Jf[2]-curPos3>dl3/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR3_PIN, HIGH);
    while (-Jf[2]+curPos3>dl3/2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 - dl3/2.0;
      if (-Jf[2]+curPos3>dl3/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #4
  if (Jf[3]-curPos4>0.0) { // positive direction of rotation
    digitalWrite(DIR4_PIN, HIGH);
    while (Jf[3]-curPos4>dl4/2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 + dl4/2.0;
      if (Jf[3]-curPos4>dl4/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR4_PIN, LOW);
    while (-Jf[3]+curPos4>dl4/2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 - dl4/2.0;
      if (-Jf[3]+curPos4>dl4/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #5
  if (Jf[4]-curPos5>0.0) { // positive direction of rotation
    digitalWrite(DIR5_PIN, HIGH);
    while (Jf[4]-curPos5>dl5/2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 + dl5/2.0;
      if (Jf[4]-curPos5>dl5/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR5_PIN, LOW);
    while (-Jf[4]+curPos5>dl5/2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 - dl5/2.0;
      if (-Jf[4]+curPos5>dl5/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #6
  if (Jf[5]-curPos6>0.0) { // positive direction of rotation
    digitalWrite(DIR6_PIN, HIGH);
    while (Jf[5]-curPos6>dl6/2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 + dl6/2.0;
      if (Jf[5]-curPos6>dl6/2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR6_PIN, LOW);
    while (-Jf[5]+curPos6>dl6/2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 - dl6/2.0;
      if (-Jf[5]+curPos6>dl6/2.0) {
        delayMicroseconds(delF);
      }
    }
  }
}

void InverseK(float* Xik, float* Jik)
{
  // inverse kinematics
  // input: Xik - pos value for the calculation of the inverse kinematics
  // output: Jfk - joints value for the calculation of the inversed kinematics
  
  // from deg to rad
  // Xik(4:6)=Xik(4:6)*pi/180;
  Xik[3]=Xik[3]*PI/180.0;
  Xik[4]=Xik[4]*PI/180.0;
  Xik[5]=Xik[5]*PI/180.0;
  
  // Denavit-Hartenberg matrix
  float theta[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0}; // theta=[0; -90+0; 0; 0; 0; 0];
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; // alfa=[-90; 0; -90; 90; -90; 0];
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0}; // r=[47; 110; 26; 0; 0; 0];
  float d[6]={d1, 0.0, d3, d4, 0.0, d6}; // d=[133; 0; 0; 117.5; 0; 28];
  // from deg to rad
  MatrixScale(theta, 6, 1, PI/180.0); // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0); // alfa=alfa*pi/180;
  
  // work frame
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  
  // tool frame
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  
  // work frame transformation matrix
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  
  // tool frame transformation matrix
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  
  // total transformation matrix
  float Twt[16];
  pos2tran(Xik, Twt); // Twt=pos2tran(Xik);
  
  // find T06
  float inTwf[16], inTtf[16], Tw6[16], T06[16];
  invtran(Twf, inTwf); // inTwf=invtran(Twf);
  invtran(Ttf, inTtf); // inTtf=invtran(Ttf);
  MatrixMultiply(Twt, inTtf, 4, 4, 4, Tw6); // Tw6=Twt*inTtf;
  MatrixMultiply(inTwf, Tw6, 4, 4, 4, T06); // T06=inTwf*Tw6;
  
  // positon of the spherical wrist
  float Xsw[3];
  // Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
  Xsw[0]=T06[0*4 + 3]-d[5]*T06[0*4 + 2];
  Xsw[1]=T06[1*4 + 3]-d[5]*T06[1*4 + 2];
  Xsw[2]=T06[2*4 + 3]-d[5]*T06[2*4 + 2];
  
  // joints variable
  // Jik=zeros(6,1);
  // first joint
  Jik[0]=atan2(Xsw[1],Xsw[0])-atan2(d[2],sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])); // Jik(1)=atan2(Xsw(2),Xsw(1))-atan2(d(3),sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2));
  // second joint
  Jik[1]=PI/2.0
  -acos((r[1]*r[1]+(Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])-(r[2]*r[2]+d[3]*d[3]))/(2.0*r[1]*sqrt((Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))))
  -atan((Xsw[2]-d[0])/(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])); // Jik(2)=pi/2-acos((r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));
  // third joint
  Jik[2]=PI
  -acos((r[1]*r[1]+r[2]*r[2]+d[3]*d[3]-(Xsw[2]-d[0])*(Xsw[2]-d[0])-(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))/(2*r[1]*sqrt(r[2]*r[2]+d[3]*d[3])))
  -atan(d[3]/r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));
  // last three joints
  float T01[16], T12[16], T23[16], T02[16], T03[16], inT03[16], T36[16];
  DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
  DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
  DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
  MatrixMultiply(T01, T12, 4, 4, 4, T02); // T02=T01*T12;
  MatrixMultiply(T02, T23, 4, 4, 4, T03); // T03=T02*T23;
  invtran(T03, inT03); // inT03=invtran(T03);
  MatrixMultiply(inT03, T06, 4, 4, 4, T36); // T36=inT03*T06;
  // forth joint
  Jik[3]=atan2(-T36[1*4+2], -T36[0*4+2]); // Jik(4)=atan2(-T36(2,3),-T36(1,3));
  // fifth joint
  Jik[4]=atan2(sqrt(T36[0*4+2]*T36[0*4+2]+T36[1*4+2]*T36[1*4+2]), T36[2*4+2]); // Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
  // sixth joints
  Jik[5]=atan2(-T36[2*4+1], T36[2*4+0]); // Jik(6)=atan2(-T36(3,2),T36(3,1));
  // rad to deg
  MatrixScale(Jik, 6, 1, 180.0/PI); // Jik=Jik/pi*180;
}

void ForwardK(float* Jfk, float* Xfk)
{
  // forward kinematics
  // input: Jfk - joints value for the calculation of the forward kinematics
  // output: Xfk - pos value for the calculation of the forward kinematics
  
  // Denavit-Hartenberg matrix
  float theTemp[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0};
  float theta[6];
  MatrixAdd(theTemp, Jfk, 6, 1, theta); // theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0}; // alfa=[-90; 0; -90; 90; -90; 0];
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0}; // r=[47; 110; 26; 0; 0; 0];
  float d[6]={d1, 0.0, d3, d4, 0.0, d6}; // d=[133; 0; 7; 117.5; 0; 28];
  // from deg to rad
  MatrixScale(theta, 6, 1, PI/180.0); // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0); // alfa=alfa*pi/180;
  
  // work frame
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  
  // tool frame
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  
  // work frame transformation matrix
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  
  // tool frame transformation matrix
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  
  // DH homogeneous transformation matrix
  float T01[16], T12[16], T23[16], T34[16], T45[16], T56[16];
  DH1line(theta[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1),alfa(1),r(1),d(1));
  DH1line(theta[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2),alfa(2),r(2),d(2));
  DH1line(theta[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3),alfa(3),r(3),d(3));
  DH1line(theta[3], alfa[3], r[3], d[3], T34); // T34=DH1line(theta(4),alfa(4),r(4),d(4));
  DH1line(theta[4], alfa[4], r[4], d[4], T45); // T45=DH1line(theta(5),alfa(5),r(5),d(5));
  DH1line(theta[5], alfa[5], r[5], d[5], T56); // T56=DH1line(theta(6),alfa(6),r(6),d(6));

  float Tw1[16], Tw2[16], Tw3[16], Tw4[16], Tw5[16], Tw6[16], Twt[16];
  MatrixMultiply(Twf, T01, 4, 4, 4, Tw1); // Tw1=Twf*T01;
  MatrixMultiply(Tw1, T12, 4, 4, 4, Tw2); // Tw2=Tw1*T12;
  MatrixMultiply(Tw2, T23, 4, 4, 4, Tw3); // Tw3=Tw2*T23;
  MatrixMultiply(Tw3, T34, 4, 4, 4, Tw4); // Tw4=Tw3*T34;
  MatrixMultiply(Tw4, T45, 4, 4, 4, Tw5); // Tw5=Tw4*T45;
  MatrixMultiply(Tw5, T56, 4, 4, 4, Tw6); // Tw6=Tw5*T56;
  MatrixMultiply(Tw6, Ttf, 4, 4, 4, Twt); // Twt=Tw6*Ttf;
  
  // calculate pos from transformation matrix
  tran2pos(Twt, Xfk); // Xfk=tran2pos(Twt);
  // Xfk(4:6)=Xfk(4:6)/pi*180;
  Xfk[3]=Xfk[3]/PI*180.0;
  Xfk[4]=Xfk[4]/PI*180.0;
  Xfk[5]=Xfk[5]/PI*180.0;
}

void invtran(float* Titi, float* Titf)
{
  // finding the inverse of the homogeneous transformation matrix
  // first row
  Titf[0*4 + 0] = Titi[0*4 + 0];
  Titf[0*4 + 1] = Titi[1*4 + 0];
  Titf[0*4 + 2] = Titi[2*4 + 0];
  Titf[0*4 + 3] = -Titi[0*4 + 0]*Titi[0*4 + 3]-Titi[1*4 + 0]*Titi[1*4 + 3]-Titi[2*4 + 0]*Titi[2*4 + 3];
  // second row
  Titf[1*4 + 0] = Titi[0*4 + 1];
  Titf[1*4 + 1] = Titi[1*4 + 1];
  Titf[1*4 + 2] = Titi[2*4 + 1];
  Titf[1*4 + 3] = -Titi[0*4 + 1]*Titi[0*4 + 3]-Titi[1*4 + 1]*Titi[1*4 + 3]-Titi[2*4 + 1]*Titi[2*4 + 3];
  // third row
  Titf[2*4 + 0] = Titi[0*4 + 2];
  Titf[2*4 + 1] = Titi[1*4 + 2];
  Titf[2*4 + 2] = Titi[2*4 + 2];
  Titf[2*4 + 3] = -Titi[0*4 + 2]*Titi[0*4 + 3]-Titi[1*4 + 2]*Titi[1*4 + 3]-Titi[2*4 + 2]*Titi[2*4 + 3];
  // forth row
  Titf[3*4 + 0] = 0.0;
  Titf[3*4 + 1] = 0.0;
  Titf[3*4 + 2] = 0.0;
  Titf[3*4 + 3] = 1.0;
}

void tran2pos(float* Ttp, float* Xtp)
{
  // pos from homogeneous transformation matrix
  Xtp[0] = Ttp[0*4 + 3];
  Xtp[1] = Ttp[1*4 + 3];
  Xtp[2] = Ttp[2*4 + 3];
  Xtp[4] = atan2(sqrt(Ttp[2*4 + 0]*Ttp[2*4 + 0] + Ttp[2*4 + 1]*Ttp[2*4 + 1]),Ttp[2*4 + 2]);
  Xtp[3] = atan2(Ttp[1*4 + 2]/sin(Xtp[4]),Ttp[0*4 + 2]/sin(Xtp[4]));
  Xtp[5] = atan2(Ttp[2*4 + 1]/sin(Xtp[4]),-Ttp[2*4 + 0]/sin(Xtp[4]));
}

void pos2tran(float* Xpt, float* Tpt)
{
  // pos to homogeneous transformation matrix
  // first row
  Tpt[0*4 + 0] = cos(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])-sin(Xpt[3])*sin(Xpt[5]);
  Tpt[0*4 + 1] = -cos(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])-sin(Xpt[3])*cos(Xpt[5]);
  Tpt[0*4 + 2] = cos(Xpt[3])*sin(Xpt[4]);
  Tpt[0*4 + 3] = Xpt[0];
  // second row
  Tpt[1*4 + 0] = sin(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])+cos(Xpt[3])*sin(Xpt[5]);
  Tpt[1*4 + 1] = -sin(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])+cos(Xpt[3])*cos(Xpt[5]);
  Tpt[1*4 + 2] = sin(Xpt[3])*sin(Xpt[4]);
  Tpt[1*4 + 3] = Xpt[1];
  // third row
  Tpt[2*4 + 0] = -sin(Xpt[4])*cos(Xpt[5]);
  Tpt[2*4 + 1] = sin(Xpt[4])*sin(Xpt[5]);
  Tpt[2*4 + 2] = cos(Xpt[4]);
  Tpt[2*4 + 3] = Xpt[2];
  // forth row
  Tpt[3*4 + 0] = 0.0;
  Tpt[3*4 + 1] = 0.0;
  Tpt[3*4 + 2] = 0.0;
  Tpt[3*4 + 3] = 1.0;
}

void DH1line(float thetadh, float alfadh, float rdh, float ddh, float* Tdh)
{
  // creats Denavit-Hartenberg homogeneous transformation matrix
  // first row
  Tdh[0*4 + 0] = cos(thetadh);
  Tdh[0*4 + 1] = -sin(thetadh)*cos(alfadh);
  Tdh[0*4 + 2] = sin(thetadh)*sin(alfadh);
  Tdh[0*4 + 3] = rdh*cos(thetadh);
  // second row
  Tdh[1*4 + 0] = sin(thetadh);
  Tdh[1*4 + 1] = cos(thetadh)*cos(alfadh);
  Tdh[1*4 + 2] = -cos(thetadh)*sin(alfadh);
  Tdh[1*4 + 3] = rdh*sin(thetadh);
  // third row
  Tdh[2*4 + 0] = 0.0;
  Tdh[2*4 + 1] = sin(alfadh);
  Tdh[2*4 + 2] = cos(alfadh);
  Tdh[2*4 + 3] = ddh;
  // forth row
  Tdh[3*4 + 0] = 0.0;
  Tdh[3*4 + 1] = 0.0;
  Tdh[3*4 + 2] = 0.0;
  Tdh[3*4 + 3] = 1.0;
}

void MatrixPrint(float* A, int m, int n, String label)
{
  // A = input matrix (m x n)
  int i, j;
  Serial.println();
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      Serial.print(A[n * i + j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void MatrixCopy(float* A, int n, int m, float* B)
{
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      B[n * i + j] = A[n * i + j];
    }
}

//Matrix Multiplication Routine
// C = A*B
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      C[n * i + j] = 0;
      for (k = 0; k < p; k++)
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
    }
}


//Matrix Addition Routine
void MatrixAdd(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A+B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] + B[n * i + j];
}


//Matrix Subtraction Routine
void MatrixSubtract(float* A, float* B, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A-B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] - B[n * i + j];
}


//Matrix Transpose Routine
void MatrixTranspose(float* A, int m, int n, float* C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[m * j + i] = A[n * i + j];
}

void MatrixScale(float* A, int m, int n, float k)
{
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      A[n * i + j] = A[n * i + j] * k;
}
