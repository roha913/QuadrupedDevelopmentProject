#define trig 4
#define echo 5



#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;


void setupForEachMotor(uint8_t dxl_id){
  // Get DYNAMIXEL information
  dxl.ping(dxl_id);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(dxl_id);
  dxl.setOperatingMode(dxl_id, OP_POSITION);
  dxl.torqueOn(dxl_id);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, dxl_id, 30);
}
void goToPosition(float p1, float p2, float p3, float p4){
  dxl.torqueOn(1);
  dxl.torqueOn(2);
  dxl.torqueOn(3);
  dxl.torqueOn(4);
  dxl.setGoalPosition(1, p1, UNIT_DEGREE);
  dxl.setGoalPosition(2, p2, UNIT_DEGREE);
  dxl.setGoalPosition(3, p3, UNIT_DEGREE);
  dxl.setGoalPosition(4, p4, UNIT_DEGREE);
  delay(500);
}

// Genetic Algorithm Parameters
 const int maxim = 150;
const int populationSize = 5;
const int geneLength = 12; // 4 servos * 3 steps  
const int maxGenerations = 10;//in the mBot robot it was 10000, but I shortened it to save time

//A struct describing an individual. Each individual has a "gene" array and a float describing the fitness, or how good the gene sequence is
struct Individual {
    int genes[geneLength]; 
    float fitness;
};


//Tools to see if COM is in base polygon, so that the robot doesn't fall over. Origin is midpoint of body. Assuming that is where COM is. OpenRB should be there.
//x-axis is length, y-axis is width

bool pit(float x, float y, float x1, float y1, float x2, float y2, float x3, float y3)
{
    int as_x = x - x1;
    int as_y = y - y1;

    bool s_ab = (x2 - x1) * as_y - (y2 - y1) * as_x > 0;

    if ((x3 - x1) * as_y - (y3 - y1) * as_x > 0 == s_ab) 
        return false;
    if ((x3 - x2) * (y - y2) - (y3 - y2)*(x - x2) > 0 != s_ab) 
        return false;
    return true;
}

bool PointInQuadrilateral(float x, float y, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, bool point1, bool point2, bool point3, bool point4){//only works for convex quadrilaterals
  if(point1 && point2 && point3 && point4){
      return pit(x, y, x1, y1, x2, y2, x3, y3) || pit(x, y, x1, y1, x2, y2, x4, y4) || pit(x, y, x2, y2, x3, y3, x4, y4) || pit(x, y, x1, y1, x3, y3, x4, y4);
  }
  if(point1 && point2 && point3){
    return pit(x, y, x1, x2, y1, y2, x3, y3);
  }
  if(point1 && point2 && point4){
    return pit(x, y, x1, y1, x2, y2, x4, y4);
  }
  if(point2 && point3 && point4){
    return pit(x, y, x2, y2, x3, y3, x4, y4);
  } 
  if(point1 && point3 && point4){
    return pit(x, y, x1, y1, x3, y3, x4, y4);
  }
  return false;
}


//End of polygon tool code


//An array of individuals describing the population
Individual population[populationSize];


//Initializing the population
void initializePopulation() {
    for (int i = 0; i < populationSize; i++) {
        for (int j = 0; j < geneLength; j++) {
            Serial.print("i: ");
            Serial.print(i);
            Serial.print(" , j: ");
            Serial.println(j);
            population[i].genes[j] = random(0, maxim); // Random value for each part of each codon
            //int width = 20;
            //population[i].genes[j] = random(60 - width, 60 + width);
            if(j%4 == 3){
                int x1 = 7 - 10.5*sin((3.14/180)*(population[i].genes[j - 2] + 30));//front-left
                int y1 = 3;
                int x2 = 6.5 + 10.5*sin((3.14/180)*(population[i].genes[j - 1] + 30));//front-right
                int y2 = 3;
                int x3 = -7 + 9.5*sin((3.14/180)*(population[i].genes[j] + 30));//back-right
                int y3 = -3;
                int x4 = -7 + 9.5*sin((3.14/180)*(population[i].genes[j - 3] + 30));//back-left
                int y4 = -3;
                if(!PointInQuadrilateral(0, 0, x1, y1, x2, y2, x3, y3, x4, y4, (population[i].genes[j - 2] + 30) < 150, (population[i].genes[j - 1] + 30) < 150, (population[i].genes[j] + 30) < 150, (population[i].genes[j - 3] + 30) < 150)){
                    j = 4*((int)(j/4));
                    String s = "Not in: ";
                    Serial.println(s + population[i].genes[j - 2] + "," + population[i].genes[j - 1] + "," + population[i].genes[j] + "," + population[i].genes[j - 3]);
                }else{
                    String s = "In Quadrilateral: ";
                    Serial.println(s + population[i].genes[j - 2] + "," + population[i].genes[j - 1] + "," + population[i].genes[j] + "," + population[i].genes[j - 3]);
                    //Serial.println(s + x1 + "," + y1 + "," + x2 + "," + y2 + "," + x3 + "," + y3 + "," + x4 + "," + y4);
                }
            } 
        }
        population[i].fitness = 0;
    }
}

/*
void initializePopulation() {
    for (int i = 0; i < populationSize; i++) {
        for (int j = 0; j < geneLength; j++) {
            population[i].genes[j] = random(0, maxim); // Random value for each part of each codon
        }
        population[i].fitness = 0;
    }
}
*/

void setup() {
  delay(5000);
  Serial.begin(9600);
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  Serial.println("About to setup");
  setupForEachMotor(1);
  setupForEachMotor(2);
  setupForEachMotor(3);
  setupForEachMotor(4);

  Serial.println("About to initializePopulation()");
  initializePopulation();
  //The delay is so that when uploading the code to get EEPROM values to test, you have time to upload before it overwrites the EEPROM value
  //delay(10000);
  Serial.println("Starting loop!");
}

//Getting the distance from the ultrasonic sensor


float readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);  // Clear the trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(echoPin, HIGH)*0.343/2;
}



//The below code is to control the motors based on the gene sequence.
void controlMotors(int indiv) {
    int gene[geneLength];
    for(int CRISPR = 0; CRISPR < geneLength; CRISPR++){
      gene[CRISPR] = population[indiv].genes[CRISPR]; 
    }

    float old_dist = readUltrasonicDistance(trig, echo);//To measure the change in distance
    for(int goTo_start = 0; goTo_start < 3; goTo_start++){
      goToPosition(gene[4*goTo_start], gene[4*goTo_start + 1], gene[4*goTo_start + 2], gene[4*goTo_start + 3]);
    }

    float new_dist = readUltrasonicDistance(trig, echo);
    population[indiv].fitness = abs(new_dist - old_dist);
}   
    


//Genetic algorithms by nature involve crossover, where different genes are combined in different parts to try to get "the best of both worlds"
void crossover(float geneA[], float geneB[]){//geneA and geneB are the genes we are combining
  for(int child = 0; child < populationSize; child++){//We want the population to be the same size, for simplicity
    float child_gene[geneLength];
    //We split each gene into three parts and mix and mash these three parts together randomly
    int firstPart = random(2);
    int secondPart = random(2);
    int thirdPart = random(2); 
    int startPoint = 0;
    int endPoint = int(geneLength/3);
    switch(firstPart){
      case 0:
        for(int j = startPoint; j < endPoint; j++){
          child_gene[j] = geneA[j];
        }
        break;
      case 1:
        for(int j = startPoint; j < endPoint; j++){
          child_gene[j] = geneB[j];
        }
        break;
    }
    startPoint = endPoint + 1;
    endPoint = startPoint + int(geneLength/3);
    switch(secondPart){
      case 0:
        for(int j = startPoint; j < endPoint; j++){
          child_gene[j] = geneA[j];
        }
        break;
      case 1:
        for(int j = startPoint; j < endPoint; j++){
          child_gene[j] = geneB[j];
        }
        break;
   }
   startPoint = endPoint + 1;
   endPoint = geneLength;
   switch(thirdPart){
      case 0:
        for(int j = startPoint; j < endPoint; j++){
          child_gene[j] = geneA[j];
        }
        break;
      case 1:
        for(int j = startPoint; j < endPoint; j++){
          child_gene[j] = geneB[j];
        }
        break;
    }
    //replacing one individual's genes with the new gene
    for(int splicer = 0; splicer < geneLength; splicer++){
      population[child].genes[splicer] = child_gene[splicer];
    }
 }
}

//In many implementations of the genetic algorithm, there is something called mutations. This introduces randomness in the process.
float mutation_rate = 0.1;
void mutation(){
  float rand_float = random(100)/100.0;
  if(rand_float < mutation_rate){
    int random_individual = random(populationSize);
    int random_GCAT = random(geneLength);
    population[random_individual].genes[random_GCAT] = random(0, maxim);
  }
}

//This finds the top two fitnesses. The first element of topTwo is the index of the top one, the second element is the fitness of the top, and the 
//third and fourth one is the same for the second-most top
float topTwo[4] = {-1, -1.0, -1, -1.0};
void findTopTwoFitnesses() {
    for (int i = 0; i < populationSize; i++) {
        float the_fit = population[i].fitness;
        if (the_fit > topTwo[1]) {
            topTwo[2] = topTwo[0];  // Move current top to second top
            topTwo[3] = topTwo[1];  // Move current top to second top
            topTwo[1] = the_fit;
            topTwo[0] = (float) i;
        } else if (the_fit > topTwo[3]) {
            topTwo[2] = (float) i;
            topTwo[3] = the_fit;
        }
    }
}

bool memoryWrittenTo = false;
int gen = 0;//generation number
float best_gene[geneLength];
void loop() {
  delay(500);
  if(millis() - 10000 < 300000 && gen <= maxGenerations){
    for(int ind = 0; ind < populationSize; ind++){//trying out all the individuals' genes
      controlMotors(ind);
      Serial.print("Ind: ");
      Serial.println(ind);
    }

    Serial.print("Entire population fitnesses: ");
    for (int i = 0; i < populationSize; i++) {
        Serial.print(population[i].fitness);
        Serial.print(",");
    }  
    Serial.println("");
    findTopTwoFitnesses();
    float top_fit = topTwo[1];
      //analogWrite(motorApwm, 255);
      //analogWrite(motorBpwm, 255);
      float geneA[geneLength];
      float geneB[geneLength];
      for(int splice = 0; splice < geneLength; splice++){
        geneA[splice] = population[(int)topTwo[0]].genes[splice];
        geneB[splice] = population[(int)topTwo[2]].genes[splice];
      }
      crossover(geneA, geneB);//We want to breed the top two genes together.
      mutation();
    for(int splice = 0; splice < geneLength; splice++){
      best_gene[splice] = population[(int)topTwo[0]].genes[splice];
    }
    gen++;
    
    Serial.print("Generation ");
    Serial.print(gen);
    Serial.println(":");
    for(int sanger = 0; sanger < geneLength; sanger++){
      Serial.print(int(best_gene[sanger]));
      Serial.print(",");
    }
    Serial.println("");
  }
}
