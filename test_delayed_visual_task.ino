#include <ShiftOutX.h>
#include <ShiftPinNo.h>
int numOfRegisters = 3;
byte* registerState;
const byte Latch_S2P=14;
const byte Clk_S2P=13;
const byte PE=12;
const byte Clk_P2S=27;
const byte Latch_P2S=26;
const byte Ser2=25;
const byte P2S_trigger=21;
//main_switch
const byte Main_switch =18;
int amount=500;int disp_speed=2;//dispende speed ms
const byte P2S_Out=32;//33
shiftOutX regOne(Latch_S2P, Ser2, Clk_S2P, MSBFIRST, 3); 
bool port_triggered=false;
bool trial_on=false;
bool dispensing =false;
bool Camera_on=false;
int port_trigger_debounce=500;// ms 
long prev_port_trigger=0;//ms
int trial_duration=30000;//ms
int ITI=1000;//ms
int trial_count=0;
byte shape_seq[]={0b001,0b010,0b100};//square,circle,triangle
String shape_string[3] = {"square", "circle", "triangle"};
byte shift_register1;   byte shift_register2;   byte shift_register3;
byte test_Sig= 0b00000000;
int Correct_shape;int Correct_port[2];
TaskHandle_t dispense_reward;//create a new task for running stepper motors
//stepper motor pins 
int Rwd_prt[6]={23,19,18,17,16,4};
// stepper motor shift register pins 
const byte Clk_Stp=4;
const byte Latch_Stp=16;
const byte Data_Stp=17;
const byte Camera_trigger=19;
const byte IR_break_pwr=22;
const byte IR_break_gnd=23;
int prev_cam_trigger=0;
int cam_trigger_debounce=2000;//ms
int port_num;
void IRAM_ATTR  Log_IR_Break(){
  if (trial_on==true && dispensing ==false){
    if (millis()-prev_port_trigger>port_trigger_debounce){
      prev_port_trigger=millis();
      port_triggered=true;
    }
  }
  }
void IRAM_ATTR Camera_pulse(){
      if (millis()-prev_cam_trigger>cam_trigger_debounce){
      Camera_on=true;
      prev_port_trigger=millis();
    }
  }
void setup() {
  // put your setup code here, to run once:
  registerState = new byte[numOfRegisters];
  for (size_t i = 0; i < numOfRegisters; i++) {
    registerState[i] = 0;
  }
pinMode(Latch_S2P,OUTPUT);
pinMode(Clk_S2P,OUTPUT);
pinMode(Latch_P2S,OUTPUT);
pinMode(Ser2,OUTPUT);
pinMode(Clk_P2S,OUTPUT);
pinMode(P2S_trigger,INPUT);
pinMode(Main_switch,INPUT);
pinMode(Clk_Stp,OUTPUT);pinMode(Latch_Stp,OUTPUT);pinMode(Data_Stp,OUTPUT);
pinMode(Main_switch,INPUT_PULLDOWN);
pinMode(Camera_trigger,INPUT_PULLDOWN);
pinMode(IR_break_pwr,OUTPUT);
pinMode(IR_break_gnd,INPUT_PULLDOWN);
attachInterrupt(Camera_trigger,Camera_pulse,HIGH);
attachInterrupt(P2S_trigger,Log_IR_Break,RISING);
digitalWrite(Latch_S2P, 1);
pinMode(P2S_Out,INPUT);
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    dispense_reward_code,   /* Task function. */
                    "dispense_reward",     /* name of task. */
                    5000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &dispense_reward,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

Serial.begin(9600);
//Serial.print("Task0 running on core ");
//Serial.println(xPortGetCoreID());
}
//Task1code: blinks an LED every 1000 ms
void dispense_reward_code( void * pvParameters ){
  delay(10);
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());
  
  for(;;){
    delay(100);
    if (Camera_on==true){
      Serial.print("Camera on, ");
      Serial.println(millis());
      Camera_on=false;
      detachInterrupt(digitalPinToInterrupt(Camera_trigger));
      }
  if (dispensing==true){
  Serial.print("dispensing reward:,");Serial.println(millis());
  //dispense(Rwd_prt[port_num], amount, disp_speed);
  byte port_shift_reg_data=0b00000000;
  bitWrite(port_shift_reg_data,port_num,1);
  for (int i =0; i<amount; i++){
   digitalWrite(Latch_Stp, LOW);
   shiftOut(Data_Stp, Clk_Stp, MSBFIRST, port_shift_reg_data);
   digitalWrite(Latch_Stp, HIGH);
   delay(disp_speed);
   digitalWrite(Latch_Stp, LOW);
   shiftOut(Data_Stp, Clk_Stp, MSBFIRST, 0b000000);
   digitalWrite(Latch_Stp, HIGH);
   delay(disp_speed);
  }
  dispensing=false;
  }
  } 
}
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(IR_break_pwr,HIGH);
  if (digitalRead(Main_switch)==HIGH){
  trial_count++;
  if(trial_count==1){print_experiment_parameters();}
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial.print("Trial No: ,");Serial.println(trial_count);
  Correct_shape=random(2);
  Serial.print("Correct shape is (index): ,");Serial.println(Correct_shape);
  Serial.print("Correct shape is (string): ,");Serial.println(shape_string[Correct_shape]);
  // blink all the lights to indicate trial start 
  blink_all();
  assign_shapes();
  display_shapes();
  int trial_start=millis();
  Serial.print("Trial start time: ,");Serial.println(trial_start);
  trial_on=true;
  while(millis()-trial_start<trial_duration){
    if (port_triggered==true)
    {
      port_triggered=false;
      //Serial.println("interrupted!");
      Read_port();
    }
  }
  Serial.print("trial end time:, ");Serial.println(millis());
  Serial.print("Inter-trial interval: , ");Serial.println(ITI);
  Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  trial_on=false;
  delay(ITI);
  }
  else {regOne.allOff();  digitalWrite(IR_break_pwr,LOW);}
}
void Read_port(){
  digitalWrite(Latch_P2S, 0);
  digitalWrite(Clk_P2S, 0);
  digitalWrite(Clk_P2S, 1);
  digitalWrite(Latch_P2S, 1); 
  digitalWrite(Clk_P2S, 0);
  byte value=shiftIn(P2S_Out, Clk_P2S, LSBFIRST);
  //Serial.print("P2S output:");Serial.println(value,BIN);
  for(int i =0; i<8;i++){
     if (bitRead(value,i)==1){
      Serial.print("port interrupted :,");Serial.print(millis());Serial.print(i-2);
      if (i-2==(Correct_port[0])){
        Correct_port[0]=9;//remove the correct port by assigning a value that will never occur 
        Serial.println(",correct");
        dispensing =true;
        port_num=i-2;
        }
      else if (i-2==(Correct_port[1])){
          Correct_port[1]=9;//remove the correct port by assigning a value that will never occur 
          dispensing =true;
          port_num=i-2;
          Serial.println(",correct");
          }
        else{Serial.println(",incorrect");}
        }
      }
    }
void blink_all(){
  for (int i =0;i<3;i++){
    int pause_time=250;
    regOne.allOn();
    delay(pause_time);
    regOne.allOff();
    delay(pause_time);
    delay(pause_time);
    }
  }
void assign_shapes(){
  int shapes_assigned[]={0,0,0,0,0,0};
  //assign triangle
    for (int i=0;i<2;i++){
    int shape_pos=random(6);
    while (shapes_assigned[shape_pos]!=0){shape_pos=random(6);}
    shapes_assigned[shape_pos]=1;
      if (Correct_shape==0){
        Correct_port[i]=shape_pos;
      }
    }
  //find squares
    for (int i=0;i<2;i++){
    int shape_pos=random(6);
    while (shapes_assigned[shape_pos]!=0){shape_pos=random(6);}
    shapes_assigned[shape_pos]=2;
      if (Correct_shape==1){
        Correct_port[i]=shape_pos;
      }
    }
  //find circles
    for (int i=0;i<2;i++){
    int shape_pos=random(6);
    while (shapes_assigned[shape_pos]!=0){shape_pos=random(6);}
    shapes_assigned[shape_pos]=3;
      if (Correct_shape==2){
        Correct_port[i]=shape_pos;
      }
    }
  //print configuration to serial port
  Serial.print("Shape sequence(index):,");
  Serial.print(shapes_assigned[0]-1);Serial.print(",");Serial.print(shapes_assigned[1]-1);Serial.print(",");
  Serial.print(shapes_assigned[2]-1);Serial.print(",");Serial.print(shapes_assigned[3]-1);Serial.print(",");
  Serial.print(shapes_assigned[4]-1);Serial.print(",");Serial.println(shapes_assigned[5]-1);
  Serial.print("Shape sequence(string):,");
  Serial.print(shape_string[shapes_assigned[0]-1]);Serial.print(",");Serial.print(shape_string[shapes_assigned[1]-1]);Serial.print(",");
  Serial.print(shape_string[shapes_assigned[2]-1]);Serial.print(",");Serial.print(shape_string[shapes_assigned[3]-1]);Serial.print(",");
  Serial.print(shape_string[shapes_assigned[4]-1]);Serial.print(",");Serial.println(shape_string[shapes_assigned[5]-1]);
  //assign shapes to register 1 
  shift_register1=(((shape_seq[shapes_assigned[0]-1]<<3)|(shape_seq[shapes_assigned[1]-1]))<<2|(0b00));
  //Serial.print("Sequence for shift register1:");Serial.println(shift_register1,BIN);
   //assign shapes to register 2 
   shift_register2=(((shape_seq[shapes_assigned[2]-1]<<3)|(shape_seq[shapes_assigned[3]-1]))<<2|(0b00));
  //Serial.print("Sequence for shift register2:");Serial.println(shift_register2,BIN);
    //assign shapes to register 3
  shift_register3=(((shape_seq[shapes_assigned[4]-1]<<3)|(shape_seq[shapes_assigned[5]-1]))<<2|(0b00));
  //Serial.print("Sequence for shift register3:");Serial.println(shift_register3,BIN);
  Serial.print("Correct port locations are: ,");Serial.print(Correct_port[0]);Serial.print(",");Serial.println(Correct_port[1]);
}
void display_shapes(){
  for (int n =0; n <8;n++){
    bool state =bitRead(shift_register1, 8-n);
    regWrite(n, state);}
  for (int n =0; n <8;n++){
    bool state =bitRead(shift_register2, 8-n);
    regWrite(n+8, state);}
  for (int n =0; n <8;n++){
    bool state =bitRead(shift_register3, 8-n);
    regWrite(n+16, state);}
  } 
void regWrite(int pin, bool state){
  //Determines register
  int reg = pin / 8;
  //Determines pin for actual register
  int actualPin = pin - (8 * reg);
  //Begin session
  digitalWrite(Latch_S2P, LOW);
  for (int i = 0; i < numOfRegisters; i++){
    //Get actual states for register
    byte* states = &registerState[i];
    //Update state
    if (i == reg){
      bitWrite(*states, actualPin, state);
    }
    //Write
    shiftOut(Ser2, Clk_S2P, MSBFIRST, *states);
  }
  //End session
  digitalWrite(Latch_S2P, HIGH);
}
void print_experiment_parameters(){
  Serial.println("====================================================");
Serial.println("Visual delay Task|Author: Vikram Pal Singh |2023");
Serial.println("Task parameters:");
Serial.print("port_trigger_debounce:,");Serial.println(port_trigger_debounce);
Serial.print("Trial duration:,");Serial.println(trial_duration);
Serial.print("Inter-trial interval:,");Serial.println(ITI);
Serial.print("Amount dispensed for correct attempt:,");Serial.print(amount);Serial.println(", steps");
Serial.print("delay between steps:,");Serial.print(disp_speed);Serial.println(", ms");
Serial.println("====================================================");}
