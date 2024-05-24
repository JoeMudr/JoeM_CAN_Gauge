#include <Arduino.h>
#include <mcp2515.h>
#include <Stepper.h>
#include <EEPROM.h>

#define Firmware 240524

#define EEPROM_ConfVersion 2

struct conf {
  uint8_t ConfVersion;
  uint32_t CANID;
  uint8_t Position;
  int8_t Bits;
  float Gain;
  float ValMin;
  float ValMax;
};

conf Settings;

can_frame canMsg;
MCP2515 CAN(10); // CS PIN 10

#define  STEPS  720    // steps per revolution (630 = limited to 315°)
#define  COIL1  7
#define  COIL2  6
#define  COIL3  8
#define  COIL4  9
#define  max_Steps 630


Stepper stepper(STEPS, COIL1, COIL2, COIL3, COIL4);

//int val = 0;
int rel_pos = 0;    
int abs_pos = 0;   

bool CanDebug = false;
bool ValDebug = false;

String incomingByte; 

void Config();
void Stepper_Drive(float _val);
void Serial_clear();
void Can_Debug(can_frame canMsg);
void Can_Decode(can_frame canMsg);

void setup() {
  EEPROM.get(0, Settings);
  if(Settings.ConfVersion != EEPROM_ConfVersion){
    Settings.ConfVersion = EEPROM_ConfVersion;
    Settings.CANID = 0;
    Settings.Position = 0;
    Settings.Bits = 0;
    Settings.Gain = 0.0;
    Settings.ValMin = 0.0;
    Settings.ValMax = 0.0;
  }
  CAN.reset();
  CAN.setBitrate(CAN_500KBPS, MCP_8MHZ);
  CAN.setNormalMode();
  stepper.setSpeed(60);   // set the motor speed to 30 RPM (360 PPS aprox.).
  stepper.step(-670);     // Reset Position.
  Serial.begin(115200);
  if(Serial){
    Config();
  }
}

void loop() {
  if (Serial.available()) {Config();}  
  if (CAN.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if((CanDebug && canMsg.can_id == Settings.CANID) || (CanDebug && Settings.CANID == 0)){ Can_Debug(canMsg); }
    if(canMsg.can_id == Settings.CANID){ Can_Decode(canMsg);}
  }
}

void Config(){
  String menu_option_string;
  char menu_option_char;
  byte menu_option = 0;
  int32_t menu_option_val_Int = 0;
  float menu_option_val_Float = 0.0;
  String menu_option_val_String = "0";

  if (Serial.available()){
    menu_option_string = Serial.readString(); 
    menu_option_char = menu_option_string.charAt(0);
    menu_option = menu_option_string.toInt();
    if (menu_option == 0){ menu_option = menu_option_char; }
    for (uint16_t i = 0; i < menu_option_string.length(); i++) {
      if (menu_option_string.substring(i, i+1) == "=") { 
        menu_option_val_Int = menu_option_string.substring(i+1).toInt();
        menu_option_val_Float = menu_option_string.substring(i+1).toFloat();
        menu_option_val_String = menu_option_string.substring(i+1);
        break;
      }
    }
  } else { menu_option = 0; }

  switch (menu_option){
    //case 1: Settings.CANID = menu_option_val_Int; Config();break;
    case 1: Settings.CANID = strtol(menu_option_val_String.c_str(),NULL,16); Config();break;
    case 2: Settings.Position = menu_option_val_Int; Config();break;
    case 3: Settings.Bits = menu_option_val_Int; Config();break;
    case 4: Settings.Gain = menu_option_val_Float; Config();break;
    case 5: Settings.ValMin = menu_option_val_Float; Config();break;
    case 6: Settings.ValMax = menu_option_val_Float; Config();break;
    case 7: Stepper_Drive(constrain(menu_option_val_Int,0,100)); Config();break;
    case 8: CanDebug = !CanDebug; Config(); break;
    case 9: ValDebug = !ValDebug; Config(); break;
    case 10: EEPROM.put(0, Settings); Config(); Serial.println(":::::Saved:::::");break;
    default:
      Serial_clear();
      Serial.print("Firmware: "); Serial.println(Firmware);
      Serial.println("----------------------------------");
      Serial.print("[1] CAN ID (HEX): ");Serial.println(Settings.CANID, HEX);
      Serial.print("[2] Position:     ");Serial.println(Settings.Position);
      Serial.print("[3] Bits:         ");Serial.println(Settings.Bits);
      Serial.print("[4] Gain:         ");Serial.println(Settings.Gain);
      Serial.print("[5] Minimum:      ");Serial.println(Settings.ValMin);
      Serial.print("[6] Maximum:      ");Serial.println(Settings.ValMax);
      Serial.print("[7] Gauge Test (%)");Serial.println();
      Serial.print("[8] CAN Test:     "+String(CanDebug?"ON":"OFF"));Serial.println();
      Serial.print("[9] Value Test:   "+String(ValDebug?"ON":"OFF"));Serial.println();
      Serial.print("[10] Safe Settings!");Serial.println();
      break;
  }
}

void Serial_clear(){
  // Form feed
  Serial.write(12);
  
  // ESC + Clear
  //Serial.write(27);
  //Serial.print("[2J");
  
  // bunch of new lines
  //for (byte i = 0; i < 40; i++){Serial.println();}
}

// pass a value between 0 & 100 (%).
void Stepper_Drive(float _val){
  int rel_pos = 0;    

  _val = _val * max_Steps / 100;
  _val = constrain(_val,0,max_Steps);
  rel_pos = _val-abs_pos;
  abs_pos = _val;
  stepper.step(rel_pos);
}

void Can_Debug(can_frame canMsg){
  Serial.print("ID: ");
  Serial.print(canMsg.can_id, HEX); // print ID
  Serial.print(" len: ["); 
  Serial.print(canMsg.can_dlc, HEX); // print DLC
  Serial.print("] ");
  
  for (int i = 0; i < canMsg.can_dlc; i++)  {  // print the data
    Serial.print(canMsg.data[i],HEX);
    Serial.print(" ");
  }
  Serial.println(); 
}

void Can_Decode(can_frame canMsg){
  uint64_t tmp_val = 0;
  int32_t val = 0;
  for (byte i = 0; i < canMsg.can_dlc; i++){
    tmp_val |= uint64_t(canMsg.data[i]) << (8*i);
  }
  
  if(Settings.Bits > 0){
    // Little Endian
    uint64_t mask = (1ULL << Settings.Bits) - 1;
    val = tmp_val >> (Settings.Position) & mask;
  } else {
    // Big Endian 
    for (int i = (Settings.Position + Settings.Bits); i <= Settings.Position; ++i) {
      val = (val << 1) | ((tmp_val >> i) & 1);
    }
  }
  val = val * Settings.Gain;
  val = constrain(val, Settings.ValMin, Settings.ValMax);

  if(ValDebug){
    Serial.println(val);
  }

  val = map(val, Settings.ValMin, Settings.ValMax, 0, 100);

  Stepper_Drive(val);
}
