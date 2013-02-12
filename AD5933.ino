/* 
Javier L. Prieto javierl.prieto@gmail.com
 */

#include <Wire.h>
#include <math.h>

#define DEVICE 0x0D //I2C default address of AD5933
#define freqRatio 32 // (4*(2^27)/16.776e6)

#define ADGain 3240000 //Gain was obtained with calibration and will result in kOhm values
#define ADSystemPhase -1.37

/* AD5933 REGISTRY MAP */
#define rControlMSB byte(0x80)
#define rControlLSB byte(0x81)

#define rStartFreqMMSB byte(0x82)
#define rStartFreqMSB byte(0x83)
#define rStartFreqLSB byte(0x84)

#define rFreqIncrementMMSB byte(0x85)
#define rFreqIncrementMSB byte(0x86)
#define rFreqIncrementLSB byte(0x87)

#define rNumberIncrementsMSB byte(0x88)
#define rNumberIncrementsLSB byte(0x89)

#define rNumberSettlingCyclesMSB byte(0x8A)
#define rNumberSettlingCyclesLSB byte(0x8B)

#define rStatus byte(0x8F)

#define rTemperatureMSB byte(0x92)
#define rTemperatureLSB byte(0x93)

#define rRealDataMSB byte(0x94)
#define rRealDataLSB byte(0x95)

#define rImDataMSB byte(0x96)
#define rImDataLSB byte(0x97)
/* *********************** */

void setup() { 
  Wire.begin();
  Serial.begin(9600); 
}

byte ADReadRegister(byte rAddress){
//Read a register from AD5933 
//HARDWARE DEBUG: AD5933 should return the value at register rAddress  
  
  //Set the pointer to read the register @rAddress
  Wire.beginTransmission(DEVICE); 
  Wire.write(0xB0); // Address pointer command 
  Wire.write(rAddress); // sets register pointer to rAddress
  Wire.endTransmission(); 

  //Request 1 byte 
  Wire.requestFrom(DEVICE, 1);  
  byte value=0x00;
  if(1 <= Wire.available()){
    value = Wire.read();
  }
  return value;
}

void ADWriteRegister(byte rAddress, byte value){
//Write value into the register rAddress @AD5933
//HARDWARE DEBUG: AD5933 should have stored 'value' in 'rAddress'
  Wire.beginTransmission(DEVICE); 
  Wire.write(rAddress); 
  Wire.write(value); 
  Wire.endTransmission();
}

void ADReset(){
//To reset AD5933 set to 1 bit D4 in rControlLSB and then set the same bit to 0
//HARDWARE DEBUG: AD5933 should stop the frequency sweep with a reset command
  //Maintain value of external clock flag
  byte auxReg=ADReadRegister(rControlLSB);
  auxReg &= 0x0F; // 0000 ****
  auxReg |= 0x10; // 0001 ****
  ADWriteRegister(rControlLSB,byte(auxReg)); //set reset to 1
  auxReg &= 0x0F; // 0000 ****
  ADWriteRegister(rControlLSB,byte(auxReg)); //set reset back to 0
}

void ADStandby(){
//To set AD5933 in stand by set to 1 bit D4 in rControlLSB and then set the same bit to 0
//HARDWARE DEBUG: AD5933 should put VIN and VOUT at 0V
  //keep value of voltage and gain values
  byte auxReg=ADReadRegister(rControlMSB);
  auxReg &= 0x0F; // 0000 ****
  auxReg |= 0xB0; // 1011 ****
  ADWriteRegister(rControlMSB,auxReg);   
}

void ADProgramFrequencySweep(unsigned long int startF,unsigned long int incrementF,int nIncrements){
//Program start frequency, frequency increment and number of increments
//Input values given in Hz
  startF=freqRatio*startF;  
  ADWriteRegister(rStartFreqMMSB,byte(0x000000ff & (startF>>16)));
  ADWriteRegister(rStartFreqMSB,byte(0x000000ff & (startF>>8)));
  ADWriteRegister(rStartFreqLSB,byte(0x000000ff & startF));  
  
  incrementF=freqRatio*incrementF;
  ADWriteRegister(rFreqIncrementMMSB,byte(0x000000ff & (incrementF>>16)));
  ADWriteRegister(rFreqIncrementMSB,byte(0x000000ff & (incrementF>>8)));
  ADWriteRegister(rFreqIncrementLSB,byte(0x000000ff & incrementF));
  
  if(nIncrements>=511) nIncrements=511;
  ADWriteRegister(rNumberIncrementsMSB,byte(0x000000ff & (nIncrements>>8)));
  ADWriteRegister(rNumberIncrementsLSB,byte(0x000000ff & nIncrements));
}

void ADSetGain(byte gain){
//Program control register D8 according to gain
  byte auxReg=ADReadRegister(rControlMSB);
  if(gain!=0x01) auxReg |=0x10;  // ***1 ****
  else auxReg &=0xEF;            // ***0 ****
  ADWriteRegister(rControlMSB,auxReg);  
}

void ADSetAmplitude(byte amp){
//Program amplitude in bits D9-D10 according to amp
// amp=1 -> 2Vpp
// amp=2 -> 1Vpp
// amp=3 -> 400mVpp
// amp=4 -> 200mVpp
  byte auxReg=ADReadRegister(rControlMSB);
  auxReg &= 0xF9; // **** *00*
  if(amp==0x02) auxReg |= 0x06; // **** *11*
  if(amp==0x03) auxReg |= 0x04; // **** *10*
  if(amp==0x04) auxReg |= 0x02; // **** *01*
  ADWriteRegister(rControlMSB,auxReg);
}

void ADSettlingTime(int nCycles, byte multiplier){
//Program settling time
//Multiplier can be 0,2 or 4
  if(nCycles>=511) nCycles=511;
  //build MSB considering multiplier and nIncrements
  byte auxReg =  byte(0x00000001 & (nCycles>>8)); // 000X
  if(multiplier==2) auxReg |= 0x02; // 001*
  if(multiplier==4) auxReg |= 0x06; // 011*  
  ADWriteRegister(rNumberSettlingCyclesMSB,auxReg);
  ADWriteRegister(rNumberSettlingCyclesLSB,byte(0x000000ff & nCycles));
}


void ADInit(){    
  ADReset();
  ADStandby();
  ADSetAmplitude(1);
  ADSetGain(1);
  ADProgramFrequencySweep(10000,10,100);
  ADSettlingTime(511,0);

}

void ADInitializeFrequency(){
  byte auxReg = ADReadRegister(rControlMSB);
  auxReg &= 0x0F; // 0000 ****
  auxReg |= 0x10; // 0001 ****
  ADWriteRegister(rControlMSB,auxReg);
}

void ADSweep(){
  byte auxReg = ADReadRegister(rControlMSB);
  auxReg &= 0x0F; // 0000 ****
  auxReg |= 0x20; // 0010 ****
  ADWriteRegister(rControlMSB,auxReg);
}

void ADRepeatFrequency(){
  byte auxReg = ADReadRegister(rControlMSB);
  auxReg &= 0x0F; // 0000 ****
  auxReg |= 0x40; // 0100 ****
  ADWriteRegister(rControlMSB,auxReg);
}

void ADIncrementFrequency(){
  byte auxReg = ADReadRegister(rControlMSB);
  auxReg &= 0x0F; // 0000 ****
  auxReg |= 0x30; // 0011 ****
  ADWriteRegister(rControlMSB,auxReg);
}

void ADPrintRegisters(){
  byte r;  
  for (byte reg = 0x80; reg <= 0x8B; reg++){
    r=ADReadRegister(reg);
//    Serial.print("register: "); Serial.print(reg,HEX);  Serial.print("     value: ");
    Serial.print(r);  Serial.print("\n");
  }
  r=ADReadRegister(0x8F);
//    Serial.print("register: ");   Serial.print(reg,HEX); Serial.print("     value: ");
    Serial.print(r);  Serial.print("\n");
  for (byte reg = 0x92; reg <= 0x97; reg++){
    r=ADReadRegister(reg);
//    Serial.print("register: ");  Serial.print(reg,HEX);  Serial.print("     value: ");
    Serial.print(r);  Serial.print("\n");
  }
  Serial.print("\n\n\n");
}

int real, img;
float auxFloat1,auxFloat2;
double mag, phase;
byte auxByte;


void loop() {
  ADInit();    //Hardware Debug - All registers set correctly and control is 0xA000 = power down mode
  ADStandby();  //Hardware Debug - control is 0xB000 = Standby mode
  ADInitializeFrequency();  //Hardware Debug - control is 0x1000 = Initialize with start frequency
  delay(500); //Make sure settling time has elapsed 
  ADSweep();  //Hardware Debug - control is 0x2000 = Start frequency sweep
  
//  ADPrintRegisters(); delay(6000);
  while(!(ADReadRegister(rStatus) & 0x04)){
    while(!(ADReadRegister(rStatus)&0x02)); //Wait until DFT conversion is complete      
    auxByte=ADReadRegister(rRealDataMSB);    real=auxByte<<8;
    auxByte=ADReadRegister(rRealDataLSB);    real|=auxByte;
    auxFloat1=abs(real);    auxFloat1=auxFloat1*auxFloat1; //real^2
    
    auxByte=ADReadRegister(rImDataMSB);      img=auxByte<<8;
    auxByte=ADReadRegister(rImDataLSB);      img|=auxByte;
    auxFloat2=abs(img);  auxFloat2=auxFloat2*auxFloat2; //img^2

    mag=ADGain/sqrt(auxFloat1+auxFloat2);  
    phase=1/tan(img/real);//-ADSystemPhase; //phase=180*phase/M_PI;
    
//    Serial.print("Real: "); Serial.print(real);  Serial.print("    Imaginary: ");  Serial.print(img);    Serial.print("\n");
//    Serial.print("|Z|: "); 
    real=int(mag);
    Serial.print(real);  //Serial.print("    Phase: ");   Serial.print(phase);   
    Serial.print("\n");  
    delay(50);
//    ADIncrementFrequency();       
    ADRepeatFrequency();
  }
  Serial.print("\n\n\n\n");
  delay(6000);
}

