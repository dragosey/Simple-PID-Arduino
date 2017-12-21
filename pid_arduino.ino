#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int Voutput_pin = 3;     // OUTPUT PWM pin
int Vinput_pin = A1;    // Analog input pin

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

class keyBoard_input
{
  private:
    int adc_key_in = 0;
  public:
  int getKey(){
    adc_key_in = analogRead(0);
    if (adc_key_in > 1000) 
      return btnNONE;
    if (adc_key_in < 50)   
      return btnRIGHT;  
    if (adc_key_in < 250)  
      return btnUP; 
    if (adc_key_in < 450)  
      return btnDOWN; 
    if (adc_key_in < 650)  
      return btnLEFT; 
    if (adc_key_in < 850)  
      return btnSELECT;
  }
};

class Signal:public keyBoard_input
{
  protected:
    float value_ref = 2.5;
    float value_mas = 0.0;
    float value_reg = 0.0;
    float p=1.0; //proportional constant component
    float pPart=1.0;
    float iPart = 1.0;
    float error = value_ref - value_mas; //error (refreshed every 1 second)
    float i=0.80; //integrative constant component
    float sum_difference = 0.0;
  public:
    void setRef(){
      int lcd_key = getKey(); 
      switch (lcd_key)
      {
        case btnUP:
        {
          value_ref += 0.5;
          if(value_ref >= 5.0)
            value_ref = 5.0;
          break;
        }
        case btnDOWN:
        {
          value_ref -= 0.5;
          if(value_ref <= 0)
            value_ref = 0.0;
          break;
        }
      }
    }
    void getMas(){
      int analog_input_value = analogRead(Vinput_pin);
	  
      value_mas = analog_input_value * (5.0 / 1023.0);
	  //conversion of analog value (0 - 1023) to a value between 0V - 5V (the working interval of the regulator) 
    }
    
    void setReg(){
      error = value_ref - value_mas;
    
      pPart = p*error; //proportional part of PID
      if (pPart > 5.0)
        pPart = 5.0;

      sum_difference += error; //add here each error

      iPart = i*sum_difference; //integrative part of PID
      if (iPart > 5.0)
        iPart = 5.0;

      if (iPart == 5.0 && error < 0)
          sum_difference = 5.0; //if the regulator is set to 5V for a long time, when the desired value is lowered it may take a long time
	// until the PID will react. To avoid this, sum_difference is set to 5.0 when that situation is reached.

      value_reg =  pPart + iPart;
      if ( value_reg > 5.0)
        value_reg = 5.0;
      if ( value_reg < 0.0)
        value_reg = 0.0;
        
      int converted_reg_value = value_reg * (255.0 / 5.0);
      analogWrite(Voutput_pin, converted_reg_value); // the value is converted to a value between 0-255 and send to the PWM pin
      
    }   
};

class Display:public Signal
{
  public:
  void display(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("REF: MAS: REG:");
    lcd.setCursor(0,1);
    lcd.print(value_ref); 
    lcd.setCursor(5,1);
    lcd.print(value_mas);
    lcd.setCursor(10,1);
    lcd.print(value_reg*20);
    lcd.print("%");

	//represent values in Plotter too.
	Serial.print(value_reg*20.0);
	Serial.print(" ");
	Serial.print(value_mas*20.0);
	Serial.print(" ");
	Serial.println(value_ref*20.0);
  }
};

Display element; //use a instance of Display class

void setup() {
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Start PID Alg");
  lcd.setCursor(0,1);
  lcd.print("Init. OK");

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  pinMode(Voutput_pin, OUTPUT); //set PWM Digital pin as OUTPUT pin

  delay(1000);
}

void loop() {
  element.setRef();
  element.getMas();
  element.setReg();
  element.display();
  
  delay(1000);
}
