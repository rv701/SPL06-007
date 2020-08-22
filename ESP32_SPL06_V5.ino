#include <Wire.h>

uint8_t EEPROM_CHIP_ADDRESS = 0x76;

void setup() {
  pinMode(4, OUTPUT); //LORA SD PIN
  digitalWrite(4, HIGH); //Disable Lora otherwise it hangs SPI bus

  pinMode(13, OUTPUT); //3.3V Main Switch
  digitalWrite(13, LOW); //3.3V Main Switch
  
  Wire.begin();    // begin Wire(I2C)
  Serial.begin(115200); // begin Serial

  Serial.println("\nGoertek-SPL06-007 Demo\n");

  uint8_t tmp;

  
  //Serial.println("\nDevice Reset");
  //i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0X0C, 0b1001);
  //delay(1000);
  
  i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x0D);
  Serial.print("ID: ");
  Serial.println(tmp);

  Serial.println("Setting pressure configuration register - 0X00");
  i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0X06, 0B11); // Weather Station Mode
  
  Serial.println("Setting temperature configuration register - 0x80");
  i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0X07, 0X80); // Weather Station Mode

  Serial.println("Setting measurement register - 0B0111");
  i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0X08, 0B0111); // continuous temp and pressure measurement
  //i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0X08, 0B0001); // standby pressure measurement
  
  Serial.println("Setting config register - 0X00");
  i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0X09, 0X00); // FIFO Pressure measurement   
}

void loop() {
  uint8_t tmp;
  int32_t c00,c10;
  int16_t c0,c1,c01,c11,c20,c21,c30;

 
  //Serial.println("\nDevice Reset\n");
  //i2c_eeprom_write_uint8_t(EEPROM_CHIP_ADDRESS, 0x0C, 0b1001);
  //delay(1000);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x0D);
  Serial.print("ID: ");
  Serial.println(tmp);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x06);
  Serial.print("PRS_CFG: ");
  Serial.println(tmp,BIN);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x07);
  Serial.print("TMP_CFG: ");
  Serial.println(tmp,BIN);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x08);
  Serial.print("MEAS_CFG: ");
  Serial.println(tmp,BIN);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x09);
  Serial.print("CFG_REG: ");
  Serial.println(tmp,BIN);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x0A);
  Serial.print("INT_STS: ");
  Serial.println(tmp,BIN);

  tmp = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0x0B);
  Serial.print("FIFO_STS: ");
  Serial.println(tmp,BIN);

  c0 = get_c0();
  Serial.print("c0: ");
  Serial.println(c0);

  c1 = get_c1();
  Serial.print("c1: ");
  Serial.println(c1);

  c00 = get_c00();
  Serial.print("c00: ");
  Serial.println(c00);

  c10 = get_c10();
  Serial.print("c10: ");
  Serial.println(c10);

  c01 = get_c01();
  Serial.print("c01: ");
  Serial.println(c01);

  c11 = get_c11();
  Serial.print("c11: ");
  Serial.println(c11);
  
  c20 = get_c20();
  Serial.print("c20: ");
  Serial.println(c20);
  
  c21 = get_c21();
  Serial.print("c21: ");
  Serial.println(c21);

  c30 = get_c30();
  Serial.print("c30: ");
  Serial.println(c30);

  
  int32_t traw = get_traw();
  Serial.print("traw: ");
  Serial.println(traw);





  double traw_sc = (double(traw)/get_temperature_scale_factor());
  Serial.print("traw_sc: ");
  Serial.println(traw_sc,3);
  
  double temp = (double(c0) * 0.5f) + (double(c1) * traw_sc);

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println("C");
  temp = (temp * 9/5) + 32;
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println("F");
  
  

  int32_t praw = get_praw();
  //praw = praw / 2; // oversample rate 2 compinsation
  Serial.print("praw: ");
  Serial.println(praw);


  double praw_sc = (double(praw)/get_pressure_scale_factor());
  //double praw_sc = (double(praw)/2);
  
  Serial.print("praw_sc: ");
  Serial.println(praw_sc,3);

  
  double pcomp = double(c00) + praw_sc * (double(c10) + praw_sc * (double(c20) + praw_sc * double(c30))) + traw_sc * double(c01) + traw_sc * praw_sc * ( double(c11) + praw_sc * double(c21));
  //double pcomp = c00+ praw_sc*(c10+ praw_sc*(c20+ praw_sc*c30)) + traw_sc*c01 + traw_sc*praw_sc*(c11+praw_sc*c21);
  Serial.print("pcomp: ");
  Serial.println(pcomp,2);

  //pcomp = pcomp * 4.0d;
  //Serial.print("pcomp scaled by 2.105: ");
  //Serial.print("pcomp * 4: ");
  //Serial.println(pcomp,2);

  double local_pressure = 1010.5; // Look up local sea level pressure on google
  double altitude = get_altitude(pcomp,local_pressure);
  Serial.print("altitude: ");
  Serial.print(altitude,1);
  Serial.println("m");


  /*
  // calculate
  Serial.println("\nValues from external library");
  float ftsc = (float)traw_sc;
  float fpsc = (float)praw_sc;
  float qua2 = (float)c10 + fpsc * ((float)c20 + fpsc * (float)c30);
  float qua3 = ftsc * fpsc * ((float)c11 + fpsc * (float)c21);

  float fp = (float)c00 + fpsc * qua2 + ftsc * (float)c01 + qua3;
  
  fp = fp * 2.11; // Rick's sacle factor
  Serial.print("fp: ");
  Serial.println(fp);

  

  // tropospheric properties (0-11km) for standard atmosphere
  const float T1 = 15.0f + 273.15f;       // temperature at base height in Kelvin
  const float a  = -6.5f / 1000.0f;       // temperature gradient in degrees per metre
  const float g  = 9.80665f;              // gravity constant in m/s/s
  const float R  = 287.05f;               // ideal gas constant in J/kg/K
  const float msl_pressure = 101325.0f;   // in Pa
  float pK = fp / msl_pressure;
  //float pK = 101325 / msl_pressure;


  float baro_temperature = (float)c0 * 0.5f + (float)c1 * ftsc;
  Serial.print("baro_temperature: ");
  Serial.println(baro_temperature);
  float baro_pressure = fp;
  Serial.print("baro_pressure: ");
  Serial.println(baro_pressure);


  float baro_altitude = (((powf(pK, (-(a * R) / g))) * T1) - T1) / a;

  
  Serial.print("baro_altitude: ");
  Serial.println(baro_altitude);
  */

            
  
  Serial.println("");
  delay(2000);
}

double get_altitude(double pressure, double seaLevelhPa) {
  double altitude;

  //double pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}


double get_temperature_scale_factor()
{
  
  double k;

  uint8_t tmp_Byte;
  tmp_Byte = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X07); // MSB

  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  tmp_Byte = tmp_Byte >> 4; //Focus on bits 6-4
  tmp_Byte = tmp_Byte & 0B00000111;
  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  switch (tmp_Byte) 
  {
    case 0B000:
      k = 524288.0d;
    break;

    case 0B001:
      k = 1572864.0d;
    break;

    case 0B010:
      k = 3670016.0d;
    break;

    case 0B011:
      k = 7864320.0d;
    break;

    case 0B100:
      k = 253952.0d;
    break;

    case 0B101:
      k = 516096.0d;
    break;

    case 0B110:
      k = 1040384.0d;
    break;

    case 0B111:
      k = 2088960.0d;
    break;
  }

  //Serial.print("k: ");
  //Serial.println(k);

  return k;
}

double get_pressure_scale_factor()
{
  
  double k;

  uint8_t tmp_Byte;
  tmp_Byte = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X06); // MSB

  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  //tmp_Byte = tmp_Byte >> 4; //Focus on bits 6-4 - measurement rate
  tmp_Byte = tmp_Byte & 0B00000111; // Focus on 2-0 oversampling rate 
  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  //tmp_Byte = 0B011;

  switch (tmp_Byte) // oversampling rate
  {
    case 0B000:
      k = 524288.0d;
    break;

    case 0B001:
      k = 1572864.0d;
    break;

    case 0B010:
      k = 3670016.0d;
    break;

    case 0B011:
      k = 7864320.0d;
    break;

    case 0B100:
      k = 253952.0d;
    break;

    case 0B101:
      k = 516096.0d;
    break;

    case 0B110:
      k = 1040384.0d;
    break;

    case 0B111:
      k = 2088960.0d;
    break;
  }

  Serial.print("k: ");
  Serial.println(k);

  return k;
}


int32_t get_traw()
{
  int32_t tmp;
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X03); // MSB
  //Serial.print("TMP2: ");
  //Serial.print(tmp_MSB);
  
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X04); // LSB
  //Serial.print(" - TMP1: ");
  //Serial.print(tmp_LSB);

  tmp_XLSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X05); // XLSB
  //Serial.print(" - TMP0: ");
  //Serial.println(tmp_XLSB,BIN);
  

  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;

  //Serial.print("tmp: ");
  //Serial.println(tmp,BIN);

  if(tmp & (1 << 23))
    tmp = tmp | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return tmp;
}

int32_t get_praw()
{
  int32_t tmp;
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X00); // MSB
  //Serial.print("PSR2: ");
  //Serial.print(tmp_MSB,BIN);

  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X01); // LSB
  //Serial.print(" - PSR1: ");
  //Serial.print(tmp_LSB,BIN);

  tmp_XLSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X02); // XLSB
  //Serial.print(" - PSR0: ");
  //Serial.println(tmp_XLSB,BIN);
  
  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;

  //Serial.print("tmp: ");
  //Serial.println(tmp,BIN);

  if(tmp & (1 << 23))
    tmp = tmp | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return tmp;
}

int16_t get_c0()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X10); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X11); 

  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB,BIN);
  //Serial.print("tmp_LSB: ");
  //Serial.println(tmp_LSB,BIN);

  tmp_LSB = tmp_LSB >> 4;
  //Serial.print("tmp_LSB: ");
  //Serial.println(tmp_LSB,BIN);

  tmp = (tmp_MSB << 4) | tmp_LSB;

  if(tmp & (1 << 11)) // Check for 2's complement negative number
    tmp = tmp | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
  
  return tmp;
}


int16_t get_c1()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X11); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X12); 

  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB,BIN);
  //Serial.print("tmp_LSB: ");
  //Serial.println(tmp_LSB,BIN);

  tmp_MSB = tmp_MSB & 0XF;
  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB,BIN);

  tmp = (tmp_MSB << 8) | tmp_LSB;

  if(tmp & (1 << 11)) // Check for 2's complement negative number
    tmp = tmp | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
  
  return tmp;
}

int32_t get_c00()
{
  int32_t tmp; 
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X13); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X14); 
  tmp_XLSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X15);

  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB, BIN);
  //Serial.print("tmp_LSB: ");
  //Serial.println(tmp_LSB, BIN);
  //Serial.print("tmp_XLSB: ");
  //Serial.println(tmp_XLSB, BIN);
  
  tmp_XLSB = tmp_XLSB >> 4;
  //Serial.print("tmp_XLSB: ");
  //Serial.println(tmp_XLSB, BIN);
   
  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 4) | tmp_XLSB;

  if(tmp & (1 << 19))
    tmp = tmp | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number
    
  //Serial.print("tmp: ");
  //Serial.println(tmp, BIN);
  return tmp;
}

int32_t get_c10()
{
  int32_t tmp; 
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X15); // 4 bits
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X16); // 8 bits
  tmp_XLSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X17); // 8 bits


  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB, BIN);

  tmp_MSB = tmp_MSB & 0b00001111;
  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB, BIN);
  
  //Serial.print("tmp_LSB: ");
  //Serial.println(tmp_LSB, BIN);
  //Serial.print("tmp_XLSB: ");
  //Serial.println(tmp_XLSB, BIN);


  tmp = (tmp_MSB << 4) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;

  //Serial.print("tmp_c10: ");
  //Serial.println(tmp, BIN);


  if(tmp & (1 << 19))
    tmp = tmp | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number
  //else
  //  Serial.println("c10 is positive");
  
  return tmp;
}



int16_t get_c01()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X18); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X19); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c11()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X1A); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X1B); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c20()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X1C); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X1D); 

  //Serial.print("tmp_MSB: ");
  //Serial.println(tmp_MSB, BIN);
  //Serial.print("tmp_LSB: ");
  //Serial.println(tmp_LSB, BIN);

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c21()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X1E); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X1F); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c30()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X20); 
  tmp_LSB = i2c_eeprom_read_uint8_t(EEPROM_CHIP_ADDRESS, 0X21); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
  Serial.print("tmp: ");
  Serial.println(tmp);
}

void i2c_eeprom_write_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress, uint8_t data ) 
{
    uint8_t rdata = data;
    delay(5); // Make sure to delay log enough for EEPROM I2C refresh time
    Wire.beginTransmission(deviceaddress);
    Wire.write((uint8_t)(eeaddress));
    Wire.write(rdata);
    Wire.endTransmission();
}



uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress ) 
{
    uint8_t rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write(eeaddress); 
    Wire.endTransmission(false); // false to not release the line
    
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}
