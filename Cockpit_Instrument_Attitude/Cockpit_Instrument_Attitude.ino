#include <SPI.h>

#include "back.h"
#include "cover.h"
#include "pitch.h"
#include "bank.h"

#include "rotation.h"
#include "alphaMixer.h"
#include <hardware/spi.h>
#include "string"

#define LCD_DC_PIN 8
#define LCD_CS_PIN 9
#define LCD_CLK_PIN 10
#define LCD_MISO_PIN 12
#define LCD_MOSI_PIN 11
#define LCD_RST_PIN 12
#define LCD_BL_PIN 25

#define CHUNK 60


void LCD_Send_Command(byte command);
void LCD_Send_Data(byte data);
byte LCD_Get_Data();
void setDataWindow(unsigned char img[], byte x_1, byte y_1, byte x_2, byte y_2);
void LCD_init();
unsigned int pxl(unsigned char r,unsigned char g,unsigned char b);
unsigned int mixAlpha(unsigned int pxl1, unsigned int pxl2, unsigned char alpha);
//void drawFullHeadingIndicator(int deg);
void drawAttitudeIndicatorPart1(int pitchDeg);
void drawAttitudeIndicatorPart2(int bankDeg);
void setDataPoint(unsigned int pixel,byte x,byte y);
//double mycos(int deg);
//double mysin(int deg);
//void fillSinValueArray();
int rotationPartSin(int deg, int pixel);
unsigned int alphaScale(unsigned char value, unsigned char alpha);
int readNumber(int place);


char isDisplayWriteMode;
byte rgbIter=0;
int pxlCounter=0;
int received=1;
unsigned char rgbByte[5];
bool image1Selected=true;
unsigned char currentImage[240*240*2];
unsigned char currentImage2[240*240*2];
int cnt=0;


bool send = false;

int pitch = 0;
int bank = 0;


void setup() {
  Serial.begin(9600);
  //delay(10);
  Serial.println("Initializing pins");
  drawAttitudeIndicatorPart1(pitch);
  drawAttitudeIndicatorPart2(bank);
      
}

void setup1() {
  //Initialize Pins for Display
  pinMode(LCD_RST_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_BL_PIN, OUTPUT);

  analogWrite(LCD_BL_PIN, 0); //PWM for the LCD BL pin

  //Set up SPI for LCD communication
  SPI1.setRX(LCD_MISO_PIN);
  SPI1.setCS(LCD_CS_PIN);
  SPI1.setSCK(LCD_CLK_PIN);
  SPI1.setTX(LCD_MOSI_PIN);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

  delay(100);

  //Send LCD Reset
  digitalWrite(LCD_RST_PIN, 1);
  delay(100);
  digitalWrite(LCD_RST_PIN, 0);
  delay(100);
  digitalWrite(LCD_RST_PIN, 1);
  digitalWrite(LCD_CS_PIN, 0);
  delay(100);

  analogWrite(LCD_BL_PIN, 128); //PWM for the LCD BL pin
  //Normal mode on
  LCD_Send_Command(0x13);
  //Sleep mode off
  LCD_Send_Command(0x11);

  //Set to Horizontal Mode and RGB mode
  LCD_Send_Command(0x36); //Memory access control reg
  LCD_Send_Data(0Xc4);  //Set Horizontal and RGB

  //COLMOD Pixel Format
  //LCD_Send_Command(0x3A);
  //LCD_Send_Data(0X55);

  //Display On
  LCD_Send_Command(0x29);

  //Set frame rate
  
  //LCD_Send_Command(0xE8); //frame rate command
  //LCD_Send_Data(0X30);  //set to 0



  LCD_init();

  drawAttitudeIndicatorPart1(pitch);
  drawAttitudeIndicatorPart2(bank);
  setDataWindow((unsigned char *)&currentImage2[0],0,0,239,239);
}

void loop() {
/*
  for(kollsmanSet=28.00;kollsmanSet<32.00;kollsmanSet+=0.1) {
    altitude+=10;
    Serial.println("start");
    int starting = millis();
    drawPart2AltimeterIndicator(altitude,kollsmanSet);
    setDataWindow((unsigned char *)&currentImage[0],0,0,239,239);
    
    Serial.print("Calc: ");
    Serial.println(millis()-starting);
    starting=millis();
    
  }
*/
  while( Serial.available() ) {
    char nextChar = Serial.read();
    if(nextChar=='p') {
      pitch = readNumber(3);
      while(Serial.available()) {
        if( Serial.read() == 'b' ) {
          break;
        }
      }
      bank = readNumber(3);
      int starting = millis();

      drawAttitudeIndicatorPart1(pitch);
      Serial.print("Calc: ");
      Serial.println(millis()-starting);

      while(Serial.available()) {
        Serial.read();
      }
    }
  }
}

void loop1() {
  drawAttitudeIndicatorPart2(bank);
  setDataWindow((unsigned char *)&currentImage2[0],0,0,239,239);
}

int readNumber(int place) {
  while(Serial.available()) {
    char c = Serial.read();
    if(c=='-') {
      return -readNumber(place);
    } else {
      if(place==1) {
        return (c-48);
      } else {
        return (int)(c-48)*pow(10,place-1)+readNumber(place-1);
      }
    }
  }
  return 0;
}

int rotationPartSin(int deg, int pixel) {
  return rotationLookup[deg*240+pixel];
}

//Draw part1 attitude indicator
void drawAttitudeIndicatorPart1(int pitchDeg) {    
    if(pitchDeg<-25) {
      pitchDeg=-25;
    } else if(pitchDeg>25) {
      pitchDeg=25;
    }
    int pitchTravel = (int)((pitchDeg)*2.4);


    unsigned int pixelBackground;
    unsigned char alphaBackground;
    unsigned int carryBackground=1;
    int iBackground=0;

    unsigned int pixelPitch;
    unsigned char alphaPitch;
    unsigned int carryPitch=1;
    int iPitch=0;
    if(pitchTravel>0) {
      for(int j=0;j<240*pitchTravel;j++) {
        if(--carryPitch<=0) {
          pixelPitch = pitchImg[iPitch++];
          alphaPitch = pitchImg[iPitch++];
          carryPitch = pitchImg[iPitch++];
        }
      }
    } else if(pitchTravel<0) {
      iPitch=pitchTravel*240*3;
    }

    unsigned int pixelBank;
    unsigned char alphaBank;
    unsigned int carryBank=1;
    int iBank=0;

    int i=0;

    for(int x=0;x<240;x++) {
      for(int y=239;y>=0;y--) {
        
        if(--carryBackground<=0) {
          pixelBackground = backgroundImg[iBackground++];
          alphaBackground = backgroundImg[iBackground++];
          carryBackground = backgroundImg[iBackground++];
        }

        //pitchTravel*
        if(iPitch>=0) {
          if(--carryPitch<=0) {
            pixelPitch = pitchImg[iPitch++];
            alphaPitch = pitchImg[iPitch++];
            carryPitch = pitchImg[iPitch++];
          }
        } else if(iPitch>=8803) {
          pixelPitch = 0x0000;
          alphaPitch = 0x00;          
        } else {
          iPitch+=3;
          pixelPitch = 0x0000;
          alphaPitch = 0x00;
        }
        unsigned int mixedValue1 = mixAlpha(pixelBackground, pixelPitch, alphaPitch);

        if(--carryBank<=0) {
          pixelBank = bankImg[iBank++];
          alphaBank = bankImg[iBank++];
          carryBank = bankImg[iBank++];
        }
        unsigned int mixedValue2 = mixAlpha(mixedValue1, pixelBank, alphaBank);

        currentImage[i+1] = mixedValue2&0xff;
        currentImage[i] = mixedValue2>>8;
        i+=2;
      }
    }
}

//Draw part2 attitude indicator
void drawAttitudeIndicatorPart2(int bankDeg) {    
		bankDeg = ((bankDeg+90)+360)%360;
		int bankDegCos = (bankDeg+90)%360;    
    
    unsigned int pixelCover;
    unsigned char alphaCover;
    unsigned int carryCover=1;
    int iCover=0;


    int i=0;
    for(int x=0;x<240;x++) {
      for(int y=239;y>=0;y--) {
        unsigned int pixelBank;
        if(x>67) {
          int rotYBank=((int)(rotationPartSin(bankDegCos,x) - rotationPartSin(bankDeg,y))>>8)+120;
          int rotXBank=((int)(rotationPartSin(bankDeg,x) + rotationPartSin(bankDegCos,y))>>8)+120;
          int iRotateBank=rotYBank*2+rotXBank*480;
          pixelBank = currentImage[iRotateBank+1]|currentImage[iRotateBank]<<8;
        } else {
          pixelBank = 0x0000;
        }

        if(--carryCover<=0) {
          pixelCover = coverImg[iCover++];
          alphaCover = coverImg[iCover++];
          carryCover = coverImg[iCover++];
        }

        unsigned int mixedValue3 = mixAlpha(pixelBank, pixelCover, alphaCover);

        currentImage2[i+1] = mixedValue3&0xff;
        currentImage2[i] = mixedValue3>>8;

        i+=2;
      }
    }

}

unsigned int alphaScale(unsigned char value, unsigned char alpha) {
  return alphaMixerLookup[value*256+alpha];
}

//Mix two pixel values together with alpha value for transparancy effect
unsigned int mixAlpha(unsigned int pxl1, unsigned int pxl2, unsigned char alpha) {
  if(alpha==0xff) {
    return pxl2;
  } else if(alpha==0x00) {
    return pxl1;
  }
  unsigned int pxl1_red = (pxl1>>11);
  unsigned int pxl2_red = (pxl2>>11);
  
  unsigned char pxl1_green = (pxl1>>6)&0x001f;
  unsigned char pxl2_green = (pxl2>>6)&0x001f;

  unsigned char pxl1_blue = pxl1&0x001f;
  unsigned char pxl2_blue = pxl2&0x001f;
  unsigned char antiAlpha = 255-alpha;

  unsigned char pxl_red_combo = (unsigned char)((alphaScale(pxl1_red,antiAlpha) + alphaScale(pxl2_red,alpha))>>12);
  //if(pxl_red_combo > 0x1f) {
    //pxl_red_combo = 0x1f;
  //}
  unsigned char pxl_green_combo = (unsigned char)((alphaScale(pxl1_green,antiAlpha) + alphaScale(pxl2_green,alpha))>>12);
  //if(pxl_green_combo > 0x3f) {
    //pxl_green_combo = 0x3f;
  //}
  unsigned char pxl_blue_combo = (unsigned char)((alphaScale(pxl1_blue,antiAlpha) + alphaScale(pxl2_blue,alpha))>>12);
  //if(pxl_blue_combo > 0x1f) {
    //pxl_blue_combo = 0x1f;
  //}
  return pxl_red_combo<<11|pxl_green_combo<<6|pxl_blue_combo;
}

unsigned int pxl(unsigned char r,unsigned char g,unsigned char b) {
  return ((r>>3)<<11)|((g>>2)<<5)|(b>>3);
}

void setDataPoint(unsigned int pixel,byte x,byte y) {
    //set the X coordinates
    LCD_Send_Command(0x2A);
    LCD_Send_Data(0x00);
    LCD_Send_Data(x);
	  LCD_Send_Data(0x00);
    LCD_Send_Data(x);

    //set the Y coordinates
    LCD_Send_Command(0x2B);
    LCD_Send_Data(0x00);
  	LCD_Send_Data(y);
  	LCD_Send_Data(0x00);
    LCD_Send_Data(y);

    LCD_Send_Command(0x2C);
    digitalWrite(LCD_DC_PIN, 1);
    LCD_Send_Data((pixel>>8)&0xff);
    LCD_Send_Data((pixel)&0xff);

}

void setDataWindow(unsigned char img[], byte x_1, byte y_1, byte x_2, byte y_2) {
    //set the X coordinates
    LCD_Send_Command(0x2A);
    LCD_Send_Data(0x00);
    LCD_Send_Data(x_1);
	  LCD_Send_Data(0x00);
    LCD_Send_Data(x_2);

    //set the Y coordinates
    LCD_Send_Command(0x2B);
    LCD_Send_Data(0x00);
  	LCD_Send_Data(y_1);
  	LCD_Send_Data(0x00);
    LCD_Send_Data(y_2);

    //Sleep mode on
    //LCD_Send_Command(0x10);

    //Idle mode on
    //LCD_Send_Command(0x39);

    //Partial mode on
    //LCD_Send_Command(0x12);

    int imageDataLength = 240*240*2;//;(x_2-x_1)*(y_2-y_1)*2;
    LCD_Send_Command(0x2C);
    digitalWrite(LCD_DC_PIN, 1);
    SPI1.transfer(&img[0], imageDataLength);

    //Sleep mode off
    //LCD_Send_Command(0x11);

    //Idle mode off
    //LCD_Send_Command(0x38);

    //Normal mode
    //LCD_Send_Command(0x13);
}

void LCD_Send_Command(uint8_t command) {
  digitalWrite(LCD_DC_PIN, 1);
  //delay(1);
  digitalWrite(LCD_DC_PIN, 0);
  SPI1.transfer(command);  //Send command
}
void LCD_Send_Data(uint8_t data) {
  digitalWrite(LCD_DC_PIN, 1);
  SPI1.transfer(data);  //Send data
}
byte LCD_Read_Data(uint8_t command) {
  digitalWrite(LCD_DC_PIN, 1);
  return SPI1.transfer(command);  //read data
}

void LCD_init() {
  LCD_Send_Command(0xEF);
	LCD_Send_Command(0xEB);
	LCD_Send_Data(0x14); 
	
  LCD_Send_Command(0xFE);			 
	LCD_Send_Command(0xEF); 

	LCD_Send_Command(0xEB);	
	LCD_Send_Data(0x14); 

	LCD_Send_Command(0x84);			
	LCD_Send_Data(0x40); 

	LCD_Send_Command(0x85);			
	LCD_Send_Data(0xFF); 

	LCD_Send_Command(0x86);			
	LCD_Send_Data(0xFF); 

	LCD_Send_Command(0x87);			
	LCD_Send_Data(0xFF);

	LCD_Send_Command(0x88);			
	LCD_Send_Data(0x0A);

	LCD_Send_Command(0x89);			
	LCD_Send_Data(0x21); 

	LCD_Send_Command(0x8A);			
	LCD_Send_Data(0x00); 

	LCD_Send_Command(0x8B);			
	LCD_Send_Data(0x80); 

	LCD_Send_Command(0x8C);			
	LCD_Send_Data(0x01); 

	LCD_Send_Command(0x8D);			
	LCD_Send_Data(0x01); 

	LCD_Send_Command(0x8E);			
	LCD_Send_Data(0xFF); 

	LCD_Send_Command(0x8F);			
	LCD_Send_Data(0xFF); 


	LCD_Send_Command(0xB6);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x20);

	LCD_Send_Command(0x36);
	LCD_Send_Data(0x08);//Set as vertical screen

	LCD_Send_Command(0x3A);			
	LCD_Send_Data(0x05); 


	LCD_Send_Command(0x90);			
	LCD_Send_Data(0x08);
	LCD_Send_Data(0x08);
	LCD_Send_Data(0x08);
	LCD_Send_Data(0x08); 

	LCD_Send_Command(0xBD);			
	LCD_Send_Data(0x06);
	
	LCD_Send_Command(0xBC);			
	LCD_Send_Data(0x00);	

	LCD_Send_Command(0xFF);			
	LCD_Send_Data(0x60);
	LCD_Send_Data(0x01);
	LCD_Send_Data(0x04);

	LCD_Send_Command(0xC3);			
	LCD_Send_Data(0x13);
	LCD_Send_Command(0xC4);			
	LCD_Send_Data(0x13);

	LCD_Send_Command(0xC9);			
	LCD_Send_Data(0x22);

	LCD_Send_Command(0xBE);			
	LCD_Send_Data(0x11); 

	LCD_Send_Command(0xE1);			
	LCD_Send_Data(0x10);
	LCD_Send_Data(0x0E);

	LCD_Send_Command(0xDF);			
	LCD_Send_Data(0x21);
	LCD_Send_Data(0x0c);
	LCD_Send_Data(0x02);

	LCD_Send_Command(0xF0);   
	LCD_Send_Data(0x45);
	LCD_Send_Data(0x09);
	LCD_Send_Data(0x08);
	LCD_Send_Data(0x08);
	LCD_Send_Data(0x26);
 	LCD_Send_Data(0x2A);

 	LCD_Send_Command(0xF1);    
 	LCD_Send_Data(0x43);
 	LCD_Send_Data(0x70);
 	LCD_Send_Data(0x72);
 	LCD_Send_Data(0x36);
 	LCD_Send_Data(0x37);  
 	LCD_Send_Data(0x6F);


 	LCD_Send_Command(0xF2);   
 	LCD_Send_Data(0x45);
 	LCD_Send_Data(0x09);
 	LCD_Send_Data(0x08);
 	LCD_Send_Data(0x08);
 	LCD_Send_Data(0x26);
 	LCD_Send_Data(0x2A);

 	LCD_Send_Command(0xF3);   
 	LCD_Send_Data(0x43);
 	LCD_Send_Data(0x70);
 	LCD_Send_Data(0x72);
 	LCD_Send_Data(0x36);
 	LCD_Send_Data(0x37); 
 	LCD_Send_Data(0x6F);

	LCD_Send_Command(0xED);	
	LCD_Send_Data(0x1B); 
	LCD_Send_Data(0x0B); 

	LCD_Send_Command(0xAE);			
	LCD_Send_Data(0x77);
	
	LCD_Send_Command(0xCD);			
	LCD_Send_Data(0x63);		


	LCD_Send_Command(0x70);			
	LCD_Send_Data(0x07);
	LCD_Send_Data(0x07);
	LCD_Send_Data(0x04);
	LCD_Send_Data(0x0E); 
	LCD_Send_Data(0x0F); 
	LCD_Send_Data(0x09);
	LCD_Send_Data(0x07);
	LCD_Send_Data(0x08);
	LCD_Send_Data(0x03);

	LCD_Send_Command(0xE8);			
	LCD_Send_Data(0x34);

	LCD_Send_Command(0x62);			
	LCD_Send_Data(0x18);
	LCD_Send_Data(0x0D);
	LCD_Send_Data(0x71);
	LCD_Send_Data(0xED);
	LCD_Send_Data(0x70); 
	LCD_Send_Data(0x70);
	LCD_Send_Data(0x18);
	LCD_Send_Data(0x0F);
	LCD_Send_Data(0x71);
	LCD_Send_Data(0xEF);
	LCD_Send_Data(0x70); 
	LCD_Send_Data(0x70);

	LCD_Send_Command(0x63);			
	LCD_Send_Data(0x18);
	LCD_Send_Data(0x11);
	LCD_Send_Data(0x71);
	LCD_Send_Data(0xF1);
	LCD_Send_Data(0x70); 
	LCD_Send_Data(0x70);
	LCD_Send_Data(0x18);
	LCD_Send_Data(0x13);
	LCD_Send_Data(0x71);
	LCD_Send_Data(0xF3);
	LCD_Send_Data(0x70); 
	LCD_Send_Data(0x70);

	LCD_Send_Command(0x64);			
	LCD_Send_Data(0x28);
	LCD_Send_Data(0x29);
	LCD_Send_Data(0xF1);
	LCD_Send_Data(0x01);
	LCD_Send_Data(0xF1);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x07);

	LCD_Send_Command(0x66);			
	LCD_Send_Data(0x3C);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0xCD);
	LCD_Send_Data(0x67);
	LCD_Send_Data(0x45);
	LCD_Send_Data(0x45);
	LCD_Send_Data(0x10);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x00);

	LCD_Send_Command(0x67);			
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x3C);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x00);
	LCD_Send_Data(0x01);
	LCD_Send_Data(0x54);
	LCD_Send_Data(0x10);
	LCD_Send_Data(0x32);
	LCD_Send_Data(0x98);

	LCD_Send_Command(0x74);			
	LCD_Send_Data(0x10);	
	LCD_Send_Data(0x85);	
	LCD_Send_Data(0x80);
	LCD_Send_Data(0x00); 
	LCD_Send_Data(0x00); 
	LCD_Send_Data(0x4E);
	LCD_Send_Data(0x00);					
	
  LCD_Send_Command(0x98);			
	LCD_Send_Data(0x3e);
	LCD_Send_Data(0x07);

	LCD_Send_Command(0x35);	
	LCD_Send_Command(0x21);

	LCD_Send_Command(0x11);
	delay(120);
	LCD_Send_Command(0x29);
	delay(20);
  return;
}

