#include <SPI.h>

#include "back.h"
#include "kollsman.h"
#include "ft100.h"
#include "ft1000.h"
#include "ft10000.h"

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
void drawPart1AltimeterIndicator(int altitude, double kollsman);
void drawPart2AltimeterIndicator(int altitude, double kollsman);
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
double kollsmanSet = 28.00;
double lastKollsmanSet = 28.10;
int altitude = -10000;
int lastAltitude = -1000000;



void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("Initializing pins");
  //Initialize Pins for Display
  pinMode(LCD_RST_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_BL_PIN, OUTPUT);

  Serial.println("Setting PWM to 0");
  analogWrite(LCD_BL_PIN, 0); //PWM for the LCD BL pin

  Serial.println("Setting SPI communications");
  //Set up SPI for LCD communication
  SPI1.setRX(LCD_MISO_PIN);
  SPI1.setCS(LCD_CS_PIN);
  SPI1.setSCK(LCD_CLK_PIN);
  SPI1.setTX(LCD_MOSI_PIN);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

  delay(100);

  Serial.println("Reseting LCD");
  //Send LCD Reset
  digitalWrite(LCD_RST_PIN, 1);
  delay(100);
  digitalWrite(LCD_RST_PIN, 0);
  delay(100);
  digitalWrite(LCD_RST_PIN, 1);
  digitalWrite(LCD_CS_PIN, 0);
  delay(100);

  Serial.println("Setting PWM to 100%");
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
  drawPart1AltimeterIndicator(altitude,kollsmanSet);
  drawPart2AltimeterIndicator(altitude,kollsmanSet);
    
}

void setup1() {

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
    if(nextChar=='k') {
      kollsmanSet = readNumber(4)/100.0;
      while(Serial.available()) {
        if( Serial.read() == 'a' ) {
          break;
        }
      }
      altitude = readNumber(6);
      drawPart2AltimeterIndicator(altitude,kollsmanSet);
      setDataWindow((unsigned char *)&currentImage[0],0,0,239,239);
      while(Serial.available()) {
        Serial.read();
      }
    }
  }
}

void loop1() {
  if(lastKollsmanSet!=kollsmanSet) {
    drawPart1AltimeterIndicator(altitude,kollsmanSet);
    lastKollsmanSet=kollsmanSet;
  }
  
  if(lastAltitude>altitude+50 || lastAltitude<altitude-50) {
    drawPart1AltimeterIndicator(altitude,kollsmanSet);
    lastAltitude=altitude;
  }

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

//Draw full heading indicator
void drawPart1AltimeterIndicator(int altitude, double kollsman) {
    int degKollsman=-((int)((kollsman-28)*-70))%360;
    int degForCosKollsman=(degKollsman+90)%360;
    
    int xxxxxftDeg = (360-(((int)(altitude*0.0036))%360))%360;
    int xxxxxftDegCos = (xxxxxftDeg+90)%360;

		int xxxxftDeg = (360-(((int)(altitude*0.036))%360))%360;
		int xxxxftDegCos = (xxxxftDeg+90)%360;
		
    int xxxftDeg = (360-(((int)(altitude*0.36))%360))%360;
    int xxxftDegCos = (xxxftDeg+90)%360;
    
    int deg = 0;
    int degForCos=(deg+90)%360;
    
    

    //Serial.print("ANGLE");
    //Serial.print("\t");
    //Serial.println(degKollsman);
    int centerX=120;
    int centerY=120;
    unsigned int pixelBackground;
    unsigned char alphaBackground;
    unsigned int pixel2;
    unsigned int alpha2;
    unsigned int carry=1;


    unsigned int backgroundColor = 0x0000;
    int i=0;
    int iFixed=0;

    for(int x=0;x<240;x++) {
      for(int y=239;y>=0;y--) {

        unsigned int pixelKollsman;
        if(y>225 || y>230 || x>145 || x<92) {
          pixelKollsman=0x0000;
        } else {
          int rotYKollsman=((int)(rotationPartSin(degForCosKollsman,x) - rotationPartSin(degKollsman,y))>>8)+120;//((rotationPartSin(degForCos,x) - rotationPartSin(deg,y))>>8)+120;
          int rotXKollsman=((int)(rotationPartSin(degKollsman,x) + rotationPartSin(degForCosKollsman,y))>>8)+120;//((rotationPartSin(deg,x) + rotationPartSin(degForCos,y))>>8)+120;
          int iRotateKollsman=rotYKollsman*4+rotXKollsman*960;
          pixelKollsman = pxl(kollsmanImg[iRotateKollsman],kollsmanImg[iRotateKollsman+1],kollsmanImg[iRotateKollsman+2]);
          //alphaKollsman = kollsmanImg[iRotateKollsman+3];
        }
        
        if(--carry<=0) {
          pixelBackground = backgroundImg[iFixed++];
          alphaBackground = backgroundImg[iFixed++];
          carry = backgroundImg[iFixed++];
        }

        unsigned int mixedValue1 = mixAlpha(pixelKollsman, pixelBackground, alphaBackground);

        //add 10000ft hand
        unsigned int pixel10000ft;
        unsigned char alpha10000ft;
        int rotY10000ft=((int)(rotationPartSin(xxxxxftDegCos,x) - rotationPartSin(xxxxxftDeg,y))>>8)+120;//((rotationPartSin(degForCos,x) - rotationPartSin(deg,y))>>8)+120;
        if(rotY10000ft<40) {
          pixel10000ft= 0x0000;
          alpha10000ft=0x00;
        } else {
          int rotX10000ft=((int)(rotationPartSin(xxxxxftDeg,x) + rotationPartSin(xxxxxftDegCos,y))>>8)+120;//((rotationPartSin(deg,x) + rotationPartSin(degForCos,y))>>8)+120;
          if(rotX10000ft<40 || rotX10000ft>200) {
            pixel10000ft=0x0000;
            alpha10000ft=0x00;
          } else {
            int iRotate10000ft=rotY10000ft*4+rotX10000ft*960;
            pixel10000ft = pxl(ft10000Img[iRotate10000ft],ft10000Img[iRotate10000ft+1],ft10000Img[iRotate10000ft+2]);
            alpha10000ft = ft10000Img[iRotate10000ft+3];
          }
        }

        unsigned int mixedValue2 = mixAlpha(mixedValue1, pixel10000ft, alpha10000ft);


        //add 1000ft hand
        unsigned int pixel1000ft;
        unsigned char alpha1000ft;
        int rotY1000ft=((int)(rotationPartSin(xxxxftDegCos,x) - rotationPartSin(xxxxftDeg,y))>>8)+120;
        if(rotY1000ft<70 || rotY1000ft>190) {
          pixel1000ft= 0x0000;
          alpha1000ft=0x00;
        } else {
          int rotX1000ft=((int)(rotationPartSin(xxxxftDeg,x) + rotationPartSin(xxxxftDegCos,y))>>8)+120;
          if(rotX1000ft<100 || rotX1000ft>140) {
            pixel1000ft=0x0000;
            alpha1000ft=0x00;
          } else {
            int iRotate1000ft=rotY1000ft*4+rotX1000ft*960;
            pixel1000ft = pxl(ft1000Img[iRotate1000ft],ft1000Img[iRotate1000ft+1],ft1000Img[iRotate1000ft+2]);
            alpha1000ft = ft1000Img[iRotate1000ft+3];
          }
        }

        unsigned int mixedValue3 = mixAlpha(mixedValue2, pixel1000ft, alpha1000ft);

        currentImage2[i+1] = mixedValue3&0xff;
        currentImage2[i] = mixedValue3>>8;

        i+=2;
      }
    }
}

//Draw full heading indicator
void drawPart2AltimeterIndicator(int altitude, double kollsman) {
    int xxxftDeg = (360-(((int)(altitude*0.36))%360))%360;
    int xxxftDegCos = (xxxftDeg+90)%360;
    int deg = 0;
    int degForCos=(deg+90)%360;
    
    

    //Serial.print("ANGLE");
    //Serial.print("\t");
    //Serial.println(degKollsman);
    int centerX=120;
    int centerY=120;

    int i=0;

    for(int x=0;x<240;x++) {
      for(int y=239;y>=0;y--) {
        //add 100ft hand
        unsigned int pixel100ft;
        unsigned char alpha100ft;
        int rotY100ft=((int)(rotationPartSin(xxxftDegCos,x) - rotationPartSin(xxxftDeg,y))>>8)+120;//((rotationPartSin(degForCos,x) - rotationPartSin(deg,y))>>8)+120;
        if(rotY100ft<70) {
          pixel100ft= 0x0000;
          alpha100ft=0x00;
        } else {
          int rotX100ft=((int)(rotationPartSin(xxxftDeg,x) + rotationPartSin(xxxftDegCos,y))>>8)+120;//((rotationPartSin(deg,x) + rotationPartSin(degForCos,y))>>8)+120;
          if(rotX100ft<103 || rotX100ft>137) {
            pixel100ft=0x0000;
            alpha100ft=0x00;
          } else {
            int iRotate100ft=rotY100ft*4+rotX100ft*960;
            pixel100ft = pxl(ft100Img[iRotate100ft],ft100Img[iRotate100ft+1],ft100Img[iRotate100ft+2]);
            alpha100ft = ft100Img[iRotate100ft+3];
          }
        }
        unsigned int part1 = currentImage2[i+1] | currentImage2[i]<<8;
        unsigned int mixedValue4 = mixAlpha(part1, pixel100ft, alpha100ft);

        currentImage[i+1] = mixedValue4&0xff;
        currentImage[i] = mixedValue4>>8;

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

    int imageDataLength = 240*240*2;//(x_2-x_1)*(y_2-y_1)*2;
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

