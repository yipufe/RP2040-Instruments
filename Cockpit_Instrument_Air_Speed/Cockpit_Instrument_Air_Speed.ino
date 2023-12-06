#include <SPI.h>
#include "back.h"
#include "arrow.h"
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
void drawFullAirSpeedIndicator(int ftmin);
void setDataPoint(unsigned int pixel,byte x,byte y);
//double mycos(int deg);
//double mysin(int deg);
//void fillSinValueArray();
int rotationPartSin(int deg, int pixel);
int rotationPartCos(int deg, int pixel);
unsigned int alphaScale(unsigned char value, unsigned char alpha);
int readNumber(int place);
void drawLine(int x, int y, double deg, int len, double width, unsigned int color, unsigned char alpha);
//void drawLine(int x1, int y1, int x2, int y2, double width, unsigned int color, unsigned char alpha);
void drawPxl(int x, int y, unsigned int color, unsigned char alpha);
int gadgeSpeedToDeg(int speed);

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


  drawFullAirSpeedIndicator(0);

}
char isDisplayWriteMode;
byte rgbIter=0;
int pxlCounter=0;
int received=1;
unsigned char rgbByte[5];
unsigned char currentImage[240*240*2];
int cnt=0;

int currentValue = 0; 
bool goingUp = true;
void loop() {
  while( Serial.available() ) {
    if(Serial.read()=='s') {
      currentValue = readNumber(4);
      drawFullAirSpeedIndicator(currentValue);
      while(Serial.available()) {
        Serial.read();
      }
    }
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
int rotationPartCos(int deg, int pixel) {
  return rotationLookup[deg*240+pixel];
}

//double sinValues[360];
//const double rotationLookup[86400];
//double mycos(int deg) { return cos(deg*3.14159/180);/*mysin(deg+90);*/ }
//double mysin(int deg) { return sin(deg*3.14159/180);/*sinValues[deg%360];*/ }
/*
void fillSinValueArray() {
  for(int deg=0;deg<360;deg++) {
    double rotateRad = (((double)deg)/180)*3.14159;
    sinValues[deg] = sin(rotateRad)*256;
  }
}
*/

int gadgeStart = 40;
int gadgeStartAngle = 0;
int majorMarksInterval = 10;
int minorMarksInterval = 5;
unsigned int markColor = 0xffff;
int flapsStart=40;
int flapsStop=85;
unsigned int flapsBandColor = 0xffff;
int normalFlyingStart=48;
int normalFlyingStop=129;
unsigned int normalFlyingBandColor = 0x05E0;
int smoothFlyingStart=129;
int smoothFlyingStop=163;
unsigned int smoothFlyingBandColor = 0xffe0;
int dangerStart=163;
int dangerStop=180;
unsigned int dangerBandColor = 0xf800;
int gadgeStop = 200;
int gadgeStopAngle = 340;

int gadgeSpeedToDeg(int speed) {
  //speed=speed*(sin((speed/200)*3.14159)+1.2)/2.2;
  int speedRange = gadgeStop;
  int degRange = gadgeStopAngle;
  double degPerSpeedUnit =((double)degRange/speedRange);
  return (int)(degPerSpeedUnit*speed);
}

//Draw full heading indicator
void drawFullAirSpeedIndicator(int airSpeedBy10) {
    double airSpeedInKnots = (double)airSpeedBy10/10.0;
    int deg = 0;
    if( airSpeedInKnots < 40 ) {
      deg = (int)(airSpeedInKnots*0.625);// 25 deg / 40 knots
    } else if (airSpeedInKnots >= 40 && airSpeedInKnots < 50) {
      deg = (int)(25+(airSpeedInKnots-40)*1.2); // 12 deg / 10 knots
    } else if (airSpeedInKnots >=50 && airSpeedInKnots < 60) {
      deg = (int)(25+12+(airSpeedInKnots-50)*1.3); // 13 deg / 10 knots
    } else if (airSpeedInKnots >=60 && airSpeedInKnots < 65) {
      deg = (int)(25+12+13+(airSpeedInKnots-60)*1.7); // 8.5 deg / 5 knots
    } else if (airSpeedInKnots >=65 && airSpeedInKnots < 70) {
      deg = (int)(25+12+13+8.5+(airSpeedInKnots-65)*1.7); // 8.5 deg / 5 knots
    } else if (airSpeedInKnots >=70 && airSpeedInKnots < 75) {
      deg = (int)(25+12+13+8.5+8.5+(airSpeedInKnots-70)*1.8); // 9 deg / 5 knots
    } else if (airSpeedInKnots >=75 && airSpeedInKnots < 80) {
      deg = (int)(25+12+13+8.5+8.5+9+(airSpeedInKnots-75)*1.8); // 9 deg / 5 knots
    } else if (airSpeedInKnots >=80 && airSpeedInKnots < 85) {
      deg = (int)(25+12+13+8.5+8.5+9+9+(airSpeedInKnots-80)*1.8); // 9 deg / 5 knots
    } else if (airSpeedInKnots >=85 && airSpeedInKnots < 90) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+(airSpeedInKnots-85)*2); // 10 deg / 5 knots
    } else if (airSpeedInKnots >=90 && airSpeedInKnots < 95) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+(airSpeedInKnots-90)*2); // 10 deg / 5 knots
    } else if (airSpeedInKnots >=95 && airSpeedInKnots < 100) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+(airSpeedInKnots-95)*2.2); // 11 deg / 5 knots
    } else if (airSpeedInKnots >=100 && airSpeedInKnots < 105) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+(airSpeedInKnots-100)*2.3); // 11.5 deg / 5 knots
    } else if (airSpeedInKnots >=105 && airSpeedInKnots < 110) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+(airSpeedInKnots-105)*2.7); // 13.5 deg / 5 knots
    } else if (airSpeedInKnots >=110 && airSpeedInKnots < 115) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+(airSpeedInKnots-110)*2.8); // 14 deg / 5 knots
    } else if (airSpeedInKnots >=115 && airSpeedInKnots < 120) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+(airSpeedInKnots-115)*3.2); // 16 deg / 5 knots
    } else if (airSpeedInKnots >=120 && airSpeedInKnots < 125) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+(airSpeedInKnots-120)*3.2); // 16 deg / 5 knots
    } else if (airSpeedInKnots >=125 && airSpeedInKnots < 130) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+(airSpeedInKnots-125)*3.2); // 16 deg / 5 knots
    } else if (airSpeedInKnots >=130 && airSpeedInKnots < 135) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+(airSpeedInKnots-130)*2.8); // 14 deg / 5 knots
    } else if (airSpeedInKnots >=135 && airSpeedInKnots < 140) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+(airSpeedInKnots-135)*2.8); // 14 deg / 5 knots
    } else if (airSpeedInKnots >=140 && airSpeedInKnots < 145) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+(airSpeedInKnots-140)*2.4); // 12 deg / 5 knots
    } else if (airSpeedInKnots >=145 && airSpeedInKnots < 150) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+(airSpeedInKnots-145)*2.4); // 12 deg / 5 knots
    } else if (airSpeedInKnots >=150 && airSpeedInKnots < 155) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+(airSpeedInKnots-150)*2.2); // 11 deg / 5 knots
    } else if (airSpeedInKnots >=155 && airSpeedInKnots < 160) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+(airSpeedInKnots-155)*2); // 10 deg / 5 knots
    } else if (airSpeedInKnots >=160 && airSpeedInKnots < 165) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+(airSpeedInKnots-160)*1.6); // 8 deg / 5 knots
    } else if (airSpeedInKnots >=165 && airSpeedInKnots < 170) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+(airSpeedInKnots-165)*1.6); // 8 deg / 5 knots
    } else if (airSpeedInKnots >=170 && airSpeedInKnots < 175) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+(airSpeedInKnots-170)*1.4); // 7 deg / 5 knots
    } else if (airSpeedInKnots >=175 && airSpeedInKnots < 180) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+7+(airSpeedInKnots-175)*1.4); // 7 deg / 5 knots
    } else if (airSpeedInKnots >=180 && airSpeedInKnots < 185) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+7+7+(airSpeedInKnots-180)*1.3); // 6.5 deg / 5 knots
    } else if (airSpeedInKnots >=185 && airSpeedInKnots < 190) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+7+7+6.5+(airSpeedInKnots-185)*1.3); // 6.5 deg / 5 knots
    } else if (airSpeedInKnots >=190 && airSpeedInKnots < 195) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+7+7+6.5+6.5+(airSpeedInKnots-190)*1.2); // 6 deg / 5 knots
    } else if (airSpeedInKnots >=195 && airSpeedInKnots < 200) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+7+7+6.5+6.5+6+(airSpeedInKnots-195)*1.2); // 6 deg / 5 knots
    } else if (airSpeedInKnots >=200) {
      deg = (int)(25+12+13+8.5+8.5+9+9+9+10+10+11+11.5+13.5+14+16+16+16+14+14+12+12+11+10+8+8+7+7+6.5+6.5+6+6);
    } else {
      deg = 0;
    }
    deg=(360-deg)+270;
    deg%=360;
    int degForCos=(deg+90)%360;
    double rotateRad = deg*3.14159/180;
    int centerX=120;
    int centerY=120;
    unsigned int pixel1;
    unsigned char alpha1;
    unsigned int pixel2;
    unsigned int alpha2;
    unsigned int carry=1;


    unsigned int backgroundColor = 0x0000;
    Serial.println("start");
    int starting = millis();
    int i=0;
    int iFixed=0;
    for(int x=0;x<240;x++) {
      for(int y=239;y>=0;y--) {
        int rotY=((int)(rotationPartCos(degForCos,x) - rotationPartSin(deg,y))>>8)+120;//((rotationPartCos(degForCos,x) - rotationPartSin(deg,y))>>8)+120;
        int rotX=((int)(rotationPartSin(deg,x) + rotationPartCos(degForCos,y))>>8)+120;//((rotationPartSin(deg,x) + rotationPartCos(degForCos,y))>>8)+120;
            int iRotate=rotY*4+rotX*960;
            if(rotY<100 || rotY>140 || rotX>170 || rotX<15) {
              if(--carry<=0) {
                pixel2 = backgroundImg[iFixed++];
                iFixed++;
                //alpha2 = background[iFixed++];
                carry = backgroundImg[iFixed++];
              }

              i+=2;
              currentImage[i+1] = pixel2&0xff;
              currentImage[i] = pixel2>>8;
            } else {
              pixel1 = pxl(arrow[iRotate],arrow[iRotate+1],arrow[iRotate+2]);
              alpha1 = arrow[iRotate+3];

              i+=2;
              unsigned int mixedValue1 = pixel1;//mixAlpha(backgroundColor, pixel1, alpha1);

              if(--carry<=0) {
                pixel2 = backgroundImg[iFixed++];
                alpha2 = backgroundImg[iFixed++];
                carry = backgroundImg[iFixed++];
              }

              unsigned int mixedValue2 = mixAlpha(pixel2, mixedValue1, alpha1);
              
              currentImage[i+1] = mixedValue2&0xff;
              currentImage[i] = mixedValue2>>8;

            }
      }
    }
    Serial.println(millis()-starting);
    starting = millis();
    

    //White Band
    for(int iDegr=gadgeSpeedToDeg(flapsStart);iDegr<gadgeSpeedToDeg(flapsStop);iDegr+=1) {
      int iDeg = ((90-iDegr)+360)%360;
      int iDegCos = (int)(iDeg+90)%360;
      int x1=120;
      int y1=20;

      int rotY=((int)(rotationPartCos(iDegCos,x1) - rotationPartSin(iDeg,y1))>>8)+120;
      int rotX=((int)(rotationPartSin(iDeg,x1) + rotationPartCos(iDegCos,y1))>>8)+120;
      
      drawLine(rotX,rotY,iDeg+270,18,2,flapsBandColor,0xff);
    }
    //Green band
    for(int iDegr=gadgeSpeedToDeg(normalFlyingStart);iDegr<gadgeSpeedToDeg(normalFlyingStop);iDegr+=1) {
      int iDeg = ((90-iDegr)+360)%360;
      int iDegCos = (int)(iDeg+90)%360;
      int x1=120;
      int y1=20;

      int rotY=((int)(rotationPartCos(iDegCos,x1) - rotationPartSin(iDeg,y1))>>8)+120;
      int rotX=((int)(rotationPartSin(iDeg,x1) + rotationPartCos(iDegCos,y1))>>8)+120;
      
      drawLine(rotX,rotY,iDeg+270,10,2,normalFlyingBandColor,0xff);
    }
    //Yellow band
    for(int iDegr=gadgeSpeedToDeg(smoothFlyingStart);iDegr<gadgeSpeedToDeg(smoothFlyingStop);iDegr+=1) {
      int iDeg = ((90-iDegr)+360)%360;
      int iDegCos = (int)(iDeg+90)%360;
      int x1=120;
      int y1=20;

      int rotY=((int)(rotationPartCos(iDegCos,x1) - rotationPartSin(iDeg,y1))>>8)+120;
      int rotX=((int)(rotationPartSin(iDeg,x1) + rotationPartCos(iDegCos,y1))>>8)+120;
      
      drawLine(rotX,rotY,iDeg+270,10,2,smoothFlyingBandColor,0xff);
    }
    //Red band
    for(int iDegr=gadgeSpeedToDeg(dangerStart);iDegr<gadgeSpeedToDeg(dangerStop);iDegr+=1) {
      int iDeg = ((90-iDegr)+360)%360;
      int iDegCos = (int)(iDeg+90)%360;
      int x1=120;
      int y1=20;

      int rotY=((int)(rotationPartCos(iDegCos,x1) - rotationPartSin(iDeg,y1))>>8)+120;
      int rotX=((int)(rotationPartSin(iDeg,x1) + rotationPartCos(iDegCos,y1))>>8)+120;
      
      drawLine(rotX,rotY,iDeg+270,10,2,dangerBandColor,0xff);
    }

/*
int gadgeStart = 40;
int gadgeStartAngle = 25;
int majorMarksInterval = 10;
int minorMarksInterval = 5;
unsigned int markColor = 0xffff;
int flapsStart=40;
int flapsStop=85;
unsigned int flapsBandColor = 0xffff;
int normalFlyingStart=48;
int normalFlyingStop=129;
unsigned int normalFlyingBandColor = 0x07e0;
int smoothFlyingStart=129;
int smoothFlyingStop=163;
unsigned int smoothFlyingBandColor = 0xffe0;
int dangerStart=163;
int dangerStop=180;
unsigned int dangerBandColor = 0xf800;
int gadgeStop = 200;
int gadgeStopAngle = 335;
*/

    //Major lines
    for(int speed=gadgeStart;speed<=gadgeStop;speed+=majorMarksInterval) {
      int iDeg = ((90-gadgeSpeedToDeg(speed))+360)%360;
      int iDegCos = (iDeg+90)%360;
      int x1=120;
      int y1=27;

      int rotY=((int)(rotationPartCos(iDegCos,x1) - rotationPartSin(iDeg,y1))>>8)+120;
      int rotX=((int)(rotationPartSin(iDeg,x1) + rotationPartCos(iDegCos,y1))>>8)+120;


      drawLine(rotX,rotY,iDeg+270,27,1,markColor,0xff);
    }
    //Minor lines
    for(int speed=gadgeStart+minorMarksInterval;speed<=gadgeStop;speed+=majorMarksInterval) {
      int iDeg = ((90-gadgeSpeedToDeg(speed))+360)%360;
      int iDegCos = (iDeg+90)%360;
      int x1=120;
      int y1=20;

      int rotY=((int)(rotationPartCos(iDegCos,x1) - rotationPartSin(iDeg,y1))>>8)+120;
      int rotX=((int)(rotationPartSin(iDeg,x1) + rotationPartCos(iDegCos,y1))>>8)+120;


      drawLine(rotX,rotY,iDeg+270,20,1.6,markColor,0xff);
    }



    setDataWindow((unsigned char *)&currentImage[0],0,0,239,239);
    Serial.println(millis()-starting);
    
}

void drawLine(int x, int y, double deg, int len, double width, unsigned int color, unsigned char alpha) {
  float rad = deg*3.14159/180;
  width/=2;
  unsigned char currentAlpha;
  for(double iX=0;iX<len;iX+=0.1) {
    for(double iY=-width;iY<width;iY+=0.1) {
      double widthDiff = width-abs(iY);
      if(widthDiff>0) {
        currentAlpha = alpha * widthDiff/width;
      } else {
        currentAlpha = 0;
      }
      
      int yRot=cos(rad)*iX-sin(rad)*iY;
      int xRot=sin(rad)*iX+cos(rad)*iY;
      drawPxl(x+xRot, y+yRot, color, currentAlpha);
    }
  }
}
/*
void drawLine(int x1, int y1, int x2, int y2, double width, unsigned int color, unsigned char alpha) {
  width /=2;
  int deltaX = abs(x2-x1);
  int deltaY = abs(y2-y1);
  if(deltaX>deltaY) {
    double slope = (double)(y2-y1)/(x2-x1);
    double pSlope = (double)(x2-x1)/(y2-y1);
    if(x2>x1) {
      for(int i=0;i<x2-x1;i++) {
        for(double tang=-width;tang<=width;tang+=1) {     
          drawPxl(x1+i,y1+slope*i, color, alpha);
        }
      }
    } else {
      for(int i=0;i<x1-x2;i++) {
        for(double tang=-width;tang<=width;tang+=1) {     
          drawPxl(x2+i,y2+slope*i, color, alpha);
        }
      }
    }
  } else {
    double slope = (double)(x2-x1)/(y2-y1);
    if(y2>y1) {
      for(int i=0;i<y2-y1;i++) {
        drawPxl(x1+slope*i,y1+i, color, alpha);
      }
    } else {
      for(int i=0;i<y1-y2;i++) {
        drawPxl(x2+slope*i,y2+i, color, alpha);
      }
    }
  }

}
*/
void drawPxl(int x, int y, unsigned int color, unsigned char alpha) {
  if(x<0 || x>239 || y < 0 || y > 239)
    return;
  int i = x*2+y*480;
  unsigned int currentPixel = currentImage[i+1] | currentImage[i]<<8;
  unsigned int mixedValue = mixAlpha(currentPixel, color, alpha);
  currentImage[i+1] = mixedValue&0xff;
  currentImage[i] = mixedValue>>8;
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

