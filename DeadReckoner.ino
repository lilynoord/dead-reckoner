#include <QMC5883LCompass.h>
#include <SPI.h>
#include <TFT.h>
#include <Elegoo_GFX.h>     // Core graphics library
#include <Elegoo_TFTLCD.h>  // Hardware-specific library
#include <math.h>

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3  // Chip Select goes to Analog 3
#define LCD_CD A2  // Command/Data goes to Analog 2
#define LCD_WR A1  // LCD Write goes to Analog 1
#define LCD_RD A0  // LCD Read goes to Analog 0

#define LCD_RESET A4  // Can alternately just connect to Arduino's reset pin

/*
How this works:
- Pressing the Waypoint button once creates a line start posiiton and saves the current heading.
- Pressing the waypoint button a second time creates a line end position. 
- If the tracker button is turned on, then a line will be drawn in 
the direction indicated when the waypoint button was first pressed
- If the tracker button is turned off, the cursor will not move. If it is on, the cursor will move at a constant rate.


*/

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

QMC5883LCompass compass;

const int waypointPin = 48;
const int trackerPin = 40;
const int rightButtonPin = 30;
const int leftButtonPin = 32;
int waypointState = 0;
int trackerState = 0;
int az = 0;
const int width = tft.width();
const int height = tft.height();
const int mid[2] = { width / 2, height / 2 };
// {{{x1,y1},{x2,y2},{az,time}}}
int waypoints[100][3][2];
int waypointIndex = 0;
long globalTimer = 0;
long startTime = 0;
long endTime = 0;

void setup() {
  Serial.begin(9600);
  compass.init();
  pinMode(waypointPin, INPUT);
  pinMode(trackerPin, INPUT);
  for (int i = 0; i < 100; i++) {
    waypoints[i][0][0] = mid[0];
    waypoints[i][0][1] = mid[1];
    waypoints[i][1][0] = mid[0];
    waypoints[i][1][1] = mid[1];
    waypoints[i][2][0] = 0;
    waypoints[i][2][1] = 0;
  }
  tft.reset();
  uint16_t identifier = tft.readID();
  identifier = 0x9341;
  tft.begin(identifier);
  tft.fillScreen(BLACK);
  tft.drawLine(0, 0, 100, 100, WHITE);
}

void compasser() {
  az = compass.getAzimuth();
}
void mockCompasser() {
  az = random(0, 360);
}

/* int convertAzimuthToPolar(int azimuth){
  int angle = azimuth;
  if (angle <= 90){
    angle = 90 - angle;
  }
  return angle;
}*/


void addStartPoint() {
  waypointIndex += 1;
  mockCompasser();
  waypoints[waypointIndex][0][0] = 0;
  waypoints[waypointIndex][0][1] = 0;
  waypoints[waypointIndex][1][0] = 0;
  waypoints[waypointIndex][1][1] = 0;
  waypoints[waypointIndex][2][0] = 0;
  waypoints[waypointIndex][2][1] = 0;
  waypoints[waypointIndex][0][0] = waypoints[waypointIndex - 1][1][0];
  waypoints[waypointIndex][0][1] = waypoints[waypointIndex - 1][1][1];
  waypoints[waypointIndex][2][0] = az;
  startTime = globalTimer;
}

void addEndPoint() {
  int wI = waypointIndex;
  int dist = int((globalTimer - startTime) / 10);

  waypoints[wI][2][1] = dist;
  double azRads = waypoints[wI][2][0] * (3.1415 / 180);
  int polarX = int(dist * sin(azRads));
  int polarY = int(dist * cos(azRads));
  waypoints[wI][1][0] = waypoints[waypointIndex][0][0] + polarX;
  waypoints[wI][1][1] = waypoints[waypointIndex][0][1] + polarY;
}

void draw() {
  tft.fillScreen(BLACK);
  for (int i = 0; i < 100; i++) {

    int x0 = waypoints[i][0][0];
    int y0 = waypoints[i][0][1];
    int x1 = waypoints[i][1][0];
    int y1 = waypoints[i][1][1];
    if (x1 != 0 && y1 != 0) {
      tft.drawLine(x0, y0, x1, y1, WHITE);
    }
  }
}
int waypointSet = 0;
long ignoreTimer = 0;
void loop() {
  Serial.println(ignoreTimer);
  mockCompasser();
  // waypointState = digitalRead(waypointPin);
  // trackerState = digitalRead(trackerPin);
  trackerState = HIGH;
  ignoreTimer++;
  if (ignoreTimer % 100 == 0) {
    waypointState = HIGH;
  } else {
    waypointState = LOW;
  }
  if (trackerState == HIGH) {
    globalTimer += 1;
  }
  if (waypointState == HIGH) {
    if (waypointSet == 0) {
      waypointSet = 1;
      addStartPoint();
    } else {
      waypointSet = 0;
      addEndPoint();
      draw();
    }
  }
  delay(1);
  if (ignoreTimer % 200 == 0) {
    draw();
  }
}
