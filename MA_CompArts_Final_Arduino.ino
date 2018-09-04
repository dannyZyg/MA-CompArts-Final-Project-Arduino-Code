// Starter code from https://learn.adafruit.com/multi-tasking-the-arduino-part-3/overview
//multitasking the arduino with neopixels

// serial output code and conversion from Joshua Noble's "Programming Interactivity: A Designer's Guide to
// Processing, Arduino, and openFrameworks" (p. 228-229, 2012)
// available at https://books.google.co.uk/books?id=sAsHA1HM1WcC&pg=PA225&lpg=PA225&dq=serial+input+from+arduino+openframeworks&source=bl&ots=KHz3UccApe&sig=mI6JR-iB4WiLT9Q-opHa-OZiNZQ&hl=en&sa=X&ved=0ahUKEwjmvtnD_I7aAhWBWhQKHW5EAKw4ChDoAQg1MAI#v=snippet&q=ofSerial&f=false

// Adafruit VL53L0X sensor code from example
// available at https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code

// This Arduino code uses a proximity sensor to detect hand pressence and create lighting effects.
// The sensor value is sent to openFrameworks for further processing.

#include <Adafruit_NeoPixel.h>
#include "Adafruit_VL53L0X.h"

#define BRIGHTNESS 255

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int val;
bool trigger = false;

// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
  public:

    // Member Variables:
    pattern  ActivePattern;  // which pattern is running
    direction Direction;     // direction to run the pattern

    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position

    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    uint16_t Index;  // current step within the pattern

    void (*OnComplete)();  // Callback on completion of pattern

    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
      : Adafruit_NeoPixel(pixels, pin, type)
    {
      OnComplete = callback;
    }

    // Update the pattern
    void Update()
    {
      if ((millis() - lastUpdate) > Interval) // time to update
      {
        lastUpdate = millis();
        switch (ActivePattern)
        {
          case RAINBOW_CYCLE:
            RainbowCycleUpdate();
            break;
          case THEATER_CHASE:
            TheaterChaseUpdate();
            break;
          case COLOR_WIPE:
            ColorWipeUpdate();
            break;
          case SCANNER:
            ScannerUpdate();
            break;
          case FADE:
            FadeUpdate();
            break;
          default:
            break;
        }
      }
    }

    // Increment the Index and reset at the end
    void Increment()
    {
      if (Direction == FORWARD)
      {
        Index++;
        if (Index >= TotalSteps)
        {
          Index = 0;
          if (OnComplete != NULL)
          {
            OnComplete(); // call the comlpetion callback
          }
        }
      }
      else // Direction == REVERSE
      {
        --Index;
        if (Index <= 0)
        {
          Index = TotalSteps - 1;
          if (OnComplete != NULL)
          {
            OnComplete(); // call the comlpetion callback
          }
        }
      }
    }

    // Reverse pattern direction
    void Reverse()
    {
      if (Direction == FORWARD)
      {
        Direction = REVERSE;
        Index = TotalSteps - 1;
      }
      else
      {
        Direction = FORWARD;
        Index = 0;
      }
    }

    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = RAINBOW_CYCLE;
      Interval = interval;
      TotalSteps = 255;
      Index = 0;
      Direction = dir;
    }

    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
      for (int i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
      }
      show();
      Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = THEATER_CHASE;
      Interval = interval;
      TotalSteps = numPixels();
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = dir;
    }

    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
      for (int i = 0; i < numPixels(); i++)
      {
        if ((i + Index) % 3 == 0)
        {
          setPixelColor(i, Color1);
        }
        else
        {
          setPixelColor(i, Color2);
        }
      }
      show();
      Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = COLOR_WIPE;
      Interval = interval;
      TotalSteps = numPixels();
      Color1 = color;
      Index = 0;
      Direction = dir;
    }

    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
      setPixelColor(Index, Color1);
      show();
      Increment();
    }

    // Initialize for a SCANNNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
      ActivePattern = SCANNER;
      Interval = interval;
      TotalSteps = (numPixels() - 1) * 2;
      Color1 = color1;
      Index = 0;
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    {
      for (int i = 0; i < numPixels(); i++)
      {
        if (i == Index)  // Scan Pixel to the right
        {
          setPixelColor(i, Color1);
        }
        else if (i == TotalSteps - Index) // Scan Pixel to the left
        {
          setPixelColor(i, Color1);
        }
        else // Fading tail
        {
          setPixelColor(i, DimColor(getPixelColor(i)));
        }
      }
      show();
      Increment();
    }

    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
      ActivePattern = FADE;
      Interval = interval;
      TotalSteps = steps;
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = dir;
    }

    // Update the Fade Pattern
    void FadeUpdate()
    {
      // Calculate linear interpolation between Color1 and Color2
      // Optimise order of operations to minimize truncation error
      uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
      uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
      uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;

      ColorSet(Color(red, green, blue));
      show();
      Increment();
    }

    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
      // Shift R, G and B components one bit to the right
      uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
      return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
      for (int i = 0; i < numPixels(); i++)
      {
        setPixelColor(i, color);
      }
      show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
      return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
      return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
      return color & 0xFF;
    }

    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
      WheelPos = 255 - WheelPos;
      if (WheelPos < 85)
      {
        return Color(255 - WheelPos * 3, 0, WheelPos * 3);
      }
      else if (WheelPos < 170)
      {
        WheelPos -= 85;
        return Color(0, WheelPos * 3, 255 - WheelPos * 3);
      }
      else
      {
        WheelPos -= 170;
        return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
      }
    }
};

void Ring1Complete();

// create one ring to be used.
NeoPatterns Ring1(24, 6, NEO_GRBW + NEO_KHZ800, &Ring1Complete);


// Initialise everything and prepare to start
void setup()
{
  Serial.begin(115200);

  // Initialise the ring
  Ring1.begin();
  Ring1.setBrightness(BRIGHTNESS);

  // Kick off a pattern
  Ring1.TheaterChase(Ring1.Color(255, 255, 0), Ring1.Color(0, 0, 50), 100);

  while (! Serial) {        // wait until serial port opens for native USB devices
    delay(1);
  }

  // serial.print calls are commented out to avoid adding unwanted info to what is sent to openFrameworks,
  // but are useful for debugging, so not removed completely

  //Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    //Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  //Serial.println(F("VL53L0X API Simple Ranging example\n\n"));


}

// Main loop
void loop()
{


  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);    // pass in 'true' to get debug data printout!

  val = measure.RangeMilliMeter;       // set the value variable to the distance from the sensor

  if (Serial.available() > 0) {
    Serial.read();
    printVal(val);
  }

  //  Serial.print("Reading a measurement... ");

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    ///   Serial.print("Distance (mm): ");
    //    Serial.println(val,'\n');
  } else {
    //    Serial.println(" out of range ");
  }

  //    Serial.println(val);
  delay(1);


  // Update the rings.
  Ring1.Update();




  if (val < 350 && val != 0) trigger = true;

//  if (val < 350 && val > 25) sequenceOn = true;




  else(trigger = false);

  if (trigger) {
    Ring1.ActivePattern = SCANNER;
    Ring1.Interval = 2;
//    if (!timerOn) timerOn = true;
  }

  else // Back to normal operation
  {
    //    sequenceOn = false;
    // Switch Ring1 to FADE pattern
    Ring1.ActivePattern = FADE;
    Ring1.Interval = 100;
//    sequenceOn = false;
  }
}

//------------------------------------------------------------
//Completion Routines - get called on completion of a pattern
//------------------------------------------------------------

// Ring1 Completion Callback
void Ring1Complete()
{

  if (trigger) {
    //    sequenceOn = true;
    //    if (scannerSpeed > 10) scannerSpeed -= 10;
    //    if (scannerSpeed <= 10) scannerSpeed -= 1;
    //    if (scannerSpeed <= 1) {
    //      scannerSpeed = 1;
    //      sequenceOn = true;
    //    }
    //
    //
    //    if (scannerSpeed <= 1) {
    //      sendOut = true;
    //
    //    }
    //
    //    if (sendOut) {
    //      trigger = false;
    //      startTime = millis();
    //      sendOut = false;
    //      scannerSpeed = 35;
    //                  sequenceOn = false;
    //    }
  }

  if (digitalRead(9) == LOW)  // Button #2 pressed
  {
    // Alternate color-wipe patterns with Ring2
    Ring1.Color1 = Ring1.Wheel(random(255));
    Ring1.Interval = 20000;
  }
  else  // Retrn to normal
  {
    Ring1.Reverse();
  }
}


// ----------------------------------------------------------------------------
void printVal(int val) {

  // break the integer into two bytes
  // to be read and converted back into integers in openFrameworks

  byte highByte = ((val >> 8) & 0xFF);
  byte lowByte = ((val >> 0) & 0xFF);

  Serial.write(highByte);
  Serial.write(lowByte);
//  Serial.write(sequenceOn);
}

