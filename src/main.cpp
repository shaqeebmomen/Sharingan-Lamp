#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Software defines
#define SAMPLE_SIZE 1024 // keep to a power of 2
#define MAX_SCALE 1024 // value to scale the samples by
// TODO test realistic sample scaling and adjust gain on mic breakout
#define FREQ_SAMPLE 8000 // hz
//TODO experimentally determine a realistic sample frequency to use (run a for loop and time how long it takes to complete)

// LED Count defines
#define LED_COUNT_INNER 6
#define LED_COUNT_MIDDLE 6
#define LED_COUNT_OUTER 6

// Pin defines
#define LED_PIN_INNER 2
#define LED_PIN_MIDDLE 3
#define LED_PIN_OUTER 4
#define MIC_PIN 6

enum stripName : uint8_t
{
  INNER,
  MIDDLE,
  OUTER
};

// LED objects
Adafruit_NeoPixel strips[] = {Adafruit_NeoPixel(LED_COUNT_INNER, LED_PIN_INNER, NEO_GRB + NEO_KHZ800),
                              Adafruit_NeoPixel(LED_COUNT_MIDDLE, LED_PIN_MIDDLE, NEO_GRB + NEO_KHZ800),
                              Adafruit_NeoPixel(LED_COUNT_OUTER, LED_PIN_OUTER, NEO_GRB + NEO_KHZ800)};

// Data objects
int micSignal[SAMPLE_SIZE]; // mic signal 
uint16_t bucketAvg[3]; // averages for each of the 3 zones of frequences
uint8_t lastLED[3];



// Function prototypes
void initLEDs(void);
void writeStrips(int);
void writeStaticRainbow(Adafruit_NeoPixel*,uint8_t);
void setup()
{
  initLEDs();
}


// TODO control loop freq to match sampling freq (maybe two loops,1 for sampling and the other running faster for LEDs)
void loop()
{
  // put your main code here, to run repeatedly:
}

void initLEDs()
{
  // Init LED strips
  for (uint8_t i = INNER; i != OUTER; i++)
  {
    strips[i].begin();
    strips[i].show();
    strips[i].setBrightness(50);
  }
}

// TODO function to scale fft buckets
// TODO function to seperate data into 3 buckets

void writeStrips(int bucketAvg)
{
  // calculate the last LED to turn on for each bucket
  // remember to only use half the data
}



/*******************ANIMATION FUNCTIONS**********************/
void writeStaticRainbow(Adafruit_NeoPixel *strip, uint8_t count)
{

  for (uint8_t i = 0; i < strip->numPixels(); i++)
  {
    if (i < count)
    {
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = 0 + (i * 65536L / strip->numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip->setPixelColor(i, strip->gamma32(strip->ColorHSV(pixelHue)));
    }
    else
    {
      // All LEDs beyond cap should be turned off
      strip->setPixelColor(i, strip->Color(0, 0, 0));
    }
  }
}