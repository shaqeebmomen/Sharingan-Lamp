#include <Arduino.h>
#include "Adafruit_ZeroFFT.h"
#include <Adafruit_NeoPixel.h>

// Conditional Compilation defines
// #define DEBUG_FFT
#define DEBUG_BUCKET

// Software defines
#define SAMPLE_SIZE 2048 // keep to a power of 2
#define MAX_SCALE 1023   // value to scale the samples by
// TODO test realistic sample scaling and adjust gain on mic breakout
#define FREQ_SAMPLE 68267 // hz
#define FREQ_LOW 500      // Frequencies below this trigger
#define FREQ_HIGH 800
#define FREQ_MAX 1200

// LED Count defines
#define LED_COUNT_INNER 6
#define LED_COUNT_MIDDLE 4
#define LED_COUNT_OUTER 8

// Pin defines
#define LED_PIN_INNER 2
#define LED_PIN_MIDDLE 3
#define LED_PIN_OUTER 4
#define MIC_PIN 6

enum stripName : uint8_t
{
  INNER = 0,
  MIDDLE,
  OUTER
};

// LED objects
Adafruit_NeoPixel strips[] = {Adafruit_NeoPixel(LED_COUNT_INNER, LED_PIN_INNER, NEO_GRB + NEO_KHZ800),
                              Adafruit_NeoPixel(LED_COUNT_MIDDLE, LED_PIN_MIDDLE, NEO_GRB + NEO_KHZ800),
                              Adafruit_NeoPixel(LED_COUNT_OUTER, LED_PIN_OUTER, NEO_GRB + NEO_KHZ800)};

// Adafruit_NeoPixel strip(LED_COUNT_INNER,LED_PIN_INNER,NEO_GRB+NEO_KHZ800);

// Data objects
int16_t micData[SAMPLE_SIZE]; // mic signal
int16_t bucketAvg[3];         // averages for each of the 3 zones of frequences
uint8_t lastLED[3];
uint32_t lastSampleTS; // timestamp of last sample capture

// Function prototypes
void initLEDs(void);
void takeData(int16_t *);
void compactData(int16_t *, int16_t *);
void writeStrips(int);
void writeStaticRainbow(Adafruit_NeoPixel *, uint8_t);
void colorWipe(Adafruit_NeoPixel *, uint32_t, int);
void rainbow(Adafruit_NeoPixel *, int);

uint8_t test;

void setup()
{
  Serial.begin(115200);
  initLEDs();
  test = 0;
}

// TODO control loop freq to match sampling freq (maybe two loops,1 for sampling and the other running faster for LEDs)
void loop()
{
  // takeData(micData);
  // compactData(micData, bucketAvg);

  // delay(500);
  writeStaticRainbow(&strips[INNER], test);
  test = (test + 1) % strips[INNER].numPixels();
  delay(200);
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

void takeData(int16_t *arr)
{
#ifdef DEBUG_SAMPLE_TIME
  lastSampleTS = millis();
#endif
  int32_t avg = 0;
  for (uint16_t i = 0; i < SAMPLE_SIZE; i++)
  {
    int16_t val = analogRead(MIC_PIN);
    avg += val;
    arr[i] = val;
  }
#ifdef DEBUG_SAMPLE_TIME
  uint32_t sampleTime = millis() - lastSampleTS;
  Serial.print("Sample Size: ");
  Serial.print(SAMPLE_SIZE);
  Serial.print("\t");
  Serial.print("Capture time: ");
  Serial.println(sampleTime);
#endif

  //remove DC offset and gain up to 16 bits
  avg = avg / SAMPLE_SIZE;
  for (int i = 0; i < SAMPLE_SIZE; i++)
    arr[i] = (arr[i] - avg);

  ZeroFFT(arr, SAMPLE_SIZE);

#ifdef DEBUG_FFT
  for (int i = 0; i < SAMPLE_SIZE / 2; i++)
  {
    //print the frequency
    Serial.print(FFT_BIN(i, FREQ_SAMPLE, SAMPLE_SIZE));
    Serial.print(" Hz: ");

    //print the corresponding FFT output
    Serial.println(arr[i]);
  }
#endif
}

void compactData(int16_t *data, int16_t *bucket)
{
  uint16_t count[3] = {0, 0, 0};
  // Reset bucket
  for (uint8_t i = 0; i < 3; i++)
  {
    bucket[i] = 0;
  }
  for (uint16_t i = 0; i < SAMPLE_SIZE / 2; i++)
  {
    float currentFreq = FFT_BIN(data[i], FREQ_SAMPLE, SAMPLE_SIZE);
    if (currentFreq < FREQ_LOW && currentFreq > 0)
    {
      bucket[0] += data[i];
      count[0]++;
    }
    if (currentFreq > FREQ_LOW && currentFreq < FREQ_HIGH)
      ;
    {
      bucket[1] += data[i];
      count[1]++;
    }
    if (currentFreq > FREQ_HIGH && currentFreq < FREQ_MAX)
    {
      bucket[2] += data[i];
      count[1]++;
    }
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    // bucket[i] /= count[i];
  }

#ifdef DEBUG_BUCKET
  for (uint8_t i = 0; i < 3; i++)
  {
    Serial.println();
    Serial.print("Bucket: ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(bucket[i]);
  }
#endif
}
// TODO function to scale fft buckets

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
      // All LEDs beyond count should be turned off
      strip->setPixelColor(i, strip->Color(0, 0, 0));
    }
  }
  strip->show();
}

void colorWipe(Adafruit_NeoPixel *strip, uint32_t color, int wait)
{
  for (int i = 0; i < strip->numPixels(); i++)
  {                                 // For each pixel in strip...
    strip->setPixelColor(i, color); //  Set pixel's color (in RAM)
    strip->show();                  //  Update strip to match
    delay(wait);                    //  Pause for a moment
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(Adafruit_NeoPixel *strip, int wait)
{
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256)
  {
    for (int i = 0; i < strip->numPixels(); i++)
    { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip->numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip->setPixelColor(i, strip->gamma32(strip->ColorHSV(pixelHue)));
    }
    strip->show(); // Update strip with new contents
    delay(wait);   // Pause for a moment
  }
}
