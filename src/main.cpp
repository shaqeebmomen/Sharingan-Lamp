#include <Arduino.h>
#include "Adafruit_ZeroFFT.h"
#include <Adafruit_NeoPixel.h>

// Conditional Compilation defines
// #define DEBUG_FFT
// #define DEBUG_BUCKET

// Software defines
#define SAMPLE_SIZE 2048 // keep to a power of 2
#define MAX_SCALE 45.0   // value to scale the samples by
#define FREQ_SAMPLE 68267 // hz
#define FREQ_LOW 200.0    // Frequencies below this trigger
#define FREQ_HIGH 600.0
#define FREQ_MAX 4000.0

#define ANIM_COUNT 5
// LED Count defines
#define LED_COUNT_INNER 5
#define LED_COUNT_MIDDLE 4
#define LED_COUNT_OUTER 8

// Pin defines
#define LED_PIN_INNER 4
#define LED_PIN_MIDDLE 8
#define LED_PIN_OUTER 9
#define MIC_PIN 5
#define BTN_PIN 7

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
float bucketAvg[3];           // averages for each of the 3 zones of frequences
// uint8_t lastLED[3];
// uint32_t lastSampleTS; // timestamp of last sample capture
uint8_t animIndex = 0;

// Function prototypes
void initLEDs(void);
void takeData(int16_t *);
void compactData(int16_t *, float *);
uint8_t calcCap(Adafruit_NeoPixel *, float);
uint8_t calcBrightCap(float);
void writeStaticRainbow(Adafruit_NeoPixel *, uint8_t);
void colorWipe(Adafruit_NeoPixel *, uint32_t);
void rainbow(Adafruit_NeoPixel *, int wait);
void writeSinglePixel(Adafruit_NeoPixel *, float, uint32_t);
void blueLightning(Adafruit_NeoPixel *, float *);
void fullRainbow(Adafruit_NeoPixel *, float *);
void glowingRed(Adafruit_NeoPixel *, float *);
void staticRed(Adafruit_NeoPixel *, float *);
void resetLights(Adafruit_NeoPixel *, float *);

void (*anims[ANIM_COUNT])(Adafruit_NeoPixel *, float *) = {fullRainbow, blueLightning, glowingRed, staticRed, resetLights};

void setup()
{
  Serial.begin(9600);
  pinMode(BTN_PIN, INPUT_PULLUP);
  initLEDs();
}

// TODO control loop freq to match sampling freq (maybe two loops,1 for sampling and the other running faster for LEDs)
void loop()
{
  takeData(micData);
  compactData(micData, bucketAvg);
  // delay(300);
  anims[animIndex](strips, bucketAvg);

  if (!digitalRead(BTN_PIN))
  {
    delay(50);
    if (!digitalRead(BTN_PIN))
    {
      animIndex = (animIndex + 1) % ANIM_COUNT;
    }
  }

  // blueLightning(strips, bucketAvg);
  // glowingRed(strips, bucketAvg);
}

void initLEDs()
{
  // Init LED strips
  for (uint8_t i = INNER; i <= OUTER; i++)
  {
    strips[i].begin();
    strips[i].show();
    strips[i].setBrightness(130);
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

  //remove DC offset and add gain
  avg = avg / SAMPLE_SIZE;
  for (int i = 0; i < SAMPLE_SIZE; i++)
    arr[i] = (arr[i] - avg) * 128; //64

  ZeroFFT(arr, SAMPLE_SIZE);

#ifdef DEBUG_FFT
  for (int i = 0; i < SAMPLE_SIZE / 2; i++)
  {
    Serial.print(",");
    //print the frequency
    Serial.print(FFT_BIN(i, FREQ_SAMPLE, SAMPLE_SIZE));
    // Serial.print(" Hz: ");
    Serial.print(",");

    //print the corresponding FFT output
    Serial.print(arr[i]);
    Serial.print(",");
    Serial.println();
  }
#endif
}

void compactData(int16_t *data, float *bucket)
{
  uint16_t count[3] = {0, 0, 0};
  // Reset bucket
  for (uint8_t i = 0; i < 3; i++)
  {
    bucket[i] = 0;
  }
  for (uint16_t i = 0; i < SAMPLE_SIZE / 2; i++)
  {
    float currentFreq = FFT_BIN(i, FREQ_SAMPLE, SAMPLE_SIZE);
    // Serial.println(currentFreq);
    // Serial.println("...");
    if (currentFreq < FREQ_LOW && currentFreq > 0.0)
    {
      bucket[0] += (float)data[i];
      count[0]++;
      // Serial.print("LOW: ");
      // Serial.println((float)data[i]);
      // Serial.println(bucket[0]);
      // Serial.println(count[0]);
    }
    if (currentFreq > FREQ_LOW && currentFreq < FREQ_HIGH)
    {
      bucket[1] += (float)data[i];
      count[1]++;
      // Serial.print("MED: ");
      // Serial.println((float)data[i]);
      // Serial.println(bucket[1]);
      // Serial.println(count[1]);
    }
    if (currentFreq > FREQ_HIGH && currentFreq < FREQ_MAX)
    {
      bucket[2] += (float)data[i] * 2.0; // adding a slight gain for higher frequencies
      count[2]++;
      // Serial.print("HIGH: ");
      // Serial.println((float)data[i]);
      // Serial.println(bucket[2]);
      // Serial.println(count[2]);
    }
    // delay(200);
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    bucket[i] /= (float)count[i];
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

uint8_t calcCap(Adafruit_NeoPixel *strip, float bucketAvg)
{
  uint8_t cap;
  if (bucketAvg < MAX_SCALE)
  {
    cap = (float)strip->numPixels() * bucketAvg / MAX_SCALE;
  }
  else
  {
    cap = strip->numPixels();
  }

  return cap;
}

uint8_t calcBrightCap(float bucketAvg)
{
  return (float)255 * bucketAvg / MAX_SCALE;
}

/*******************ANIMATION FUNCTIONS**********************/

void staticRed(Adafruit_NeoPixel *strips, float *buckets)
{
  for (uint8_t i = INNER; i <= OUTER; i++)
  {
    colorWipe(&strips[i], strips[i].Color(255, 0, 0));
  }
}

void blueLightning(Adafruit_NeoPixel *strips, float *buckets)
{
  float scale = (buckets[0] / MAX_SCALE);
  colorWipe(&strips[INNER], strips[INNER].Color(20 * scale, 60 * scale, 255 * scale));
  for (uint8_t i = MIDDLE; i <= OUTER; i++)
  {
    writeSinglePixel(&strips[i], buckets[OUTER - i], strips[INNER].Color(20 * scale, 60 * scale, 255 * scale));
  }
}

void fullRainbow(Adafruit_NeoPixel *strips, float *buckets)
{
  for (uint8_t i = INNER; i <= OUTER; i++)
  {
    writeStaticRainbow(&strips[i], calcCap(&strips[i], bucketAvg[OUTER - i]));
  }
}

void glowingRed(Adafruit_NeoPixel *strips, float *buckets)
{
  for (uint8_t i = INNER; i <= OUTER; i++)
  {
    colorWipe(&strips[i], strips[INNER].Color(calcBrightCap(buckets[OUTER - i]), 0, 0));
  }
}

void resetLights(Adafruit_NeoPixel *strips, float *buckets)
{
  for (uint8_t i = INNER; i <= OUTER; i++)
  {
    colorWipe(&strips[i], strips[i].Color(0, 0, 0));
  }
}

/*****************LOWER LEVEL ANIMATION FUNCTIONS***********/
void writeSinglePixel(Adafruit_NeoPixel *strip, float bucket, uint32_t color)
{
  uint32_t off = strip->Color(0, 0, 0);
  for (uint8_t i = 0; i < strip->numPixels(); i++)
  {
    if (i != calcCap(strip, bucket))
    {
      strip->setPixelColor(i, off);
    }
    else
    {
      strip->setPixelColor(i, color);
    }
  }
  strip->show();
}

// Writes a static rainbow to a strip that's length is based on a passed cap
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

void colorWipe(Adafruit_NeoPixel *strip, uint32_t color)
{
  for (int i = 0; i < strip->numPixels(); i++)
  {                                 // For each pixel in strip...
    strip->setPixelColor(i, color); //  Set pixel's color (in RAM)
  }
  strip->show(); //  Update strip to match
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
    delay(wait);
  }
}
