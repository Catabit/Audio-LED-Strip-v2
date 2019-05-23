#include <arduinoFFT.h>
#include "FastLED.h"

#define SAMPLES 64  //FFT input samples from the ADC. Must be a power of 2
#define num_buckets 32  // Total number of  frequency buckets after FFT, must be <= SAMPLES/2
#define max_out 80  // FFT output is mapped from 0 to max_out

#define updateLEDS 6  // How many leds the animation moves at a time. Must be a multiple of 2 and >= 4
#define NUM_LEDS 100 // How many leds the strip has in total. Must be a multiple of 2
#define DATA_PIN PD5  // Led Strip DATA pin
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS  32
#define MAXHUE 240 // purplish blue

//ADC input settings
#define ADC_MIC 0b11000100 // ADC setting to use the MIC
#define ADC_JACK 0b11000111 // ADC setting to use the 3.5mm JACK

//##### NOISE COMPENSATION
//The first bucket to start being relevant
#define START_JACK 2
#define START_MIC 8

// The maximum noise on the first bucket
#define NOISE_JACK 5
#define NOISE_MIC 80

// The last bucket on which noise needs to be removed
#define DENOISE_END_BUCKET 15

// Overall noise floor
#define OVERALL_NOISE 5

// Linear scaler (lowers the impact of the first buckets on the output color)
#define BASE_SCALER 5
//##### END NOISE COMPENSATION

#define ledPin 15 // pin the controls the output LED
#define buttonPin 2 // pin that controls the states
#define debounceDelay 50

CHSV bucket_color[num_buckets];
char bucket_scaler[num_buckets];

CRGB leds[NUM_LEDS];


double vReal[SAMPLES];
double vImag[SAMPLES];
char data_avgs[num_buckets];

char outValue;
char previousState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

char ledState = HIGH; // state of the output LED

// State machine states
char state = 0; // 0-jack 1-jack_reverse 2-jack_solid 3-mic

arduinoFFT FFT = arduinoFFT();                                    // FFT object
char startBucket = START_JACK;
char noiseLevel = NOISE_JACK;


int denoise(int input, int bucket, int noiseLevel) {
  int compensation = (noiseLevel / DENOISE_END_BUCKET) * bucket - noiseLevel;
  if (compensation < 0) input += compensation;
  if (input < OVERALL_NOISE) return 0;
  if (input < 0) return 0;

  return input;
}

int scaler(int x) {
  return BASE_SCALER - x;
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  ADCSRA = 0b11100101;  // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADMUX = ADC_JACK;

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);

  for (int i = 0; i < num_buckets; i++) {
    bucket_color[i] = CHSV(i * (MAXHUE / num_buckets), 255, 255); //sets the buckets hues
    bucket_scaler[i] = scaler(i);
    if (bucket_scaler[i] < 1)
      bucket_scaler[i] = 1;
  }

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1000);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading == HIGH && previousState == LOW && millis() - lastDebounceTime > debounceDelay)
  {
    state++;
    if (state == 4) state = 0;

    // 0-jack 1-jack_reverse 2-jack_solid 3-mic
    switch (state) {
      case 0:
        for (int i = 0; i < num_buckets; i++)
          bucket_color[i] = CHSV(i * (MAXHUE / num_buckets), 255, 255); //sets the buckets hues
        ADMUX = ADC_JACK;
        startBucket = START_JACK;
        noiseLevel = NOISE_JACK;
        ledState = HIGH;
        break;
      case 1:
        for (int i = 0; i < num_buckets; i++)
          bucket_color[i] = CHSV((MAXHUE - 30) - i * ((MAXHUE - 30) / num_buckets), 255, 255); //sets the buckets hues in reverse (blue to red)
        ADMUX = ADC_JACK;
        startBucket = START_JACK;
        noiseLevel = NOISE_JACK;
        ledState = HIGH;
        break;
      case 2:
        for (int i = 0; i < num_buckets; i++)
          bucket_color[i] = CHSV(i * (MAXHUE / num_buckets), 255, 255); //sets the buckets hues
        ADMUX = ADC_JACK;
        startBucket = START_JACK;
        noiseLevel = NOISE_JACK;
        ledState = HIGH;
        break;
      case 3:
        for (int i = 0; i < num_buckets; i++)
          bucket_color[i] = CHSV(i * (MAXHUE / num_buckets), 255, 255); //sets the buckets hues
        ADMUX = ADC_MIC;
        startBucket = START_MIC;
        noiseLevel = NOISE_MIC;
        ledState = LOW;
        break;
    }
    digitalWrite(ledPin, ledState);
    lastDebounceTime = millis();
  }
  previousState = reading;

  // ADC readings
  for (int i = 0; i < SAMPLES; i++)
  {
    while (!(ADCSRA & 0x10)); // wait for conversion
    ADCSRA = 0b11110101 ; // clear ADIF
    int value = ADC - 512 ; // read from ADC and remove the DC offset
    vReal[i] = value / 8; // compress the value
    vImag[i] = 0;
  }


  // FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);


  // average the collected data if the number of samples is greater than the number of buckets (num_buckets)
  int step = (SAMPLES / 2) / num_buckets;

#if num_buckets > SAMPLES/2
  int c = 0;
  for (int i = 0; i < (SAMPLES / 2); i += step)
  {
    data_avgs[c] = 0;
    for (int k = 0 ; k < step ; k++) {
      data_avgs[c] = data_avgs[c] + vReal[i + k];
    }
    data_avgs[c] = data_avgs[c] / step;
    c++;
  }
#endif

  // black
  CRGB nColor(0, 0, 0);
  int r = 0, g = 0, b = 0;

  // Compute the next color based on all the buckets and their values
  for (int i = startBucket; i < num_buckets; i++)
  {
    if (step == 1) data_avgs[i] = vReal[i];
    data_avgs[i] = constrain(data_avgs[i], 0, 80);          // set max & min values for buckets
    data_avgs[i] = map(data_avgs[i], 0, 80, 0, max_out);        // remap averaged values to max_out

    //get the mapped data and remove the noise
    outValue = data_avgs[i];
    outValue = denoise(data_avgs[i], i, noiseLevel);

    /*
      The new color is a mix of all the colors of the buckets (bucket_color[index]), but
      mixed depending on the values of the buckets so that the most dominant frequency
      has a priority and stands out
    */
    CRGB temp = bucket_color[i];

    nColor.r += temp.r * outValue / (max_out * bucket_scaler[i]);
    nColor.g += temp.g * outValue / (max_out * bucket_scaler[i]);
    nColor.b += temp.b * outValue / (max_out * bucket_scaler[i]);

#ifdef DEBUG
    Serial.print(i);
    Serial.print("  ");
    Serial.println((int) outValue);
#endif
  }

  //Do the animations
  if (state != 2) { // if the animation is not 'solid' (all leds the same)
    //Shift the left half to the left
    for (int i = 0; i <= NUM_LEDS / 2 - updateLEDS / 2 ; i++) {
      leds[i] = leds[i + updateLEDS / 2];
    }

    //Shift the right half to the right
    for (int i = NUM_LEDS - 1; i >= NUM_LEDS / 2 + updateLEDS / 2; i--) {
      leds[i] = leds[i - updateLEDS / 2];
    }

    //Add new middle leds using the new color
    for (int i = NUM_LEDS / 2 - updateLEDS / 2 + 2; i <= NUM_LEDS / 2 + updateLEDS / 2 - 2 ; i++) {
      leds[i] = nColor;
    }

    //Blend the new color with the old one on the borders
    CRGB oldColor = leds[NUM_LEDS / 2 - updateLEDS / 2 - 1];
    CRGB newColorLo = nColor, newColorMid = nColor;
    if (oldColor) {
      newColorLo = CRGB((oldColor.r * 7 + nColor.r * 3) / 10, (oldColor.g * 7 + nColor.g * 3) / 10, (oldColor.b * 7 + nColor.b * 3) / 10);
      newColorMid = CRGB((oldColor.r * 2 + nColor.r * 3) / 5, (oldColor.g * 2 + nColor.g * 3) / 5, (oldColor.b * 2 + nColor.b * 3) / 5);
    }

    //oldColor 100%
    leds[NUM_LEDS / 2 - updateLEDS / 2] = newColorLo; // more old less new
    leds[NUM_LEDS / 2 - updateLEDS / 2 + 1] = newColorMid; // less old more new
    //nColor 100%
    //MIDDLE OF THE STRIP
    //nColor 100%
    leds[NUM_LEDS / 2 + updateLEDS / 2 - 1] = newColorMid; // less old more new
    leds[NUM_LEDS / 2 + updateLEDS / 2] = newColorLo; // more old less new
    //oldColor 100%

  } else { // all the leds are set to the new color
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = nColor;
  }

  //Send the data to the LED strip
  FastLED.show();
}
