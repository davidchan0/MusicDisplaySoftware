#include <Arduino.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include <time.h>



const char* WIFI_SSID = "WhiteSky-InspireAtlanta";      
const char* WIFI_PASS = "r69kqxfw";      
const char* ACR_HOST = ""; 
const char* ACR_ACCESS_KEY = "";       
const char* ACR_ACCESS_SECRET = ""; 


const char* OPENAI_API_KEY = ""; 


#define I2S_NUM           I2S_NUM_0
#define I2S_SD_PIN        16
#define I2S_WS_PIN        17
#define I2S_SCK_PIN       18
#define I2S_SAMPLE_RATE   16000


constexpr uint8_t NUM_STRIPS = 10;        

const uint8_t LED_PIN_A = 2;             
constexpr uint16_t TOTAL_LEDS_PIN_A = 60; 

const uint8_t LED_PIN_B = 1;             
constexpr uint16_t TOTAL_LEDS_PIN_B = 60; 


const uint8_t BTN_NORMAL  = 3;   
const uint8_t BTN_BEAT    = 4;   
const uint8_t BTN_EMOTION = 5;   // Changed from BTN_NOTES
const uint8_t BTN_ROLL    = 6;   
const uint8_t BTN_AI      = 7;   
const uint8_t BTN_COLOR   = 11;  


const uint8_t SPEAKER_PIN = 10; 


#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
LiquidCrystal_I2C lcdMain(0x27, 16, 2); 
LiquidCrystal_I2C lcdAux(0x26, 16, 2);  


const int SAMPLE_RATE = 16000;
const int CAPTURE_SECONDS = 6;
const char* WAV_PATH = "/sample.wav";


bool DEBUG = true;


const unsigned long STATS_PAGE_INTERVAL_MS = 4000; 
const unsigned long AUX_ROTATE_INTERVAL_MS = 3000; 




constexpr uint16_t SAMPLES = 256;
constexpr double   SAMPLING_FREQ = (double)I2S_SAMPLE_RATE;
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);


struct Band { float lowHz, highHz; };
Band BAND_DEFS[NUM_STRIPS] = {
  { 20.0f,   120.0f  }, 
  { 120.0f,  240.0f  }, 
  { 240.0f,  400.0f  }, 
  { 400.0f,  600.0f  }, 
  { 600.0f,  900.0f  }, 
  { 900.0f,  1400.0f }, 
  { 1400.0f, 2200.0f }, 
  { 2200.0f, 3500.0f }, 
  { 3500.0f, 5500.0f }, 
  { 5500.0f, 8000.0f }  
};


const uint8_t PALETTES = 5;
const char* PALETTE_NAMES[PALETTES] = { "Default", "Warm", "Cool", "Forest", "Neon" };

const uint8_t PALETTE_DATA[PALETTES][NUM_STRIPS][3] = {
  {
    { 0, 200, 40 }, { 0,120,200 }, {0,80,255}, {80,200,255}, {255,160,0},
    {255,100,0}, {200,40,0}, {180,0,120}, {160,0,200}, {255,200,80}
  },
  {
    {255, 120, 10}, {255,80,10}, {255,60,0}, {230,100,10}, {200,120,10},
    {180,60,0}, {160,40,0}, {140,30,10}, {200,50,0}, {255,180,60}
  },
  {
    {0,180,160}, {0,120,200}, {0,80,255}, {40,120,255}, {80,200,255},
    {0,150,200}, {0,90,200}, {30,60,200}, {60,40,200}, {100,160,255}
  },
  {
    {0,180,80}, {0,140,60}, {20,120,40}, {40,100,30}, {80,160,40},
    {120,180,60}, {140,120,30}, {100,90,20}, {80,110,40}, {160,200,120}
  },
  {
    {255,0,120}, {0,255,200}, {120,0,255}, {255,80,0}, {255,200,0},
    {0,255,80}, {255,0,200}, {200,0,255}, {0,180,255}, {255,120,0}
  }
};


uint8_t currentColorRGB[NUM_STRIPS][3];


float ambientNoise[NUM_STRIPS];
float peakVal[NUM_STRIPS];
float levelSmoothed[NUM_STRIPS];

const float PEAK_DECAY = 0.96f;
const float ATTACK_ALPHA = 0.60f;
const float DECAY_ALPHA  = 0.05f;
const float COMPRESS_EXP = 0.60f;
const float DOMINANT_PEAK_FACTOR = 4.0f;
const float HARMONIC_ATTENUATION = 0.05f;
const float I2S_MAX = 8388608.0f; 


uint16_t ledsPerSection[NUM_STRIPS];
uint16_t sectionStartIdx[NUM_STRIPS];   
uint8_t  sectionChainId[NUM_STRIPS];    
bool     sectionDirection[NUM_STRIPS];  


Adafruit_NeoPixel stripA(TOTAL_LEDS_PIN_A, LED_PIN_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripB(TOTAL_LEDS_PIN_B, LED_PIN_B, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel* strips[2]; 


static int32_t i2s_buffer[SAMPLES];


enum Mode { IDLE=0, NORMAL=1, BEAT=2, EMOTION=3, ROLL=4 };
volatile Mode activeMode = IDLE;


// Emotion types
enum EmotionType { EMOTION_NONE=0, EMOTION_LOVE=1, EMOTION_HAPPY=2, EMOTION_SAD=3, EMOTION_ANGRY=4 };
EmotionType currentEmotion = EMOTION_NONE;


bool showStatsOnMain = false;


const unsigned long DEBOUNCE_MS = 50;
int lastReadingNormal = HIGH, lastReadingBeat = HIGH, lastReadingEmotion = HIGH, lastReadingRoll = HIGH, lastReadingAI = HIGH, lastReadingColor = HIGH;
unsigned long lastDebounceNormal = 0, lastDebounceBeat = 0, lastDebounceEmotion = 0, lastDebounceRoll = 0, lastDebounceAI = 0, lastDebounceColor = 0;
int stableNormal = HIGH;
int stableBeat   = HIGH;
int stableEmotion  = HIGH;
int stableRoll   = HIGH;
int stableAI     = HIGH;
int stableColor  = HIGH;


float prevMagnitudes[SAMPLES/2];
float fluxMean = 1e-6f;
float rmsMean = 1e-6f;
unsigned long lastBeatMillis = 0;
unsigned long beatExpireMillis = 0;
float lastDetectedBeatStrength = 0.0f;


const float BEAT_COMPRESS_EXP = 0.55f;
const float FLUX_ALPHA = 0.50f;
const float FLUX_THRESHOLD_FACTOR = 1.7f;
const float RMS_ALPHA = 0.60f;
const float RMS_THRESHOLD_FACTOR = 1.7f;
const uint16_t BEAT_HOLD_MS = 220;
const uint16_t MIN_MS_BETWEEN_BEATS = 80;
const bool ENABLE_MICRO_BEATS = true;
const float MICRO_BEAT_TRIGGER = 1.15f;
const float MICRO_BEAT_SCALE = 0.55f;
const float RMS_MIX_WEIGHT = 0.40f;
const float LED_ON_THRESHOLD = 0.0005f; 


float rollingAvg[NUM_STRIPS];
const float ROLL_ALPHA = 0.995f;
const float ROLL_MIN_PEAK = 1e-3f;
const float ROLL_DISPLAY_GAIN = 1.0f;


constexpr uint8_t LEDS_PER_STRIP = 12;

bool welcomed = false; 


bool aiBusy = false;
bool aiRequested = false;


String acrTitle = "";
String acrArtist = "";
String acrCombined = "";
String acrTheme = "";
String acrEmotion = ""; // NEW: stores detected emotion
unsigned long auxRotateMillis = 0;
int auxRotateIndex = 0;
const unsigned long AUX_ROTATE_INTERVAL_MS_LOCAL = AUX_ROTATE_INTERVAL_MS; 


String auxLastLine0 = "";
String auxLastLine1 = "";
unsigned long lastAuxWriteMs = 0;
const unsigned long AUX_WRITE_MIN_MS = 200; 


enum AiState { AI_IDLE=0, AI_WIFI, AI_RECORDING, AI_UPLOADING, AI_PROCESSING, AI_RESULT, AI_FAIL };
AiState aiState = AI_IDLE;


float lastRmsGlobal = 0.0f;
float lastDominantFreqGlobal = 0.0f;
float lastAvgVisualLevelGlobal = 0.0f;
float lastFluxGlobal = 0.0f;


unsigned long lastStatsFlipMillis = 0;
int statsPageIndex = 0;


int currentPaletteIndex = 0;


void i2sInitRX();
void computeSectionMapping();
void takeSamplesI2S();
void drawSectionLevelNoShow(int sectionIdx, float norm);
void setSectionPixelScaled(int sectionIdx, uint16_t pixelWithinSection, float frac);
void measureAmbientNoise(int frames, float outAmbient[]);
void doBeatBurst(const float bandEnergies[]);
void exitModeCleanup(Mode m);
void switchMode(Mode newMode);
void initNormal();
void initBeat();
void initEmotion();
void initRoll();
void handleButtons();
void runNormalTick();
void runBeatTick();
void runEmotionTick();
void runRollTick();
void playWelcomeMelody();
String acrIdentifyFromFile(const char* wavPath);
String parseAcrTitleArtist(const String& acrResp);
String askGroqForEmotion(String prompt); // Changed function
bool recordAndSaveWav(const char* path);

void updateMainLCDForModeOrStats();
void updateAuxLCDPeriodic();

void applyColorPalette(int idx);
void clearSection(uint8_t s);
void writeAuxLineIfChanged(uint8_t row, const String &text);

// Emotion visual effects
void drawLoveHeart();
void drawHappyWaves();
void drawSadRain();
void drawAngryFire();



void i2sInitRX() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  esp_err_t r = i2s_driver_install(I2S_NUM, &cfg, 0, NULL);
  if (r != ESP_OK) Serial.printf("i2s_driver_install RX failed: %d\n", (int)r);
  else if (DEBUG) Serial.println("i2s_driver_install OK");

  r = i2s_set_pin(I2S_NUM, &pins);
  if (r != ESP_OK) Serial.printf("i2s_set_pin failed: %d\n", (int)r);
  i2s_zero_dma_buffer(I2S_NUM);
}


void computeSectionMapping() {
  const uint8_t sectionsPerChain = NUM_STRIPS / 2; 

  uint16_t baseA = TOTAL_LEDS_PIN_A / sectionsPerChain;
  uint16_t remA = TOTAL_LEDS_PIN_A % sectionsPerChain;
  uint16_t idxA = 0;
  for (uint8_t s = 0; s < sectionsPerChain; ++s) {
    uint8_t virt = s;
    uint16_t n = baseA + (s < remA ? 1 : 0);
    ledsPerSection[virt] = n;
    sectionStartIdx[virt] = idxA;
    sectionChainId[virt] = 0;
    sectionDirection[virt] = (virt % 2 == 0); 
    idxA += n;
  }

  uint16_t baseB = TOTAL_LEDS_PIN_B / sectionsPerChain;
  uint16_t remB = TOTAL_LEDS_PIN_B % sectionsPerChain;
  uint16_t idxB = 0;
  for (uint8_t s = 0; s < sectionsPerChain; ++s) {
    uint8_t virt = sectionsPerChain + s;
    uint16_t n = baseB + (s < remB ? 1 : 0);
    ledsPerSection[virt] = n;
    sectionStartIdx[virt] = idxB;
    sectionChainId[virt] = 1;
    sectionDirection[virt] = (virt % 2 == 0);
    idxB += n;
  }

  if (DEBUG) {
    Serial.println("computeSectionMapping:");
    for (int i=0;i<NUM_STRIPS;i++) {
      Serial.printf(" S%d: chain=%d start=%d len=%d dir=%s\n",
                    i, sectionChainId[i], sectionStartIdx[i], ledsPerSection[i],
                    sectionDirection[i] ? "UP" : "DOWN");
    }
  }
}


void takeSamplesI2S() {
  size_t targetBytes = SAMPLES * sizeof(int32_t);
  size_t totalRead = 0;
  uint8_t* dstPtr = (uint8_t*)i2s_buffer;
  while (totalRead < targetBytes) {
    size_t bytesRead = 0;
    esp_err_t res = i2s_read(I2S_NUM, dstPtr + totalRead, targetBytes - totalRead, &bytesRead, portMAX_DELAY);
    if (res != ESP_OK) {
      Serial.print(F("i2s_read error: "));
      Serial.println((int)res);
      for (size_t i = totalRead/4; i < SAMPLES; ++i) i2s_buffer[i] = 0;
      break;
    }
    totalRead += bytesRead;
  }
  
  float mean = 0.0f;
  for (int i=0;i<SAMPLES;i++) {
    int32_t samp = i2s_buffer[i] >> 8; 
    vReal[i] = (float)samp;
    vImag[i] = 0.0f;
    mean += vReal[i];
  }
  mean /= (float)SAMPLES;
  for (int i=0;i<SAMPLES;i++) vReal[i] -= mean;
}


void setSectionPixelScaled(int sectionIdx, uint16_t pixelWithinSection, float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  uint8_t r = (uint8_t)round((float)currentColorRGB[sectionIdx][0] * frac);
  uint8_t g = (uint8_t)round((float)currentColorRGB[sectionIdx][1] * frac);
  uint8_t b = (uint8_t)round((float)currentColorRGB[sectionIdx][2] * frac);

  uint8_t chain = sectionChainId[sectionIdx];
  uint16_t base = sectionStartIdx[sectionIdx];
  uint16_t nLeds = ledsPerSection[sectionIdx];

  uint16_t relativeIdx = pixelWithinSection;
  if (relativeIdx >= nLeds) return;

  uint16_t absoluteIdx;
  if (sectionDirection[sectionIdx]) {
    absoluteIdx = base + relativeIdx;
  } else {
    absoluteIdx = base + (nLeds - 1 - relativeIdx);
  }

  if (chain == 0) {
    if (absoluteIdx < TOTAL_LEDS_PIN_A) stripA.setPixelColor(absoluteIdx, stripA.Color(r,g,b));
  } else {
    if (absoluteIdx < TOTAL_LEDS_PIN_B) stripB.setPixelColor(absoluteIdx, stripB.Color(r,g,b));
  }
}

// Helper to set any pixel with RGB values directly
void setPixelDirect(int sectionIdx, uint16_t pixelWithinSection, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t chain = sectionChainId[sectionIdx];
  uint16_t base = sectionStartIdx[sectionIdx];
  uint16_t nLeds = ledsPerSection[sectionIdx];

  uint16_t relativeIdx = pixelWithinSection;
  if (relativeIdx >= nLeds) return;

  uint16_t absoluteIdx;
  if (sectionDirection[sectionIdx]) {
    absoluteIdx = base + relativeIdx;
  } else {
    absoluteIdx = base + (nLeds - 1 - relativeIdx);
  }

  if (chain == 0) {
    if (absoluteIdx < TOTAL_LEDS_PIN_A) stripA.setPixelColor(absoluteIdx, stripA.Color(r,g,b));
  } else {
    if (absoluteIdx < TOTAL_LEDS_PIN_B) stripB.setPixelColor(absoluteIdx, stripB.Color(r,g,b));
  }
}


void drawSectionLevelNoShow(int sectionIdx, float norm) {
  norm = constrain(norm, 0.0f, 1.0f);
  uint16_t nLeds = ledsPerSection[sectionIdx];
  uint16_t base = sectionStartIdx[sectionIdx];
  uint8_t chain = sectionChainId[sectionIdx];

  if (norm < LED_ON_THRESHOLD || nLeds == 0) {
    for (uint16_t i = 0; i < nLeds; ++i) {
      uint16_t absoluteIdx = sectionDirection[sectionIdx] ? (base + i) : (base + (nLeds - 1 - i));
      if (chain == 0) {
        if (absoluteIdx < TOTAL_LEDS_PIN_A) stripA.setPixelColor(absoluteIdx, 0);
      } else {
        if (absoluteIdx < TOTAL_LEDS_PIN_B) stripB.setPixelColor(absoluteIdx, 0);
      }
    }
    return;
  }

  float filled = norm * (float)nLeds;
  uint16_t full = (uint16_t)floor(filled);
  float frac = filled - (float)full;

  // CHANGED: Start from top (nLeds-1) and fill downward
  if (full == 0 && frac > 0.03f) {
    for (uint16_t i = 0; i < nLeds; ++i) {
      uint16_t absoluteIdx = sectionDirection[sectionIdx] ? (base + i) : (base + (nLeds - 1 - i));
      if (chain == 0) {
        if (absoluteIdx < TOTAL_LEDS_PIN_A) stripA.setPixelColor(absoluteIdx, 0);
      } else {
        if (absoluteIdx < TOTAL_LEDS_PIN_B) stripB.setPixelColor(absoluteIdx, 0);
      }
    }
    // First pixel from top
    setSectionPixelScaled(sectionIdx, nLeds - 1, frac);
    return;
  }

  // Fill from top down
  for (uint16_t i = 0; i < nLeds; ++i) {
    uint16_t ledFromTop = nLeds - 1 - i; // Reverse: start from top
    
    if (i < full) {
      setSectionPixelScaled(sectionIdx, ledFromTop, 1.0f);
    } else if (i == full && frac > 0.001f) {
      setSectionPixelScaled(sectionIdx, ledFromTop, frac);
    } else {
      uint16_t absoluteIdx = sectionDirection[sectionIdx] ? (base + ledFromTop) : (base + (nLeds - 1 - ledFromTop));
      if (chain == 0) {
        if (absoluteIdx < TOTAL_LEDS_PIN_A) stripA.setPixelColor(absoluteIdx, 0);
      } else {
        if (absoluteIdx < TOTAL_LEDS_PIN_B) stripB.setPixelColor(absoluteIdx, 0);
      }
    }
  }
}

void drawIdleSubdued() {
  stripA.clear();
  stripB.clear();
}


void clearSection(uint8_t s) {
  uint16_t nLeds = ledsPerSection[s];
  uint16_t base = sectionStartIdx[s];
  uint8_t chain = sectionChainId[s];
  for (uint16_t i = 0; i < nLeds; ++i) {
    uint16_t absoluteIdx = sectionDirection[s] ? (base + i) : (base + (nLeds - 1 - i));
    if (chain == 0) { if (absoluteIdx < TOTAL_LEDS_PIN_A) stripA.setPixelColor(absoluteIdx, 0); }
    else { if (absoluteIdx < TOTAL_LEDS_PIN_B) stripB.setPixelColor(absoluteIdx, 0); }
  }
}


void applyColorPalette(int idx) {
  if (idx < 0) idx = 0;
  if (idx >= PALETTES) idx = 0;
  for (int s = 0; s < NUM_STRIPS; ++s) {
    currentColorRGB[s][0] = PALETTE_DATA[idx][s][0];
    currentColorRGB[s][1] = PALETTE_DATA[idx][s][1];
    currentColorRGB[s][2] = PALETTE_DATA[idx][s][2];
  }
  currentPaletteIndex = idx;
  if (DEBUG) Serial.printf("Applied palette %d (%s)\n", idx, PALETTE_NAMES[idx]);
}


void measureAmbientNoise(int frames, float outAmbient[]) {
  if (DEBUG) Serial.printf("measureAmbientNoise: frames=%d\n", frames);
  float acc[NUM_STRIPS];
  for (int i=0;i<NUM_STRIPS;i++) acc[i]=0.0f;
  for (int f=0; f<frames; ++f) {
    takeSamplesI2S();
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    const int maxBin = SAMPLES/2;
    const float freqStep = (float)I2S_SAMPLE_RATE / (float)SAMPLES;
    for (int s=0;s<NUM_STRIPS;s++){
      int binLo = max(1, (int)floor(BAND_DEFS[s].lowHz / freqStep));
      int binHi = min(maxBin-1, (int)ceil(BAND_DEFS[s].highHz / freqStep));
      if (binHi < binLo) continue;
      double bandSum=0; int bc=0;
      for (int b=binLo;b<=binHi;++b){ bandSum += vReal[b]; bc++; }
      if (bc>0) { float be = (float)(bandSum / (double)SAMPLES / (double)bc); acc[s]+=be; }
    }
    delay(12);
  }
  for (int s=0;s<NUM_STRIPS;s++) outAmbient[s] = max(acc[s] / (float)frames, 1e-6f);
  if (DEBUG) {
    Serial.print("Ambient noise: ");
    for (int s=0;s<NUM_STRIPS;s++) { Serial.printf("%.6f ", outAmbient[s]); }
    Serial.println();
  }
}


void doBeatBurst(const float bandEnergies[]) {
  for (int s = 0; s < NUM_STRIPS; ++s) {
    float denom = max(peakVal[s] - ambientNoise[s], 1e-6f);
    float adjusted = max(bandEnergies[s] - ambientNoise[s], 0.0f);
    float rawLevel = adjusted / denom;
    rawLevel = min(rawLevel, 6.0f);
    float boost = 2.2f;
    float level = constrain(boost * rawLevel, 0.0f, 1.0f);
    level = pow(level, 0.5f);
    drawSectionLevelNoShow(s, level);
  }
}


void exitModeCleanup(Mode m) {
  stripA.clear();
  stripB.clear();
  stripA.show();
  stripB.show();
  delay(20);
  for (int s=0;s<NUM_STRIPS;s++) {
    levelSmoothed[s] = 0.0f;
    peakVal[s] = max(ambientNoise[s], 1e-6f);
    rollingAvg[s] = ambientNoise[s];
  }
  for (int i=0;i<(SAMPLES/2); ++i) prevMagnitudes[i] = 0.0f;
  fluxMean = 1e-6f;
  rmsMean = 1e-6f;
  lastBeatMillis = 0;
  beatExpireMillis = 0;
  lastDetectedBeatStrength = 0.0f;
}


void initNormal() {
  for (int i=0;i<NUM_STRIPS;i++) {
    levelSmoothed[i] = 0.0f;
    peakVal[i] = max(ambientNoise[i], 1e-6f);
  }
  measureAmbientNoise(6, ambientNoise);
  for (int i=0;i<NUM_STRIPS;i++) peakVal[i] = max(ambientNoise[i], 1e-6f);
  if (DEBUG) Serial.println("initNormal: done");
}

void initBeat() {
  for (int i=0;i<NUM_STRIPS;i++) {
    levelSmoothed[i] = 0.0f;
    peakVal[i] = max(ambientNoise[i] * 2.0f, 1e-6f);
  }
  for (int i=0;i<(SAMPLES/2); ++i) prevMagnitudes[i] = 0.0f;
  fluxMean = 1e-6f; rmsMean = 1e-6f; lastBeatMillis = 0; beatExpireMillis=0;
  measureAmbientNoise(8, ambientNoise);
  for (int i=0;i<NUM_STRIPS;i++) peakVal[i] = max(ambientNoise[i] * 2.0f, 1e-6f);
  if (DEBUG) Serial.println("initBeat: done");
}

void initEmotion() {
  if (DEBUG) Serial.println("initEmotion: ready to display emotion visuals");
}

void initRoll() {
  if (DEBUG) Serial.println("initRoll: calibrating ambient...");
  measureAmbientNoise(12, ambientNoise);
  for (int s=0; s<NUM_STRIPS; ++s) {
    rollingAvg[s] = ambientNoise[s];
    peakVal[s] = max(ambientNoise[s] * 4.0f, ROLL_MIN_PEAK);
    levelSmoothed[s] = 0.0f;
  }
  if (DEBUG) {
    Serial.print("initRoll: done. ambient: ");
    for (int s=0;s<NUM_STRIPS;s++) Serial.printf("%.6f ", ambientNoise[s]);
    Serial.println();
  }
}


void switchMode(Mode newMode) {
  if (newMode == activeMode) {
    exitModeCleanup(activeMode);
    activeMode = IDLE;
    if (DEBUG) Serial.println("switchMode: toggled to IDLE");
    lcdMain.clear(); lcdMain.print("Mode: IDLE");
    return;
  }
  if (activeMode != IDLE) exitModeCleanup(activeMode);
  if (newMode == NORMAL) { initNormal(); activeMode = NORMAL; Serial.println("-> NORMAL"); lcdMain.clear(); lcdMain.print("Mode: NORMAL"); }
  else if (newMode == BEAT) { initBeat(); activeMode = BEAT; Serial.println("-> BEAT"); lcdMain.clear(); lcdMain.print("Mode: BEAT"); }
  else if (newMode == EMOTION) { initEmotion(); activeMode = EMOTION; Serial.println("-> EMOTION"); lcdMain.clear(); lcdMain.print("Mode: EMOTION"); }
  else if (newMode == ROLL) { initRoll(); activeMode = ROLL; Serial.println("-> ROLL"); lcdMain.clear(); lcdMain.print("Mode: ROLL"); }
  else { activeMode = IDLE; lcdMain.clear(); lcdMain.print("Mode: IDLE"); }
}


void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(BTN_NORMAL, INPUT_PULLUP);
  pinMode(BTN_BEAT, INPUT_PULLUP);
  pinMode(BTN_EMOTION, INPUT_PULLUP);
  pinMode(BTN_ROLL, INPUT_PULLUP);
  pinMode(BTN_AI, INPUT_PULLUP);
  pinMode(BTN_COLOR, INPUT_PULLUP); 

  strips[0] = &stripA;
  strips[1] = &stripB;

  computeSectionMapping();
  stripA.begin();
  stripB.begin();
  stripA.show();
  stripB.show();

  applyColorPalette(0);

  for (int i=0;i<NUM_STRIPS;i++){ ambientNoise[i]=1e-3f; peakVal[i]=1e-3f; levelSmoothed[i]=0.0f; rollingAvg[i]=1e-3f; }
  for (int i=0;i<(SAMPLES/2); ++i) prevMagnitudes[i] = 0.0f;

  delay(50);
  unsigned long now = millis();
  stableNormal = lastReadingNormal = digitalRead(BTN_NORMAL);
  stableBeat   = lastReadingBeat   = digitalRead(BTN_BEAT);
  stableEmotion  = lastReadingEmotion  = digitalRead(BTN_EMOTION);
  stableRoll   = lastReadingRoll   = digitalRead(BTN_ROLL);
  stableAI     = lastReadingAI     = digitalRead(BTN_AI);
  stableColor  = lastReadingColor  = digitalRead(BTN_COLOR);
  lastDebounceNormal = lastDebounceBeat = lastDebounceEmotion = lastDebounceRoll = lastDebounceAI = lastDebounceColor = now;

  i2sInitRX();
  Serial.println(F("I2S initialized. Calibrating ambient (play background audio)..."));
  measureAmbientNoise(30, ambientNoise);
  for (int i=0;i<NUM_STRIPS;i++) {
    peakVal[i] = max(ambientNoise[i], 1e-6f);
    Serial.printf("Ambient[%d]=%.6f\n", i, ambientNoise[i]);
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed - check partition scheme");
  } else {
    if (DEBUG) Serial.println("SPIFFS mounted");
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcdMain.init();
  lcdMain.backlight();
  lcdMain.clear();
  lcdMain.print("Ready");

  lcdAux.init();
  lcdAux.backlight();
  lcdAux.clear();
  lcdAux.print("Aux Ready");
  auxLastLine0 = "Aux Ready";
  auxLastLine1 = "";

  if (strlen(WIFI_SSID) > 0) {
    writeAuxLineIfChanged(0, "Connecting WiFi...");
    Serial.print("Connecting to WiFi ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 60) {
      delay(500);
      Serial.print(".");
      tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected");
      writeAuxLineIfChanged(0, "WiFi OK");
      configTime(0, 0, "pool.ntp.org", "time.google.com");
      Serial.print("Waiting for NTP");
      unsigned long t0 = millis();
      while (time(nullptr) < 1600000000) {
        delay(200); Serial.print(".");
        if (millis() - t0 > 15000) break;
      }
      Serial.println();
      Serial.printf("Time: %lu\n", (unsigned long)time(NULL));
    } else {
      Serial.println("WiFi failed");
      writeAuxLineIfChanged(0, "WiFi fail");
    }
  } else {
    Serial.println("No WIFI_SSID set — skipping WiFi/NTP in setup");
    writeAuxLineIfChanged(0, "No WiFi cfg");
  }

  welcomed = false;
  Serial.println("Setup complete - press a mode button to start");
}


void handleButtons() {
  int rN = digitalRead(BTN_NORMAL);
  if (rN != lastReadingNormal) lastDebounceNormal = millis();
  if ((millis() - lastDebounceNormal) > DEBOUNCE_MS) {
    if (rN != stableNormal) {
      if (rN == LOW && stableNormal == HIGH) {
        if (!welcomed) {
          welcomed = true;
          lcdMain.clear(); lcdMain.print("Welcome!");
          delay(100); 
          Serial.println("Welcome pressed -> playing welcome melody");
          playWelcomeMelody();
        }
        switchMode(NORMAL);
      }
      stableNormal = rN;
    }
  }
  lastReadingNormal = rN;

  int rB = digitalRead(BTN_BEAT);
  if (rB != lastReadingBeat) lastDebounceBeat = millis();
  if ((millis() - lastDebounceBeat) > DEBOUNCE_MS) {
    if (rB != stableBeat) {
      if (rB == LOW && stableBeat == HIGH) {
        if (!welcomed) { welcomed = true; lcdMain.clear(); lcdMain.print("Welcome!"); delay(100); playWelcomeMelody(); }
        switchMode(BEAT);
      }
      stableBeat = rB;
    }
  }
  lastReadingBeat = rB;

  int rEmotion = digitalRead(BTN_EMOTION);
  if (rEmotion != lastReadingEmotion) lastDebounceEmotion = millis();
  if ((millis() - lastDebounceEmotion) > DEBOUNCE_MS) {
    if (rEmotion != stableEmotion) {
      if (rEmotion == LOW && stableEmotion == HIGH) {
        if (!welcomed) { welcomed = true; lcdMain.clear(); lcdMain.print("Welcome!"); delay(100); playWelcomeMelody(); }
        switchMode(EMOTION);
      }
      stableEmotion = rEmotion;
    }
  }
  lastReadingEmotion = rEmotion;

  int rRoll = digitalRead(BTN_ROLL);
  if (rRoll != lastReadingRoll) lastDebounceRoll = millis();
  if ((millis() - lastDebounceRoll) > DEBOUNCE_MS) {
    if (rRoll != stableRoll) {
      if (rRoll == LOW && stableRoll == HIGH) {
        showStatsOnMain = !showStatsOnMain;
        if (showStatsOnMain) {
          lastStatsFlipMillis = millis();
          statsPageIndex = 0;
          lcdMain.clear(); lcdMain.print("Stats view");
          Serial.println("Main LCD: Stats ON");
        } else {
          if (activeMode == NORMAL) lcdMain.clear(), lcdMain.print("Mode: NORMAL");
          else if (activeMode == BEAT) lcdMain.clear(), lcdMain.print("Mode: BEAT");
          else if (activeMode == EMOTION) lcdMain.clear(), lcdMain.print("Mode: EMOTION");
          else if (activeMode == IDLE) lcdMain.clear(), lcdMain.print("Mode: IDLE");
          Serial.println("Main LCD: Stats OFF");
        }
      }
      stableRoll = rRoll;
    }
  }
  lastReadingRoll = rRoll;

  int rAI = digitalRead(BTN_AI);
  if (rAI != lastReadingAI) lastDebounceAI = millis();
  if ((millis() - lastDebounceAI) > DEBOUNCE_MS) {
    if (rAI != stableAI) {
      if (rAI == LOW && stableAI == HIGH) {
        if (!welcomed) {
          welcomed = true;
          lcdMain.clear(); lcdMain.print("Welcome!");
          delay(100); 
          playWelcomeMelody();
        }
        aiRequested = true;
        Serial.println("AI identify requested");
      }
      stableAI = rAI;
    }
  }
  lastReadingAI = rAI;

  int rC = digitalRead(BTN_COLOR);
  if (rC != lastReadingColor) lastDebounceColor = millis();
  if ((millis() - lastDebounceColor) > DEBOUNCE_MS) {
    if (rC != stableColor) {
      if (rC == LOW && stableColor == HIGH) {
        int next = (currentPaletteIndex + 1) % PALETTES;
        applyColorPalette(next);
        writeAuxLineIfChanged(0, String("Palette: ") + PALETTE_NAMES[next]);
        writeAuxLineIfChanged(1, String("Press to cycle"));
        auxRotateMillis = millis();
        if (DEBUG) Serial.printf("Palette button -> %d (%s)\n", next, PALETTE_NAMES[next]);
      }
      stableColor = rC;
    }
  }
  lastReadingColor = rC;
}


void runNormalTick() {
  takeSamplesI2S();

  double acc = 0;
  for (int i=0;i<SAMPLES;i++){ double s=vReal[i]; acc += s*s; }
  acc /= (double)SAMPLES;
  float rms = (float)sqrt(acc);
  lastRmsGlobal = rms;
  float rmsNorm = rms / I2S_MAX;

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  const int maxBin = SAMPLES/2;
  const float freqStep = (float)I2S_SAMPLE_RATE / (float)SAMPLES;

  int dominantBin=1; float dominantMag=0; double magAcc=0; int magCount=0;
  for (int b=1;b<maxBin;++b){ float m=vReal[b]; if (m>dominantMag){dominantMag=m;dominantBin=b;} magAcc+=(double)m; magCount++; }
  float meanMag = (magCount>0) ? (float)(magAcc/(double)magCount) : 0.0f;
  lastDominantFreqGlobal = dominantBin * freqStep;

  float avgVisualLevel = 0.0f;

  for (int s=0;s<NUM_STRIPS;++s) {
    int binLo = max(1,(int)ceil(BAND_DEFS[s].lowHz / freqStep));
    int binHi = min(maxBin-1,(int)floor(BAND_DEFS[s].highHz / freqStep));
    if (binHi < binLo) { drawSectionLevelNoShow(s, 0.0f); continue; }
    double bandSum=0; int bcount=0;
    for (int b=binLo;b<=binHi;++b){ bandSum += vReal[b]; ++bcount; }
    float bandEnergy = (bcount>0)?(float)(bandSum/(double)SAMPLES/(double)bcount):0.0f;

    bool strongSingleTone = (dominantMag > DOMINANT_PEAK_FACTOR * max(1e-12f, meanMag));
    if (strongSingleTone && !(dominantBin >= binLo && dominantBin <= binHi)) bandEnergy *= HARMONIC_ATTENUATION;

    const float AMBIENT_ADAPT = 0.995f;
    ambientNoise[s] = AMBIENT_ADAPT * ambientNoise[s] + (1.0f - AMBIENT_ADAPT) * bandEnergy;

    if (bandEnergy > peakVal[s]) peakVal[s] = bandEnergy;
    else peakVal[s] *= PEAK_DECAY;
    if (peakVal[s] < 1e-6f) peakVal[s] = 1e-6f;

    float adjusted = max(bandEnergy - ambientNoise[s], 0.0f);
    float denom = max(peakVal[s] - ambientNoise[s], 1e-6f);
    float levelF = adjusted / denom;
    levelF = pow(levelF, COMPRESS_EXP);
    float level = constrain(0.8f * levelF + 0.2f * rmsNorm, 0.0f, 1.0f);

    if (level > levelSmoothed[s]) levelSmoothed[s] = ATTACK_ALPHA * level + (1.0f - ATTACK_ALPHA) * levelSmoothed[s];
    else levelSmoothed[s] = DECAY_ALPHA * level + (1.0f - DECAY_ALPHA) * levelSmoothed[s];

    drawSectionLevelNoShow(s, levelSmoothed[s]);

    avgVisualLevel += levelSmoothed[s];
  }

  avgVisualLevel /= (float)NUM_STRIPS;
  lastAvgVisualLevelGlobal = avgVisualLevel;

  stripA.show();
  stripB.show();
}


void runBeatTick() {
  unsigned long loopStart = millis();

  takeSamplesI2S();

  double acc = 0;
  for (int i=0;i<SAMPLES;i++){ double s=vReal[i]; acc += s*s; }
  acc /= (double)SAMPLES;
  float rms = (float)sqrt(acc);
  lastRmsGlobal = rms;
  float rmsNorm = rms / I2S_MAX;

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  const int maxBin = SAMPLES / 2;
  const float freqStep = (float)I2S_SAMPLE_RATE / (float)SAMPLES;

  int dominantBin = 1; float dominantMag = 0; double meanMagAcc = 0; int magCount = 0;
  for (int b = 1; b < maxBin; ++b) { float m = vReal[b]; if (m > dominantMag) { dominantMag = m; dominantBin = b; } meanMagAcc += (double)m; ++magCount; }
  float meanMag = (magCount > 0) ? (float)(meanMagAcc / (double)magCount) : 0.0f;
  lastDominantFreqGlobal = dominantBin * freqStep;

  float bandEnergies[NUM_STRIPS] = {0.0f};
  float avgVisualLevel = 0.0f;

  for (int s = 0; s < NUM_STRIPS; ++s) {
    int binLo = max(1, (int)ceil(BAND_DEFS[s].lowHz / freqStep));
    int binHi = min(maxBin - 1, (int)floor(BAND_DEFS[s].highHz / freqStep));
    if (binHi < binLo) { bandEnergies[s] = 0.0f; continue; }
    double bandSum = 0.0; int binsCount = 0;
    for (int b = binLo; b <= binHi; ++b) { bandSum += vReal[b]; ++binsCount; }
    float bandEnergy = (binsCount > 0) ? (float)(bandSum / (double)SAMPLES / (double)binsCount) : 0.0f;

    bool strongSingleTone = (dominantMag > DOMINANT_PEAK_FACTOR * max(1e-12f, meanMag));
    if (strongSingleTone && !(dominantBin >= binLo && dominantBin <= binHi)) bandEnergy *= HARMONIC_ATTENUATION;

    const float AMBIENT_ADAPT = 0.995f;
    ambientNoise[s] = AMBIENT_ADAPT * ambientNoise[s] + (1.0f - AMBIENT_ADAPT) * bandEnergy;

    if (bandEnergy > peakVal[s]) peakVal[s] = bandEnergy; else peakVal[s] *= PEAK_DECAY;
    if (peakVal[s] < 1e-6f) peakVal[s] = 1e-6f;

    bandEnergies[s] = bandEnergy;
    float adjusted = max(bandEnergy - ambientNoise[s], 0.0f);
    float denom = max(peakVal[s] - ambientNoise[s], 1e-6f);
    float levelF = adjusted / denom;
    levelF = pow(levelF, BEAT_COMPRESS_EXP);

    float rmsClip = min(rmsNorm, 1.5f);
    float level = constrain((1.0f - RMS_MIX_WEIGHT) * levelF + RMS_MIX_WEIGHT * rmsClip, 0.0f, 1.0f);

    if (level > levelSmoothed[s]) levelSmoothed[s] = ATTACK_ALPHA * level + (1.0f - ATTACK_ALPHA) * levelSmoothed[s];
    else levelSmoothed[s] = DECAY_ALPHA * level + (1.0f - DECAY_ALPHA) * levelSmoothed[s];

    avgVisualLevel += levelSmoothed[s];
  }
  avgVisualLevel /= (float)NUM_STRIPS;
  lastAvgVisualLevelGlobal = avgVisualLevel;

  double flux = 0.0; int fluxBins = 0;
  for (int b = 1; b < maxBin; ++b) {
    float diff = vReal[b] - prevMagnitudes[b];
    if (diff > 0.0f) { flux += diff; ++fluxBins; }
    prevMagnitudes[b] = vReal[b];
  }
  float fluxVal = (fluxBins>0)?(float)(flux/(double)fluxBins):0.0f;
  fluxMean = FLUX_ALPHA * fluxMean + (1.0f - FLUX_ALPHA) * fluxVal;
  rmsMean = RMS_ALPHA * rmsMean + (1.0f - RMS_ALPHA) * rmsNorm;
  lastFluxGlobal = fluxVal;

  bool beatByFlux = (fluxVal > max(1e-9f, fluxMean * FLUX_THRESHOLD_FACTOR));
  bool beatByRms  = (rmsNorm > max(1e-9f, rmsMean * RMS_THRESHOLD_FACTOR));

  bool beatDetected = false;
  float beatStrength = 0.0f;
  if (beatByFlux) { beatDetected = true; beatStrength = max(beatStrength, (fluxVal / max(1e-9f, fluxMean))); }
  if (beatByRms)  { beatDetected = true; beatStrength = max(beatStrength, (rmsNorm / max(1e-9f, rmsMean))); }

  unsigned long now = millis();
  if (beatDetected && (now - lastBeatMillis > MIN_MS_BETWEEN_BEATS)) {
    lastBeatMillis = now;
    beatExpireMillis = now + BEAT_HOLD_MS;
    lastDetectedBeatStrength = beatStrength;
  }

  bool microBeatActive = ENABLE_MICRO_BEATS && !beatDetected &&
                         (rmsNorm > max(1e-9f, rmsMean * MICRO_BEAT_TRIGGER) ||
                          fluxVal > max(1e-9f, fluxMean * MICRO_BEAT_TRIGGER));

  if (now < beatExpireMillis) {
    float strengthBoost = constrain(1.0f + (lastDetectedBeatStrength - 1.0f) * 1.4f, 1.0f, 3.0f);
    float savedPeaks[NUM_STRIPS];
    for (int s=0;s<NUM_STRIPS;++s){ savedPeaks[s] = peakVal[s]; peakVal[s] *= strengthBoost; }
    doBeatBurst(bandEnergies);
    for (int s=0;s<NUM_STRIPS;++s) peakVal[s] = savedPeaks[s];
  } else {
    if (microBeatActive) {
      for (int s=0;s<NUM_STRIPS;++s) drawSectionLevelNoShow(s, levelSmoothed[s] * MICRO_BEAT_SCALE);
    } else {
      drawIdleSubdued();
    }
  }

  stripA.show();
  stripB.show();

  unsigned long loopDur = millis() - loopStart;
  if (loopDur < 10) delay(10 - loopDur);
}


// ========== EMOTION VISUAL EFFECTS ==========

void drawLoveHeart() {
  static float pulseTime = 0.0f;
  pulseTime += 0.08f;
  
  // Pulsing brightness
  float brightness = 0.5f + 0.5f * sin(pulseTime);
  
  // Heart shape pattern across 10x12 grid
  // Define heart shape (1 = lit, 0 = dark)
  const bool heartPattern[12][10] = {
    {0,0,0,0,0,0,0,0,0,0}, // row 0 (bottom)
    {0,0,0,0,0,0,0,0,0,0},
    {0,0,0,1,0,0,1,0,0,0},
    {0,0,1,1,1,1,1,1,0,0},
    {0,0,1,1,1,1,1,1,0,0},
    {0,0,1,1,1,1,1,1,0,0},
    {0,0,1,1,1,1,1,1,0,0},
    {0,0,1,1,1,1,1,1,0,0},
    {0,0,0,1,1,1,1,0,0,0},
    {0,0,0,0,1,1,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0}  // row 11 (top)
  };
  
  stripA.clear();
  stripB.clear();
  
  for (int s = 0; s < NUM_STRIPS; s++) {
    uint16_t nLeds = ledsPerSection[s];
    for (uint16_t led = 0; led < nLeds; led++) {
      if (led < 12 && heartPattern[led][s]) {
        uint8_t red = (uint8_t)(255 * brightness);
        uint8_t pink = (uint8_t)(100 * brightness);
        setPixelDirect(s, led, red, pink, pink);
      }
    }
  }
  
  stripA.show();
  stripB.show();
  delay(50);
}

void drawHappyWaves() {
  static float waveTime = 0.0f;
  waveTime += 0.15f;
  
  stripA.clear();
  stripB.clear();
  
  for (int s = 0; s < NUM_STRIPS; s++) {
    uint16_t nLeds = ledsPerSection[s];
    float xPos = (float)s / (float)(NUM_STRIPS - 1);
    
    for (uint16_t led = 0; led < nLeds; led++) {
      float yPos = (float)led / (float)(nLeds - 1);
      
      // Multiple sine waves for colorful effect
      float wave = sin(xPos * 6.0f + waveTime) * 0.3f + 
                   sin(yPos * 4.0f - waveTime * 1.5f) * 0.3f + 0.5f;
      
      // Rainbow colors
      float hue = fmod((xPos + yPos + waveTime * 0.3f) * 360.0f, 360.0f);
      float h = hue / 60.0f;
      float sat = 1.0f;
      float val = wave;
      
      float c = val * sat;
      float x_col = c * (1.0f - fabs(fmod(h, 2.0f) - 1.0f));
      
      float r = 0, g = 0, b = 0;
      if (h < 1) { r = c; g = x_col; }
      else if (h < 2) { r = x_col; g = c; }
      else if (h < 3) { g = c; b = x_col; }
      else if (h < 4) { g = x_col; b = c; }
      else if (h < 5) { b = c; r = x_col; }
      else { b = x_col; r = c; }
      
      setPixelDirect(s, led, (uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
    }
  }
  
  stripA.show();
  stripB.show();
  delay(50);
}

void drawSadRain() {
  static int raindrops[10][3]; // [strip][dropIndex] -> position
  static int rainSpeed[10][3];
  static bool initialized = false;
  
  if (!initialized) {
    for (int s = 0; s < 10; s++) {
      for (int d = 0; d < 3; d++) {
        raindrops[s][d] = random(0, 12);
        rainSpeed[s][d] = random(1, 3);
      }
    }
    initialized = true;
  }
  
  stripA.clear();
  stripB.clear();
  
  // Blue raindrop effect
  for (int s = 0; s < NUM_STRIPS; s++) {
    uint16_t nLeds = ledsPerSection[s];
    
    for (int d = 0; d < 3; d++) {
      int pos = raindrops[s][d];
      
      // Draw raindrop trail
      for (int trail = 0; trail < 3; trail++) {
        int ledPos = pos - trail;
        if (ledPos >= 0 && ledPos < nLeds) {
          float brightness = 1.0f - (trail * 0.3f);
          setPixelDirect(s, ledPos, 0, (uint8_t)(50 * brightness), (uint8_t)(255 * brightness));
        }
      }
      
      // Move raindrop down
      raindrops[s][d] -= rainSpeed[s][d];
      if (raindrops[s][d] < -3) {
        raindrops[s][d] = nLeds + random(0, 5);
        rainSpeed[s][d] = random(1, 3);
      }
    }
  }
  
  stripA.show();
  stripB.show();
  delay(80);
}

void drawAngryFire() {
  static float fireTime = 0.0f;
  fireTime += 0.25f; // Slightly faster for more movement
  
  stripA.clear();
  stripB.clear();
  
  for (int s = 0; s < NUM_STRIPS; s++) {
    uint16_t nLeds = ledsPerSection[s];
    
    for (uint16_t led = 0; led < nLeds; led++) {
      // INVERTED: LED 0 is at TOP, LED 11 is at BOTTOM
      float yPos = 1.0f - ((float)led / (float)(nLeds - 1));
      float xPos = (float)s / (float)(NUM_STRIPS - 1);
      
      // MORE chaotic flicker with multiple frequencies
      float flicker = sin(xPos * 12.0f + fireTime * 4.0f) * 0.4f +
                      sin(yPos * 10.0f + fireTime * 3.2f) * 0.3f +
                      sin((xPos + yPos) * 18.0f + fireTime * 5.0f) * 0.25f +
                      sin(xPos * 7.0f - fireTime * 2.8f) * 0.2f; // Extra turbulence
      
      // Bottom = brightest, Top = dimmer with MORE variation
      float baseIntensity = (yPos * 0.6f + 0.4f) * (0.6f + flicker * 0.4f);
      baseIntensity = constrain(baseIntensity, 0.0f, 1.0f);
      
      uint8_t red, green, blue;
      
      if (yPos > 0.85f) {
        // Bottom 15%: Orange-yellow core (small hot spot)
        red = (uint8_t)(255 * baseIntensity);
        green = (uint8_t)(150 * baseIntensity); // Less yellow, more orange
        blue = 0;
      }
      else if (yPos > 0.65f) {
        // Lower 20%: Red-orange transition
        red = (uint8_t)(255 * baseIntensity);
        green = (uint8_t)(80 * baseIntensity);
        blue = 0;
      }
      else {
        // Top 65%: MOSTLY RED with flickering
        red = (uint8_t)(255 * baseIntensity);
        // Dynamic green based on flicker - creates dancing effect
        float greenAmount = 30.0f + flicker * 40.0f;
        greenAmount = constrain(greenAmount, 0.0f, 70.0f);
        green = (uint8_t)(greenAmount * baseIntensity);
        blue = 0;
      }
      
      setPixelDirect(s, led, red, green, blue);
    }
  }
  
  stripA.show();
  stripB.show();
  delay(40); // Slightly faster refresh for more fluid movement
}

void runEmotionTick() {
  // Display visual based on detected emotion
  if (currentEmotion == EMOTION_LOVE) {
    drawLoveHeart();
  } else if (currentEmotion == EMOTION_HAPPY) {
    drawHappyWaves();
  } else if (currentEmotion == EMOTION_SAD) {
    drawSadRain();
  } else if (currentEmotion == EMOTION_ANGRY) {
    drawAngryFire();
  } else {
    // No emotion detected yet - show idle pattern
    static float idleTime = 0.0f;
    idleTime += 0.05f;
    stripA.clear();
    stripB.clear();
    for (int s = 0; s < NUM_STRIPS; s++) {
      float brightness = 0.2f + 0.1f * sin(idleTime + s * 0.5f);
      setPixelDirect(s, 5, (uint8_t)(100 * brightness), (uint8_t)(100 * brightness), (uint8_t)(100 * brightness));
    }
    stripA.show();
    stripB.show();
    delay(50);
  }
}


void runRollTick() {
  takeSamplesI2S();

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  const int maxBin = SAMPLES/2;
  const float freqStep = (float)I2S_SAMPLE_RATE / (float)SAMPLES;

  float avgVisualLevel = 0.0f;
  float rms = 0.0f;
  for (int i=0;i<SAMPLES;i++){ double s=vReal[i]; rms += s*s; }
  rms = sqrt(rms / (double)SAMPLES);
  lastRmsGlobal = rms;

  for (int s=0; s<NUM_STRIPS; ++s) {
    int binLo = max(1,(int)ceil(BAND_DEFS[s].lowHz / freqStep));
    int binHi = min(maxBin-1,(int)floor(BAND_DEFS[s].highHz / freqStep));
    if (binHi < binLo) { drawSectionLevelNoShow(s, 0.0f); continue; }

    double bandSum = 0; int bc=0;
    for (int b=binLo;b<=binHi;++b){ bandSum += vReal[b]; ++bc; }
    float bandEnergy = (bc>0)?(float)(bandSum/(double)SAMPLES/(double)bc):0.0f;

    rollingAvg[s] = ROLL_ALPHA * rollingAvg[s] + (1.0f - ROLL_ALPHA) * bandEnergy;

    if (rollingAvg[s] > peakVal[s]) peakVal[s] = rollingAvg[s];
    else peakVal[s] *= PEAK_DECAY;
    peakVal[s] = max(peakVal[s], max(ambientNoise[s] + ROLL_MIN_PEAK, ROLL_MIN_PEAK));

    float adjusted = max(rollingAvg[s] - ambientNoise[s], 0.0f);
    float denom = max(peakVal[s] - ambientNoise[s], ROLL_MIN_PEAK);
    float levelF = adjusted / denom;
    levelF = constrain(levelF * ROLL_DISPLAY_GAIN, 0.0f, 1.0f);
    levelF = pow(levelF, COMPRESS_EXP);

    if (levelF > levelSmoothed[s]) levelSmoothed[s] = ATTACK_ALPHA * levelF + (1.0f - ATTACK_ALPHA) * levelSmoothed[s];
    else levelSmoothed[s] = DECAY_ALPHA * levelF + (1.0f - DECAY_ALPHA) * levelSmoothed[s];

    drawSectionLevelNoShow(s, levelSmoothed[s]);

    avgVisualLevel += levelSmoothed[s];
  }

  avgVisualLevel /= (float)NUM_STRIPS;
  lastAvgVisualLevelGlobal = avgVisualLevel;

  stripA.show();
  stripB.show();
  delay(50);
}


void playWelcomeMelody() {
  const int notes[] = { 440, 660, 880, 660, 440 };
  const int dur = 160;
  for (int i=0;i< (int)(sizeof(notes)/sizeof(notes[0])); ++i) {
    tone(SPEAKER_PIN, notes[i]);
    delay(dur);
    noTone(SPEAKER_PIN);
    delay(30);
  }
}


#define CHANNELS 1
#define BITS_PER_SAMPLE 16
#define AUDIO_FMT_PCM 1
const size_t CHUNK_READ_SIZE = 2048;
static uint8_t readBuf[CHUNK_READ_SIZE];
static uint8_t pcmBuf[CHUNK_READ_SIZE];

void write_wav_header_placeholder(File &f) {
  uint8_t header[44] = {0};
  header[0]= 'R'; header[1]= 'I'; header[2]= 'F'; header[3]= 'F';
  header[8]= 'W'; header[9]= 'A'; header[10]= 'V'; header[11]= 'E';
  header[12]= 'f'; header[13]= 'm'; header[14]= 't'; header[15]= ' ';
  uint32_t subchunk1Size = 16;
  memcpy(header+16, &subchunk1Size, 4);
  uint16_t audioFormat = AUDIO_FMT_PCM;
  memcpy(header+20, &audioFormat, 2);
  uint16_t numChannels = CHANNELS;
  memcpy(header+22, &numChannels, 2);
  uint32_t sampleRate = SAMPLE_RATE;
  memcpy(header+24, &sampleRate, 4);
  uint16_t bitsPerSample = BITS_PER_SAMPLE;
  uint16_t byteDepth = bitsPerSample / 8;
  uint32_t byteRate = SAMPLE_RATE * CHANNELS * byteDepth;
  memcpy(header+28, &byteRate, 4);
  uint16_t blockAlign = CHANNELS * byteDepth;
  memcpy(header+32, &blockAlign, 2);
  memcpy(header+34, &bitsPerSample, 2);
  header[36]= 'd'; header[37]= 'a'; header[38]= 't'; header[39]= 'a';
  f.write(header, 44);
}

bool finalize_wav_header(const char* path) {
  File f = SPIFFS.open(path, "r+");
  if (!f) return false;
  size_t fileSize = f.size();
  if (fileSize < 44) { f.close(); return false; }
  uint32_t subchunk2Size = (uint32_t)(fileSize - 44);
  uint32_t chunkSize = 36 + subchunk2Size;
  f.seek(4);
  f.write((const uint8_t*)&chunkSize, 4);
  f.seek(40);
  f.write((const uint8_t*)&subchunk2Size, 4);
  f.close();
  return true;
}

size_t convert_i2s32_to_pcm16(uint8_t* dest, const uint8_t* src, size_t srcBytes) {
  size_t out = 0;
  for (size_t i = 0; i + 3 < srcBytes; i += 4) {
    int32_t s32 = ((int32_t)src[i+3] << 24) | ((int32_t)src[i+2] << 16) | ((int32_t)src[i+1] << 8) | (int32_t)src[i];
    int16_t s16 = (int16_t)(s32 >> 16);
    dest[out++] = (uint8_t)(s16 & 0xFF);
    dest[out++] = (uint8_t)((s16 >> 8) & 0xFF);
  }
  return out;
}

bool recordAndSaveWav(const char* path) {
  File f = SPIFFS.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("Failed to open WAV for writing");
    return false;
  }
  write_wav_header_placeholder(f);

  size_t totalBytesToRead = (size_t)SAMPLE_RATE * (size_t)CAPTURE_SECONDS * 4; 
  size_t bytesReadTotal = 0;
  size_t bytesReadNow = 0;

  while (bytesReadTotal < totalBytesToRead) {
    size_t toRead = min(CHUNK_READ_SIZE, totalBytesToRead - bytesReadTotal);
    esp_err_t r = i2s_read(I2S_NUM, readBuf, toRead, &bytesReadNow, portMAX_DELAY);
    if (r != ESP_OK) {
      Serial.printf("i2s_read err %d\n", r);
      f.close();
      return false;
    }
    if (bytesReadNow == 0) {
      Serial.println("i2s_read returned 0");
      f.close();
      return false;
    }
    size_t pcmBytes = convert_i2s32_to_pcm16(pcmBuf, readBuf, bytesReadNow);
    size_t written = f.write(pcmBuf, pcmBytes);
    if (written != pcmBytes) {
      Serial.println("Write failed");
      f.close();
      return false;
    }
    bytesReadTotal += bytesReadNow;
  }
  f.close();

  if (!finalize_wav_header(path)) {
    Serial.println("Failed to finalize WAV header");
    return false;
  }
  return true;
}


String hmacSha1Base64(const char* key, const char* message) {
  unsigned char hmac[20];
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
  if (!info) return String();

  int ret = mbedtls_md_hmac(info,
                           (const unsigned char*)key, strlen(key),
                           (const unsigned char*)message, strlen(message),
                           hmac);
  if (ret != 0) {
    Serial.printf("mbedtls_md_hmac err %d\n", ret);
    return String();
  }

  size_t olen = 0;
  unsigned char b64[64];
  ret = mbedtls_base64_encode(b64, sizeof(b64), &olen, hmac, sizeof(hmac));
  if (ret != 0) {
    Serial.printf("mbedtls_base64_encode err %d\n", ret);
    return String();
  }
  String result = String((char*)b64).substring(0, (int)olen);
  result.replace("\r", "");
  result.replace("\n", "");
  return result;
}

void debugACRSignature(const String& string_to_sign) {
  Serial.println("======================");
  Serial.println("=== ACR SIGN DEBUG ===");
  Serial.println("String to sign:");
  Serial.println(string_to_sign);
  Serial.print("Bytes: ");
  for (size_t i=0; i<string_to_sign.length(); i++) {
    Serial.printf("%02X ", (uint8_t)string_to_sign[i]);
  }
  Serial.println();
}

String acrIdentifyFromFile(const char* wavPath) {
  File f = SPIFFS.open(wavPath, FILE_READ);
  if (!f) {
    Serial.println("acrIdentifyFromFile: failed to open wav file");
    return String();
  }
  size_t fsize = f.size();

  const String http_method = "POST";
  const String uri = "/v1/identify";
  const String data_type = "audio";
  const String signature_version = "1";

  time_t ts = time(nullptr);
  if (ts < 1000000000) {
    Serial.println("Time not synced! Waiting for NTP...");
    unsigned long waitStart = millis();
    
    while (time(nullptr) < 1000000000 && millis() - waitStart < 15000) {
      delay(200);
    }
    ts = time(nullptr);
  }
  String timestamp = String((long)ts);

  String string_to_sign = http_method + "\n" +
                          uri + "\n" +
                          String(ACR_ACCESS_KEY) + "\n" +
                          data_type + "\n" +
                          signature_version + "\n" +
                          timestamp;

  debugACRSignature(string_to_sign);

  String signature = hmacSha1Base64(ACR_ACCESS_SECRET, string_to_sign.c_str());

  Serial.println("=== ACR SIGN DEBUG (final) ===");
  Serial.print("Timestamp: "); Serial.println(timestamp);
  Serial.print("Signature: "); Serial.println(signature);
  Serial.println("==============================");

  String boundary = "----ESP32BOUNDARY";
  auto addField = [&](const char* name, const String& val) {
    String s = "--" + boundary + "\r\n";
    s += "Content-Disposition: form-data; name=\"" + String(name) + "\"\r\n\r\n";
    s += val + "\r\n";
    return s;
  };

  String preamble = "";
  preamble += addField("access_key", ACR_ACCESS_KEY);
  preamble += addField("sample_bytes", String(fsize));
  preamble += addField("timestamp", timestamp);
  preamble += addField("signature", signature);
  preamble += addField("data_type", data_type);
  preamble += addField("signature_version", signature_version);

  String filePartHeader = "--" + boundary + "\r\n";
  filePartHeader += "Content-Disposition: form-data; name=\"sample\"; filename=\"sample.wav\"\r\n";
  filePartHeader += "Content-Type: audio/wav\r\n\r\n";

  String ending = "\r\n--" + boundary + "--\r\n";

  size_t contentLength = preamble.length() + filePartHeader.length() + fsize + ending.length();

  String host = String(ACR_HOST);
  String request = "";
  request += String("POST ") + uri + " HTTP/1.1\r\n";
  request += "Host: " + host + "\r\n";
  request += "User-Agent: ESP32\r\n";
  request += "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n";
  request += "Content-Length: " + String(contentLength) + "\r\n";
  request += "Connection: close\r\n\r\n";

  WiFiClientSecure client;
  client.setInsecure(); 
  if (!client.connect(host.c_str(), 443)) {
    Serial.println("TLS connect failed");
    f.close();
    return String();
  }

  client.print(request);
  client.print(preamble);
  client.print(filePartHeader);

  const size_t bufSize = 1024;
  uint8_t buf[bufSize];
  f.seek(0);
  while (f.available()) {
    size_t r = f.read(buf, bufSize);
    if (r == 0) break;
    client.write(buf, r);
  }
  client.print(ending);

  String response = "";
  unsigned long timeout = millis() + 20000;
  while (!client.available() && millis() < timeout) { delay(10); }
  while (client.available()) {
    response += client.readStringUntil('\n');
    response += "\n";
  }

  client.stop();
  f.close();

  int idx = response.indexOf("\r\n\r\n");
  if (idx >= 0) {
    String body = response.substring(idx + 4);
    Serial.println("ACR Response body:");
    Serial.println(body);
    return body;
  } else {
    Serial.println("ACR Response (no header/body split):");
    Serial.println(response);
    return response;
  }
}

String createGroqPayload(String song) {
  String payload = "{\n";
  payload += "  \"model\": \"llama-3.3-70b-versatile\",\n";
  payload += "  \"messages\": [\n";
  payload += "    {\n";
  payload += "      \"role\": \"system\",\n";
  payload += "      \"content\": \"You are a music emotion classifier. Classify songs into exactly one of these four emotions: LOVE, HAPPY, SAD, or ANGRY. Respond with only the emotion word, nothing else.\"\n";
  payload += "    },\n";
  payload += "    {\n";
  payload += "      \"role\": \"user\",\n";
  payload += "      \"content\": \"Classify the emotion of this song into one word (LOVE, HAPPY, SAD, or ANGRY): \\\"" 
             + song + "\\\"\"\n";
  payload += "    }\n";
  payload += "  ],\n";
  payload += "  \"max_tokens\": 10,\n";
  payload += "  \"temperature\": 0.3\n";
  payload += "}";
  return payload;
}

String askGroqForEmotion(String prompt) {
    String body = createGroqPayload(prompt);

    HTTPClient http;
    http.begin("https://api.groq.com/openai/v1/chat/completions");
    http.addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(body);
    String response = http.getString();
    http.end();

    if (httpResponseCode < 0) {
      Serial.println("Groq HTTP failed");
      return String("NONE");
    }

    DynamicJsonDocument doc(16384);
    DeserializationError err = deserializeJson(doc, response);
    if (err) {
      Serial.print("JSON parse failed: "); Serial.println(err.c_str());
      return String("NONE");
    }
    String reply = "";
    if (doc.containsKey("choices") && doc["choices"].size() > 0 &&
        doc["choices"][0].containsKey("message") &&
        doc["choices"][0]["message"].containsKey("content")) {
      reply = String((const char*)doc["choices"][0]["message"]["content"]);
      reply.trim();
      reply.toUpperCase();
    }
    if (reply.length()==0) return String("NONE");
    
    // Parse emotion
    if (reply.indexOf("LOVE") >= 0) return String("LOVE");
    if (reply.indexOf("HAPPY") >= 0) return String("HAPPY");
    if (reply.indexOf("SAD") >= 0) return String("SAD");
    if (reply.indexOf("ANGRY") >= 0) return String("ANGRY");
    
    return String("NONE");
}

String parseAcrTitleArtist(const String& acrResp) {
  acrTitle = "";
  acrArtist = "";
  acrCombined = "";
  if (acrResp.length() == 0) return String();
  String jsonStr = acrResp;
  jsonStr.trim();
  int firstBrace = jsonStr.indexOf('{');
  int lastBrace = jsonStr.lastIndexOf('}');
  if (firstBrace < 0 || lastBrace < 0 || lastBrace <= firstBrace) return String();
  jsonStr = jsonStr.substring(firstBrace, lastBrace + 1);
  StaticJsonDocument<12288> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) { Serial.print("JSON parse err: "); Serial.println(err.c_str()); return String(); }
  if (!doc.containsKey("metadata")) return String();
  JsonObject metadata = doc["metadata"];
  if (!metadata.containsKey("music")) return String();
  JsonArray musicArr = metadata["music"];
  if (musicArr.size() == 0) return String();
  JsonObject firstSong = musicArr[0];
  const char* title = firstSong["title"] | "";
  const char* artist = "";
  if (firstSong.containsKey("artists")) {
    JsonArray artistsArr = firstSong["artists"];
    if (artistsArr.size() > 0) {
      JsonObject firstArtist = artistsArr[0];
      artist = firstArtist["name"] | "";
    }
  }
  if (strlen(title) == 0 && strlen(artist) == 0) return String();
  acrTitle = String(title);
  acrArtist = String(artist);
  acrCombined = acrTitle;
  if (acrArtist.length() > 0) acrCombined += " - " + acrArtist;
  return acrCombined;
}


void writeAuxLineIfChanged(uint8_t row, const String &text) {
  String out = text;
  
  if (out.length() > 16) out = out.substring(0, 16);
  while (out.length() < 16) out += " ";

  unsigned long now = millis();
  if (row == 0) {
    if (auxLastLine0 != out || (now - lastAuxWriteMs) > 1000) {
      auxLastLine0 = out;
      lcdAux.setCursor(0,0);
      lcdAux.print(out);
      lastAuxWriteMs = now;
    }
  } else {
    if (auxLastLine1 != out || (now - lastAuxWriteMs) > 1000) {
      auxLastLine1 = out;
      lcdAux.setCursor(0,1);
      lcdAux.print(out);
      lastAuxWriteMs = now;
    }
  }
}


void updateMainLCDForModeOrStats() {
  if (showStatsOnMain) {
    unsigned long now = millis();
    if (now - lastStatsFlipMillis > STATS_PAGE_INTERVAL_MS) {
      lastStatsFlipMillis = now;
      statsPageIndex = (statsPageIndex + 1) % 4; 
    }

    char buf1[17], buf2[17];
    
    if (statsPageIndex == 0) {
      float rmsDisplay = lastRmsGlobal;
      float avgL = lastAvgVisualLevelGlobal;
      snprintf(buf1, sizeof(buf1), "RMS:%5.0f AVG:%0.2f", rmsDisplay, avgL);
      snprintf(buf2, sizeof(buf2), "Mode:%-6s", (activeMode==NORMAL)?"NORMAL":(activeMode==BEAT)?"BEAT":(activeMode==EMOTION)?"EMOTION":"IDLE");
    }
    else if (statsPageIndex == 1) {
      float domHz = lastDominantFreqGlobal;
      float flux = lastFluxGlobal;
      snprintf(buf1, sizeof(buf1), "DomHz:%4.0f", domHz);
      snprintf(buf2, sizeof(buf2), "Flux:%0.3f  ", flux);
    }
    else if (statsPageIndex == 2) {
      int topIdx = 0;
      float topVal = 0.0f;
      for (int i=0;i<NUM_STRIPS;i++){
        if (peakVal[i] > topVal) { topVal = peakVal[i]; topIdx = i; }
      }
      snprintf(buf1, sizeof(buf1), "TopBand:%d Peak:%0.2f", topIdx, topVal);
      snprintf(buf2, sizeof(buf2), "AvgVis:%0.2f", lastAvgVisualLevelGlobal);
    }
    else {
      const char* emotionStr = (acrEmotion.length() ? acrEmotion.c_str() : "(none)");
      snprintf(buf1, sizeof(buf1), "Emotion:%-7s", emotionStr);
      snprintf(buf2, sizeof(buf2), "RMS:%4.0f D:%04.0f", lastRmsGlobal, lastDominantFreqGlobal);
    }

    lcdMain.clear();
    lcdMain.setCursor(0,0);
    lcdMain.print(buf1);
    lcdMain.setCursor(0,1);
    lcdMain.print(buf2);
  } else {
    if (!welcomed) {
      lcdMain.clear(); lcdMain.print("Ready");
    } else {
      if (activeMode == NORMAL) { lcdMain.clear(); lcdMain.print("Mode: NORMAL"); }
      else if (activeMode == BEAT) { lcdMain.clear(); lcdMain.print("Mode: BEAT"); }
      else if (activeMode == EMOTION) { 
        lcdMain.clear(); 
        String emotionText = "EMOTION: ";
        if (currentEmotion == EMOTION_LOVE) emotionText += "LOVE";
        else if (currentEmotion == EMOTION_HAPPY) emotionText += "HAPPY";
        else if (currentEmotion == EMOTION_SAD) emotionText += "SAD";
        else if (currentEmotion == EMOTION_ANGRY) emotionText += "ANGRY";
        else emotionText += "...";
        lcdMain.print(emotionText);
      }
      else if (activeMode == IDLE) { lcdMain.clear(); lcdMain.print("Mode: IDLE"); }
      else { lcdMain.clear(); lcdMain.print("Mode"); }
    }
  }
}

void updateAuxLCDPeriodic() {
  unsigned long now = millis();
  if (aiState == AI_IDLE) {
    return;
  } else if (aiState == AI_WIFI) {
    writeAuxLineIfChanged(0, "WiFi reconnect...");
    writeAuxLineIfChanged(1, "");
  } else if (aiState == AI_RECORDING) {
    writeAuxLineIfChanged(0, "Recording...");
    writeAuxLineIfChanged(1, "");
  } else if (aiState == AI_UPLOADING) {
    writeAuxLineIfChanged(0, "Uploading...");
    writeAuxLineIfChanged(1, "");
  } else if (aiState == AI_PROCESSING) {
    writeAuxLineIfChanged(0, "Processing...");
    writeAuxLineIfChanged(1, "");
  } else if (aiState == AI_FAIL) {
    writeAuxLineIfChanged(0, "AI Fail / No Match");
    writeAuxLineIfChanged(1, "");
  } else if (aiState == AI_RESULT) {
    if (now - auxRotateMillis > AUX_ROTATE_INTERVAL_MS_LOCAL) {
      auxRotateMillis = now;
      auxRotateIndex = (auxRotateIndex + 1) % 3;
    }
    if (auxRotateIndex == 0) {
      String t = acrTitle.length()?acrTitle:"(unknown)";
      if (t.length() > 16) t = t.substring(0,16);
      writeAuxLineIfChanged(0, "Title:");
      writeAuxLineIfChanged(1, t);
    } else if (auxRotateIndex == 1) {
      String a = acrArtist.length()?acrArtist:"(unknown)";
      if (a.length() > 16) a = a.substring(0,16);
      writeAuxLineIfChanged(0, "Artist:");
      writeAuxLineIfChanged(1, a);
    } else {
      String em = acrEmotion.length()?acrEmotion:"(unknown)";
      if (em.length() > 16) em = em.substring(0,16);
      writeAuxLineIfChanged(0, "Emotion:");
      writeAuxLineIfChanged(1, em);
    }
  }
}

void loop() {
  handleButtons();

  static unsigned long lastMainLCDUpdate = 0;
  if (millis() - lastMainLCDUpdate > 500) {
    lastMainLCDUpdate = millis();
    updateMainLCDForModeOrStats();
  }

  updateAuxLCDPeriodic();

  if (aiRequested && !aiBusy) {
    aiBusy = true;
    aiRequested = false;

    if (strlen(WIFI_SSID) > 0 && WiFi.status() != WL_CONNECTED) {
      aiState = AI_WIFI;
      writeAuxLineIfChanged(0, "WiFi reconnect...");
      Serial.println("WiFi not connected - attempting reconnect...");
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      unsigned long t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print("."); }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Reconnected WiFi");
        writeAuxLineIfChanged(0, "WiFi OK");
        configTime(0,0,"pool.ntp.org","time.google.com");
        unsigned long t1 = millis();
        while (time(nullptr) < 1600000000 && millis() - t1 < 15000) delay(200);
      } else {
        Serial.println("WiFi reconnect failed");
        writeAuxLineIfChanged(0, "WiFi fail");
delay(1200);
}
}
aiState = AI_RECORDING;
writeAuxLineIfChanged(0, "Recording...");
writeAuxLineIfChanged(1, "");
Serial.println("AI: recording...");

if (!SPIFFS.begin()) {
  Serial.println("SPIFFS not mounted (retrying)");
  SPIFFS.begin(true); 
}

bool ok = recordAndSaveWav(WAV_PATH);
if (!ok) {
  Serial.println("Record fail");
  writeAuxLineIfChanged(0, "Rec fail");
  writeAuxLineIfChanged(1, "");
  delay(2000);
  aiBusy = false;
  aiState = AI_FAIL;
  return;
}

aiState = AI_UPLOADING;
writeAuxLineIfChanged(0, "Uploading...");
writeAuxLineIfChanged(1, "");
Serial.println("Uploading to ACRCloud...");
String acrResp = acrIdentifyFromFile(WAV_PATH);
String titleArtist = parseAcrTitleArtist(acrResp);
if (titleArtist.length() == 0) {
  writeAuxLineIfChanged(0, "No match");
  writeAuxLineIfChanged(1, "");
  Serial.println("No match");
  delay(2000);
  aiBusy = false;
  aiState = AI_FAIL;
  return;
}

Serial.println("Identified: " + titleArtist);

writeAuxLineIfChanged(0, "Asking AI...");
writeAuxLineIfChanged(1, "");
aiState = AI_PROCESSING;
delay(200);

String emotion = askGroqForEmotion(titleArtist);
if (emotion.length() == 0 || emotion == "NONE") {
  Serial.println("AI emotion fail");
  writeAuxLineIfChanged(0, "AI fail");
  writeAuxLineIfChanged(1, "");
  delay(2000);
  aiBusy = false;
  aiState = AI_FAIL;
  return;
}

acrEmotion = emotion;
Serial.println("AI emotion result: " + emotion);

// Set the current emotion for display
if (emotion == "LOVE") currentEmotion = EMOTION_LOVE;
else if (emotion == "HAPPY") currentEmotion = EMOTION_HAPPY;
else if (emotion == "SAD") currentEmotion = EMOTION_SAD;
else if (emotion == "ANGRY") currentEmotion = EMOTION_ANGRY;
else currentEmotion = EMOTION_NONE;

// If in EMOTION mode, automatically switch to display
if (activeMode != EMOTION) {
  switchMode(EMOTION);
}

aiState = AI_RESULT;
auxRotateIndex = 0;
auxRotateMillis = millis();

if (!showStatsOnMain) updateMainLCDForModeOrStats();
delay(3000);
aiBusy = false;
}
if (activeMode == NORMAL) runNormalTick();
else if (activeMode == BEAT) runBeatTick();
else if (activeMode == EMOTION) runEmotionTick();
else if (activeMode == ROLL) runRollTick();
else {
delay(40);
}
}