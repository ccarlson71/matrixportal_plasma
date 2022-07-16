#include <Adafruit_Protomatter.h>
#include <math.h>

#define HEIGHT  32 // Matrix height (pixels) - SET TO 64 FOR 64x64 MATRIX!
#define WIDTH   64 // Matrix width (pixels)
#define MAX_FPS 45 // Maximum redraw rate, frames/second
#define SCALE_BRIGHTNESS 0.2
#define BLACK { 0.0, 0.0, 0.0 }
#define RANDOM { random_byte(), random_byte(), random_byte() }
#define RED { 255, 0, 0 }
#define BLUE { 0, 0, 255 }
#define GREEN { 0, 255, 0 }

uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
uint8_t addrPins[] = {17, 18, 19, 20};
uint8_t clockPin   = 14;
uint8_t latchPin   = 15;
uint8_t oePin      = 16;

double current_value = 0.0;
double step_value = 0.3;
double param_1;

Adafruit_Protomatter matrix(
  64, 6, 1, rgbPins, 4, addrPins, clockPin, latchPin, oePin, true); // true indicates double-buffering

uint16_t palette[240];

double sin_lut[360];
double cos_lut[360];
double sqrt_lut[5120];

int reset_counter;
#define RESET_THRESHOLD 10000

// RANDOM SEED STUFF
// 99.9% from here: https://forum.arduino.cc/t/best-way-of-random-seed/640617/8
const byte analogPort = A1;

const int SEED_BUCKET_SIZE = 64;
int seed_buckets[SEED_BUCKET_SIZE];

long getRandomSeed(int numBits)
{
  // magic numbers tested 2016-03-28
  // try to speed it up
  // Works Well. Keep!
  //
  if (numBits > 31 or numBits <1) numBits = 31; // limit input range
 
  const int baseIntervalMs = 1UL; // minumum wait time
  const byte sampleSignificant = 7;  // modulus of the input sample
  const byte sampleMultiplier = 10;   // ms per sample digit difference

  const byte hashIterations = 3;
  int intervalMs = 0;

  unsigned long reading;
  long result = 0;
  int tempBit = 0;

  Serial.print("randomizing...");
  pinMode(analogPort, INPUT_PULLUP);
  pinMode(analogPort, INPUT);
  delay(200);
  // Now there will be a slow decay of the voltage,
  // about 8 seconds
  // so pick a point on the curve
  // offset by the processed previous sample:
  //

  for (int bits = 0; bits < numBits; bits++)
  {
      Serial.print('*');

    for (int i = 0; i < hashIterations; i++)
    {
//      Serial.print(' ');
//      Serial.print( hashIterations - i );
      delay(baseIntervalMs + intervalMs);

      // take a sample
      reading = analogRead(analogPort);
      tempBit ^= reading & 1;

      // take the low "digits" of the reading
      // and multiply it to scale it to
      // map a new point on the decay curve:
      intervalMs = (reading % sampleSignificant) * sampleMultiplier;
    }
    result |= (long)(tempBit & 1) << bits;
  }
  Serial.println(result);
//  Serial.println();
  return result;
}

void setup(void) {
  Serial.begin(9600);

  // Initialize matrix...
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  if(status != PROTOMATTER_OK) {
    for(;;);
  }

  // Initialize sine LUT
  for(int x=0;x<360;x++) {
    float x_rads = (x * 71) / 4068.0;
    sin_lut[x] = sin(x_rads);
  }

  // Initialize cosine LUT
  for(int x=0;x<360;x++) {
    float x_rads = (x * 71) / 4068.0;
    cos_lut[x] = cos(x_rads);
  }
  
  // Initialize sqrt LUT
  for(int x=0;x<64;x++) {
    for(int y=0;y<32;y++) {
      int idx = (y*64) + x;
      double dy = double(y) / 2.0;
      sqrt_lut[idx] = sqrt((x*x) + (dy*dy));
    }
  }
  randomize();
  reset_counter = 0;
}

void randomize() {
  randomize_seed();
  randomize_palette();
  randomize_location();
  param_1 = 10 + (random(750) / 50.0);
}

void randomize_seed() {
  randomSeed(getRandomSeed(31));
}

void randomize_location() {
  current_value = random(100000);
}

double sin_l(double rads) {
  int degs = int(rads * 57296 / 1000);
  int m_value = degs % 360;
  return sin_lut[m_value];
}

double cos_l(double rads) {
  int degs = int(rads * 57296 / 1000);
  int m_value = degs % 360;
  return cos_lut[m_value];
}

double random_byte(void) {
  return double(random(255) * SCALE_BRIGHTNESS);
}

void randomize_palette(void) {
  double anchors[][3] = {
    RANDOM, BLACK,
    RANDOM, RANDOM,
    RANDOM, BLACK
  };

  int palette_steps = 40;
  double deg_smoothing_step = 90.0 / palette_steps;
  
  for(int anchor_idx=0;anchor_idx<6;anchor_idx++) {
    int start_idx = anchor_idx * palette_steps;
    int stop_idx = start_idx + palette_steps;
    int to_idx = anchor_idx + 1;

    if (to_idx == 6) {
      to_idx = 1;
    }
    
    for(int i=start_idx;i<stop_idx;i++) {
      int j = i % palette_steps;
      double degs = j * deg_smoothing_step;
      double rads = (degs * 71) / 4068.0;
      double f = sin(rads);
      
      palette[i] = interpolate(anchors[anchor_idx], anchors[to_idx], f);
    }
  }
}

uint16_t interpolate(double from[3], double to[3], double f) {
  return matrix.color565(
    int((from[0] + ((to[0] - from[0]) * f))),
    int((from[1] + ((to[1] - from[1]) * f))),
    int((from[2] + ((to[2] - from[2]) * f)))
  );
}

// Uses lookup tables for lots of stuff
void fast_plasma(double seed_value) {
  
  for(int x=0; x<WIDTH; x++) {
    for(int y=0; y<HEIGHT; y++) {
      double dx = double(x);
      double dy = double(y) / 2.0;

      double calculated_value = sin_l((dx + seed_value) / param_1) + 
                                cos_l((dy + (seed_value / 2.0)) / 9.0) +
                                // sin_l(dy / 9.0) + 
                                sin_l(((dx + seed_value) + (dy - seed_value)) / 25.0) + 
                                sin_l(sqrt_lut[(y*64) + x] / 8.0);
                                // sin_l(sqrt((dx*dx) + (dy*dy)) / 8.0);
      int color_index = int((calculated_value + 4.0) * 30);
      int output_color = color_index % 240;

      matrix.drawPixel(x, y, palette[output_color]);
    }
  }
}
void loop(void) {
  matrix.fillScreen(0x0);
  fast_plasma(current_value);
  matrix.show();
  current_value += step_value;

  reset_counter += 1;
  if (reset_counter == RESET_THRESHOLD) {
    reset_counter = 0;
    randomize();
  }
}
