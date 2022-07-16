#include <Adafruit_Protomatter.h>
#include <math.h>

#define HEIGHT  32 // Matrix height (pixels) - SET TO 64 FOR 64x64 MATRIX!
#define WIDTH   64 // Matrix width (pixels)
#define MAX_FPS 45 // Maximum redraw rate, frames/second
#define SCALE_BRIGHTNESS 0.2
#define BLACK { 0, 0, 0 }
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

int random_byte(void) {
  return int(random(255) * SCALE_BRIGHTNESS);
}

void randomize_palette(void) {
  int c1[] = RANDOM;
  int c2[] = BLACK;
  int c3[] = RANDOM;
  int c4[] = BLACK;
  int c5[] = RANDOM;
  int c6[] = BLACK;
  
  // TO DO: Add easing between palette anchors
  for(int i=0;i<40;i++) {
    double f = double(i) / 40.0;
    palette[i] = interpolate(c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], f);
  }
  for(int i=40;i<80;i++) {
    double f = (double(i)-40.) / 40.0;
    palette[i] = interpolate(c2[0], c2[1], c2[2], c3[0], c3[1], c3[2], f);
  }
  for(int i=80;i<120;i++) {
    double f = (double(i)-80.0) / 40.0;
    palette[i] = interpolate(c3[0], c3[1], c3[2], c4[0], c4[1], c4[2], f);
  }
  for(int i=120;i<160;i++) {
    double f = (double(i)-120.0) / 40.0;
    palette[i] = interpolate(c4[0], c4[1], c4[2], c5[0], c5[1], c5[2], f);
  }
  for(int i=160;i<200;i++) {
    double f = (double(i)-160.0) / 40.0;
    palette[i] = interpolate(c5[0], c5[1], c5[2], c6[0], c6[1], c6[2], f);
  }
  for(int i=200;i<240;i++) {
    double f = (double(i)-200.0) / 40.0;
    palette[i] = interpolate(c6[0], c6[1], c6[2], c1[0], c1[1], c1[2], f);
  }

}

uint16_t interpolate(int r1, int g1, int b1, int r2, int g2, int b2, double f) {
  return matrix.color565(
    int((r1 + ((r2 - r1) * f))),
    int((g1 + ((g2 - g1) * f))),
    int((b1 + ((b2 - b1) * f)))
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
