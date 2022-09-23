#include <vector_type.h>
#include <quaternion_type.h>

#include <Adafruit_Protomatter.h>
#include <math.h>

#define DEBUG false

#define MILLIS_PER_PHASE 300000

#define MAX_SWARM_ROT_SPEED 1.0

#define HEIGHT  32 // Matrix height (pixels) - SET TO 64 FOR 64x64 MATRIX!
#define WIDTH   64 // Matrix width (pixels)
#define MAX_FPS 45 // Maximum redraw rate, frames/second
#define SCALE_BRIGHTNESS 1.0

#define PLASMA_PALETTE_SIZE 240
#define PLASMA_COLORS 6

#define LIFE_INIT_DENSITY 30 // Express as percentage
#define LIFE_GENERATIONS 400
#define LIFE_FADE_SPEED 1

uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
uint8_t addrPins[] = {17, 18, 19, 20};
uint8_t clockPin   = 14;
uint8_t latchPin   = 15;
uint8_t oePin      = 16;

double plasma_current_value = 0.0;
double plasma_step_value = 0.8;
double plasma_randomizing_param;

Adafruit_Protomatter matrix(
  64, 6, 1, rgbPins, 4, addrPins, clockPin, latchPin, oePin, true); // true indicates double-buffering

uint16_t plasma_palette[PLASMA_PALETTE_SIZE];
uint16_t white_pixel = matrix.color565(255, 255, 255);
uint16_t black_pixel = matrix.color565(0, 0, 0);
uint16_t grey_pixel = matrix.color565(90, 90, 90);

double sin_lut[360];
double cos_lut[360];
double sqrt_lut[5120];

const uint8_t gamma_table[32] = {
  0,
  0,
  0,
  1,
  2,
  4,
  6,
  8,
  11,
  15,
  19,
  24,
  29,
  35,
  41,
  48,
  56,
  64,
  73,
  83,
  93,
  104,
  116,
  128,
  142,
  155,
  170,
  186,
  202,
  219,
  236,
  255,
};
int gamma_table_size = (sizeof(gamma_table) / sizeof(uint8_t));

int reset_counter;
#define PLASMA_CYCLES_LIMIT 5000

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
  if (numBits > 31 or numBits < 1) numBits = 31; // limit input range

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

// END RANDOM COLOR SEED STUFF

void randomize_plasma() {
  randomize_plasma_palette();
  plasma_current_value = random(100000);
  plasma_randomizing_param = 10 + (random(750) / 50.0);
}

void randomize_seed() {
  randomSeed(getRandomSeed(31));
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
  return double(random(255));
}

void randomize_plasma_palette(void) {
  // The first color is black. The rest are randomly determined
  // We should maybe enforce some distance between two colors?
  double anchor_colors[PLASMA_COLORS][3];
  for (int x = 0; x < 3; x++) {
    anchor_colors[0][x] = 0;
  }
  for (int row = 1; row < PLASMA_COLORS; row++) {
    for (int col = 0; col < 3; col++) {
      anchor_colors[row][col] = random_byte();
    }
  }

  int palette_steps = PLASMA_PALETTE_SIZE / PLASMA_COLORS;
  double deg_smoothing_step = 180.0 / palette_steps;

  for (int anchor_idx = 0; anchor_idx < PLASMA_COLORS; anchor_idx++) {
    int start_idx = anchor_idx * palette_steps;
    int stop_idx = start_idx + palette_steps;
    int to_idx = (anchor_idx + 1) % PLASMA_COLORS;

    for (int i = start_idx; i < stop_idx; i++) {
      int j = i % palette_steps;
      double degs = j * deg_smoothing_step;
      double rads = (degs * 71) / 4068.0;
      double f = 1.0 - ((cos(rads) + 1) / 2.0);

      plasma_palette[i] = interpolate_palette(anchor_colors[anchor_idx], anchor_colors[to_idx], f);
    }
  }
}

uint16_t interpolate_palette(double from[3], double to[3], double f) {
  int red   = int((from[0] + ((to[0] - from[0]) * f)) * SCALE_BRIGHTNESS);
  int green = int((from[1] + ((to[1] - from[1]) * f)) * SCALE_BRIGHTNESS);
  int blue  = int((from[2] + ((to[2] - from[2]) * f)) * SCALE_BRIGHTNESS);

  return matrix.color565(red, green, blue);
}

void fast_plasma(double seed_value) {
  // Uses lookup tables for lots of stuff
  for (int x = 0; x < WIDTH; x++) {
    for (int y = 0; y < HEIGHT; y++) {
      double dx = double(x);
      double dy = double(y) / 1.5;

      double calculated_value = sin_l((dx + seed_value) / plasma_randomizing_param) +
                                cos_l((dy + (seed_value / 2.0)) / 9.0) +
                                // sin_l(dy / 9.0) +
                                sin_l(((dx + seed_value) + (dy - seed_value)) / 25.0) +
                                // sin_l(sqrt_lut[(y * 64) + x] / 8.0);
                                sin_l(sqrt_lut[(y * 64) + x]);
      int scaled_value = int(abs(calculated_value) * 100);
      int output_color = scaled_value % PLASMA_PALETTE_SIZE;

      matrix.drawPixel(x, y, plasma_palette[output_color]);
    }
  }
}
void run_plasma(void) {
  randomize_plasma();
  unsigned long int target_millis = millis() + MILLIS_PER_PHASE;

  for (;;) {
    matrix.fillScreen(0x0);
    fast_plasma(plasma_current_value);
    matrix.show();
    plasma_current_value += plasma_step_value;

    if (millis() > target_millis) {
      break;
    }
  }
}

void printVector( vec3_t vec ) {
  if (!DEBUG) { return; }
  
  Serial.print( vec.x );
  Serial.print( ", " );
  Serial.print( vec.y );
  Serial.print( ", " );
  Serial.print( vec.z );
  Serial.println();
}

void printQuat( quat_t quat ) {
  if (!DEBUG) { return; }

  Serial.print( quat.w );
  Serial.print( ", " );
  printVector( quat.v );
}


int z_sort(const void *cmp1, const void *cmp2)
{
  vec3_t a = *((vec3_t *)cmp1);
  vec3_t b = *((vec3_t *)cmp2);

  return a.z > b.z ? -1 : (a.z < b.z ? 1 : 0);
}

void run_particle_swarm(void) {
  // Create a cloud of particles
  Serial.println("Creating swarm.");
  vec3_t cube[20] = {
    { -7, -7, -7 },
    { -7, -7,  0 },
    { -7, -7,  7 },
    { -7,  0, -7 },
    { -7,  0,  7 },
    { -7,  7, -7 },
    { -7,  7,  0 },
    { -7,  7,  7 },
    {  7, -7, -7 },
    {  7, -7,  0 },
    {  7, -7,  7 },
    {  7,  0, -7 },
    {  7,  0,  7 },
    {  7,  7, -7 },
    {  7,  7,  0 },
    {  7,  7,  7 },
    {  0, -7, -7 },
    {  0, -7,  7 },
    {  0,  7, -7 },
    {  0,  7,  7 }
  };

  vec3_t octohedron[24] = {
    {   -7,   -7,    0 },
    {   -7,    7,    0 },
    {    7,   -7,    0 },
    {    7,    7,    0 },
    {    0,    0,    7 },
    {    0,    0,   -7 },
    { -3.5, -3.5,    0 },
    { -3.5,  3.5,    0 },
    {  3.5, -3.5,    0 },
    {  3.5,  3.5,    0 },
    { -3.5,    0, -3.5 },
    { -3.5,    0,  3.5 },
    {  3.5,    0, -3.5 },
    {  3.5,    0,  3.5 },
    {    0, -3.5, -3.5 },
    {    0, -3.5,  3.5 },
    {    0,  3.5, -3.5 },
    {    0,  3.5,  3.5 }
  };

  // vec3_t *model = cube;
  // vec3_t *sorted = model;
  
  int point_count = sizeof(cube) / sizeof(vec3_t);

  Serial.println("Done generating swarm.");

  int x_dir = 1, y_dir = 1, z_dir = -1;

  float rot_speed_x = 0.1;
  float rot_speed_y = 0.1;
  float rot_speed_z = -0.1;

  float angle_x, angle_y, angle_z, x_tmp, y_tmp;

  quat_t qrot_x, qrot_y, qrot_z, qrot_net;

  vec3_t x_axis = { 1, 0, 0 };
  vec3_t y_axis = { 0, 1, 0 };
  vec3_t z_axis = { 0, 0, 1 };

  float x_center_speed = 0.03, y_center_speed = 0.02;
  float center_y = 16, center_x = 32, x_center_dir = x_center_speed, y_center_dir = y_center_speed;

  unsigned long int last_millis;

  unsigned long int target_millis = millis() + MILLIS_PER_PHASE;

  for (;;) {
    /////
    // Calculate rotation quaternion

    last_millis = millis();

    if (last_millis > target_millis) {
      // All done!
      break;
    }

    rot_speed_x += (random(4) / 1000.0) * x_dir;
    if (abs(rot_speed_x) >= MAX_SWARM_ROT_SPEED) {
      x_dir *= -1;
    };
    rot_speed_y += (random(4) / 1000.0) * y_dir;
    if (abs(rot_speed_y) >= MAX_SWARM_ROT_SPEED) {
      y_dir *= -1;
    };
    rot_speed_z += (random(4) / 1000.0) * z_dir;
    if (abs(rot_speed_z) >= MAX_SWARM_ROT_SPEED) {
      z_dir *= -1;
    };

    angle_x = (PI / 180.0) * rot_speed_x;
    angle_y = (PI / 180.0) * rot_speed_y;
    angle_z = (PI / 180.0) * rot_speed_z;

    qrot_x.setRotation(false, x_axis, angle_x);
    qrot_y.setRotation(false, y_axis, angle_y);
    qrot_z.setRotation(false, z_axis, angle_z);

    qrot_net = qrot_x * qrot_y * qrot_z;
    /////

    int max_x = 0, min_x = 64, max_y = 0, min_y = 32;

    matrix.fillScreen(0x0);
    for (int idx = 0; idx < point_count; idx++) {
      vec3_t source = cube[idx];
      // Serial.print("=== Rotating point ");
      // Serial.println(idx);
      printVector(cube[idx]);
      vec3_t result = qrot_net.rotate(true, cube[idx]);
      cube[idx] = result;
    }

    qsort(cube, point_count, sizeof(vec3_t), z_sort);
    
    for (int idx = 0; idx < point_count; idx++) {
      vec3_t result = cube[idx];
            
      printVector(result);

      x_tmp = int(result.x + center_x);
      y_tmp = int(result.y + center_y);

      max_x = max(max_x, x_tmp);
      min_x = min(min_x, x_tmp);
      max_y = max(max_y, y_tmp);
      min_y = min(min_y, y_tmp);

      float depth_color = 128.0 + (10 * result.z);

      matrix.drawPixel(x_tmp, y_tmp, matrix.color565(depth_color, depth_color, depth_color));
    }

    // TODO: Create an array of points resulting from the rotation, sort it, then draw.
    // For a cube, the four farthest back shouldn't be drawn? Is it that simple?

    matrix.show();

    // If we've hit an edge, bounce
    if (max_x >= 63) {
      x_center_dir = -abs(x_center_dir);
    }
    if (min_x <= 0) {
      x_center_dir = abs(x_center_dir);
    }
    if (max_y >= 31) {
      y_center_dir = -abs(y_center_dir);
    }
    if (min_y <= 0 ) {
      y_center_dir = abs(y_center_dir);
    }

    center_x += x_center_dir;
    center_y += y_center_dir;

    if (DEBUG) {
      Serial.print("X min: ");
      Serial.print(min_x);
      Serial.print(" max: ");
      Serial.print(max_x);
      Serial.print(" Y min: ");
      Serial.print(min_y);
      Serial.print(" max: ");
      Serial.print(max_y);
      Serial.print("  /  Dir X: ");
      Serial.print(x_center_dir);
      Serial.print(" Y: ");
      Serial.println(y_center_dir);
    }

    // Limit frame rate
    for (;;) {
      if (millis() - last_millis >= 5) {
        break;
      };
    }
  }
}

void run_game_of_life(int reps) {
  unsigned long int target_millis = millis() + MILLIS_PER_PHASE;

  for (int rep = 0; rep < reps; rep++) {
    // Initialize
    Serial.println("Running Game of Life");
    int life_arrays[3][WIDTH * HEIGHT];
    Serial.println("Initializing random life array");
    for (int i = 0; i < (WIDTH * HEIGHT); i++) {
      life_arrays[0][i] = (random(100) < LIFE_INIT_DENSITY) ? 1 : 0;
    }

    int from_array = 0;
    int to_array, yyy, ym1, yp1, xm1, xp1, neighbors, new_value;

    // Fade into view
    matrix.fillScreen(0x0);
    for (int idx = 0; idx < gamma_table_size; idx++) {
      int fade = gamma_table[idx];
      for (int x = 0; x < WIDTH; x++) {
        for (int y = 0; y < HEIGHT; y++) {
          if (life_arrays[0][(y * WIDTH) + x] == 1) {
            matrix.drawPixel(x, y, matrix.color565(fade, fade, fade));
          }
        }
      }
      matrix.show();
      delay(10);
    }

    Serial.println("Starting simulation");

    // TO DO: Fade between frames

    unsigned long int last_millis;

    for (;;) {
      last_millis = millis();
      int to_array = (from_array + 1) % 3;
      int ancestor = (from_array - 1) % 3;
      bool matches_ancestor = true;
      int live_count = 0;
      // Populate from-array
      for (int y = 0; y < HEIGHT; y++) {
        yyy = y * WIDTH;
        ym1 = ((y + HEIGHT - 1) % HEIGHT) * WIDTH;
        yp1 = ((y + 1) % HEIGHT) * WIDTH;
        xm1 = WIDTH - 1;
        for (int x = 0; x < WIDTH; x++) {
          xp1 = (x + 1) % WIDTH;
          neighbors = (
                        life_arrays[from_array][xm1 + ym1] + life_arrays[from_array][xm1 + yyy] + life_arrays[from_array][xm1 + yp1] +
                        life_arrays[from_array][x   + ym1] +                                      life_arrays[from_array][x   + yp1] +
                        life_arrays[from_array][xp1 + ym1] + life_arrays[from_array][xp1 + yyy] + life_arrays[from_array][xp1 + yp1]
                      );
          int self_value = life_arrays[from_array][x + yyy];
          bool genesis = ((neighbors == 3) && (self_value == 0));
          new_value = ((neighbors == 3) || ((neighbors == 2) && (self_value == 1))) ? 1 : 0;
          live_count += new_value;
          life_arrays[to_array][x + yyy] = new_value;
          xm1 = x;
          matrix.drawPixel(x, y, (new_value == 1) ? (genesis ? white_pixel : grey_pixel) : black_pixel);

          // Look two generations back
          // If any cell _isn't_ a match, we have a new configuration
          // TODO: Optimize with a bitmap??
          if (matches_ancestor) {
            if (life_arrays[ancestor][x + yyy] != new_value) {
              matches_ancestor = false;
            }
          };
        }


      }

      // Activate display
      matrix.show();

      // Reset the display if the grid is about to die, if we're in a loop, or if the full run time has expired
      if (live_count < 3 || matches_ancestor || (millis() > target_millis)) {
        break;
      }

      // Control frame rate--150 =~ 6.7 frames per second.
      for (;;) {
        if (millis() > last_millis + 150) {
          break;
        }
      }

      // Serial.println(LIFE_GENERATIONS - generation);
      from_array = to_array;
    }

    // Fade out
    for (int idx = 0; idx < gamma_table_size; idx++) {
      int fade = gamma_table[gamma_table_size - idx];
      for (int x = 0; x < WIDTH; x++) {
        for (int y = 0; y < HEIGHT; y++) {
          if (life_arrays[to_array][(y * WIDTH) + x] == 1) {
            matrix.drawPixel(x, y, matrix.color565(fade, fade, fade));
          }
        }
      }
      matrix.show();
    }

    if (millis() > target_millis) {
      break;
    }
  }
}

void setup(void) {
  Serial.begin(9600);

  // Initialize matrix...
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  if (status != PROTOMATTER_OK) {
    for (;;);
  }

  randomize_seed();

  // Initialize sine LUT
  Serial.println("Building sine LUT");
  for (int x = 0; x < 360; x++) {
    float x_rads = (x * 71) / 4068.0;
    sin_lut[x] = sin(x_rads);
  }

  // Initialize cosine LUT
  Serial.println("Building cosine LUT");
  for (int x = 0; x < 360; x++) {
    float x_rads = (x * 71) / 4068.0;
    cos_lut[x] = cos(x_rads);
  }

  // Initialize sqrt LUT
  Serial.println("Building pre-scaled sqrt LUT");
  for (int x = 0; x < 64; x++) {
    for (int y = 0; y < 32; y++) {
      int idx = (y * 64) + x;
      double dy = double(y) / 2.0;
      // sqrt_lut[idx] = sqrt((x * x) + (dy * dy));
      sqrt_lut[idx] = sqrt((x * x) + (dy * dy)) / 8.0;
    }
  }
}

void loop(void) {
  run_plasma();
  run_game_of_life(8);
  run_particle_swarm();
}
