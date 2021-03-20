#include <math.h>
#include <esp_system.h>

// minmax prototype
int max_int(int x, int y);
int min_int(int x, int y);

//triangle
int triangle_wave(int i, int maxvalue);

// Do something for 1 microseconds
void wait_one_micros();

// do something for delay_us microseconds
void wait_micros(int delay_us);