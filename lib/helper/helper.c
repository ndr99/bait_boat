#include "helper.h"

int min_int(int x, int y){
    if(x<y){
        return x;
    }else{
        return y;
    }
}


int max_int(int x, int y){
    if(x>y){
        return x;
    }else{
        return y;
    }
}


//triangle wave divided into number of steps
int triangle_wave(int i, int number_of_steps){
    return abs(i % (2 * number_of_steps) - number_of_steps)-number_of_steps/2;
}


// Do something for 1 microseconds
void wait_one_micros(){
    for(volatile int k=0;k<7;k++){};
}


// do something for delay_us microseconds
void wait_micros(int delay_us){
    for(int j=0;j<delay_us;j++){
        wait_one_micros();
    }
}