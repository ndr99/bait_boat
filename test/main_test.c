#include <unity.h>
#include "servo_control.h"

void test_angle_to_us(void){
    TEST_ASSERT_EQUAL(2000, angle_to_us(0, 1000, 3000, -90, 90));
}

void app_main(){
    UNITY_BEGIN();
    //RUN_TEST(test_angle_to_us);
    UNITY_END();
}