#include "micromouse.h"
#include "memory.h"
#include "navigation.h"
#include "movement.h"

Micromouse mm;
Memory memory;

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("1: Starting mm.begin()");
  mm.begin();
  Serial.println("2: mm.begin() finished");

  mm.speed = 255;
  mm.led_green_set(true);

  Serial.println("3: Inverting motor 1");
  mm.invert_motor_1();

  Serial.println("4: About to enter run()");
  run(mm, memory);
  
  

}

void loop() {
  //   Serial.print("Front: ");
  // Serial.print(mm.get_tof_distance(1));
  // Serial.print(" mm | Left: ");
  // Serial.print(mm.get_tof_distance(2));
  // Serial.print(" mm | Right: ");
  // Serial.print(mm.get_tof_distance(3));
  // Serial.println(" mm");

  // delay(100);   // 10 prints per second
}