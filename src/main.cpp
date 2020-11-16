#include <Arduino.h>
#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"

Behaviors collisionBehavior;
extern SpeedController PIcontroller;

void setup() {
  collisionBehavior.Init();
}

void loop() {
  collisionBehavior.Run();
}