#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"

//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

//motor-speed controller
SpeedController PIcontroller;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[2]) < threshold_pick_up)) return 1;
    else return 0;
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        PIcontroller.Stop();
        if(buttonA.getSingleDebouncedRelease()){ //transition condition
            robot_state = DRIVE; 
            // PIcontroller.Stop(); //action
        } 
        // else { //transition condition
        //     robot_state = IDLE; 
        //     PIcontroller.Stop(); //action 
        // }   
        break;
    case DRIVE:
        PIcontroller.Run(50, 50);
        if (buttonA.getSingleDebouncedRelease() || DetectBeingPickedUp()) {
            robot_state = IDLE;
        }
        if (DetectCollision()) {
            robot_state = REVERSE;
        }
        break;
    case REVERSE:
        if (PIcontroller.Reverse(50, 10)) {
            robot_state = TURN;
        }
        break;
    case TURN:
        if (PIcontroller.Turn(90, 0)) {
            robot_state = DRIVE;
        }
        break;
        //assignment 3
    }
    Serial.println(robot_state);
}