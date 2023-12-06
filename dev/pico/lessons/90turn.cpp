#include "rcc_stdlib.h"
using namespace std;

//IR sensor pins
#define IR_1 26 //leftmost
#define IR_2 19 //inner left
#define IR_3 20 // middle
#define IR_4 6 //inner right
#define IR_5 5 //rightmost
#define IR_6 27 //parking sensor

int s1 = gpio_get(IR_1);
int s2 = gpio_get(IR_2);
int s3 = gpio_get(IR_3);
int s4 = gpio_get(IR_4);
int s5 = gpio_get(IR_5);
int s6 = gpio_get(IR_6);

//set state names
typedef enum{
        FORWARD,
        LEFT,
        CHECK,
        RIGHT,
        TURN90,
        TURN_AROUND,
        PARK
}state_t;

typedef enum{
    DWELL,
    INTEGRATE
}integratorstate_t;

int main(void)
{
    stdio_init_all();
    sleep_ms(100);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, true); //led on

    //setup irsensor
    gpio_init(IR_1);
    gpio_init(IR_2);
    gpio_init(IR_3);
    gpio_init(IR_4);
    gpio_init(IR_5);
    gpio_init(IR_6);
 
    gpio_set_dir(IR_1, false);
    gpio_set_dir(IR_2, false);
    gpio_set_dir(IR_3, false);
    gpio_set_dir(IR_4, false);
    gpio_set_dir(IR_5, false);
    gpio_set_dir(IR_6, false);

    //setup IMU 
    rcc_init_i2c();
    MPU6050 imu; 
    imu.begin(i2c1); 
    imu.calibrate(); 

    //struct setup
    Motor motors; 
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);  
    MotorsOn(&motors); 

    //timing variables for riemann sum
    uint32_t current_time, previous_time;
    uint32_t duration = 10000; //10000 us to be 10ms

    //setup variables to track angle 
    double theta = 0.0;

    double turn_degrees = 72.0;
    double turn_arounddegree = 140.0;

    //default state of robot
    state_t robot_state = FORWARD;
    integratorstate_t imustate = DWELL;

    while(true)
    {   
        //update imu data and update current time
        imu.update_pico(); //updates data
        current_time = time_us_32(); //update current time

        s1 = gpio_get(IR_1);
        s2 = gpio_get(IR_2);
        s3 = gpio_get(IR_3);
        s4 = gpio_get(IR_4);
        s5 = gpio_get(IR_5);
        s6 = gpio_get(IR_6);
        cout << "theta:" << theta << "IR 1: " << s1 << " IR 2: " << s2 << " IR 3: " << s3 << " IR 4: " << s4 << " IR 5: " << s5 << " IR 6: " << s6 << "\n";
                        
        
        switch(robot_state){
            case FORWARD:
            cout << "FORWARD \n";
            MotorPower(&motors, 60, 60);    
            if(!s2 && !s3 && s4 && !s6){
                robot_state = LEFT;
            }
            if(s2 && !s3 && !s4 && !s6){
                robot_state = RIGHT;
            }
            if(s1 && s2 && s3 && s4 && s5 && !s6){
                robot_state = TURN90;
            }
            if(!s1 && !s2 && !s3 && !s4 && !s5 && !s6){
                robot_state = TURN_AROUND;
            }
            /*if(s1 && s2 && s3 && s4 && s5 && s6){
                robot_state = PARK;
            }
            break;
            */
            case LEFT:
            cout << "LEFT \n";
            MotorPower(&motors, 60, 0);
            if(!s1 && !s2 && s3 && !s4 && !s5){
                robot_state = FORWARD;
                theta = 0.0;
            }
            break;

            case RIGHT:
            cout << "RIGHT \n";
            MotorPower(&motors, 0, 60);
            if(s3){
                robot_state = FORWARD;
                theta = 0.0;
            }   
            break;

            case TURN90:
            cout << "TURN90 \n";
            MotorPower(&motors, 0, 70);
                
            if(theta >= turn_degrees){
                    robot_state = CHECK;
                    theta = 0.0;
            }
            break; 

            case TURN_AROUND:
            cout << "TURNAROUND \n";
            MotorPower(&motors, -70, 70);
                
            if(theta >= turn_arounddegree){
                    robot_state = CHECK;
                    theta = 0.0;
            }
            break;

            case CHECK:
            cout << "CHECK \n";
            MotorPower(&motors, 50, 50);

            if(s3){
                robot_state = FORWARD;
            }
            if(s2 ){
                robot_state = RIGHT;
            }
            if(s4){
                robot_state = LEFT;
            }
            break;

            /*case PARK:
            cout << "PARK \n";
            MotorPower(&motors, 0, 0);
            break;
            */
        } 

        switch(imustate){
            case DWELL:
                
                if(current_time - previous_time >= duration){
                    imustate = INTEGRATE;
                }
                break;

            case INTEGRATE:
                //in this state, we do the math and update theta
                theta = theta + imu.getAngVelZ()*duration/1000000.0;
                //automatically want to go back to the other state
                imustate = DWELL;
                previous_time = current_time; //update time
                break; 
        }

}
}