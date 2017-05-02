#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__

#include <boost/timer/timer.hpp>
#include <boost/thread.hpp>
#include <boost/random.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


#include <string>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

namespace goodguy{

    class Motor{
        public:
            enum MotorType{
                DYNAMIXEL_PRO,
                DYNAMIXEL_MX64,
                DYNAMIXEL_MX28
            };

            enum MotorChangedAttributeType{
                CHANGE_ANGLE,
                CHANGE_TORQUE,
                CHANGE_SPEED, 
                CHANGE_LED,
                CHANGE_ENABLE
            };


        public:
            Motor(int motor_id, MotorType motor_type)
                : m_is_change(false), m_enable(false), m_led(false), m_motor_id(motor_id), m_motor_type(motor_type)
            {

            }

            bool isChanged() const { return m_is_change; }
            int getMotorId() const { return m_motor_id; }
            MotorType getMotorType() const { return m_motor_type; }


            void enable(){
                m_enable = true;
                applyChange(CHANGE_ENABLE);
            }
            void disable(){
                m_enable = false;
                applyChange(CHANGE_ENABLE);
            }

            void led_enable(){
                m_led = true;
                applyChange(CHANGE_LED);
            }
            void led_disable(){
                m_led = false;
                applyChange(CHANGE_LED);
            }

            void setGoalTorqueWithUpdate(double goal_torque){
                m_goal_torque = goal_torque;
                applyChange(CHANGE_TORQUE);
            }
            void setGoalSpeedWithUpdate(double goal_speed){
                m_goal_speed = goal_speed;
                applyChange(CHANGE_SPEED);
            }
            void setGoalAngleWithUpdate(double goal_angle){
                m_goal_angle = goal_angle;
                applyChange(CHANGE_ANGLE);
            }

            void setCurrTorque(double curr_torque) { m_curr_torque = curr_torque; }
            void setCurrAngle(double curr_angle)   { m_curr_angle = curr_angle; }
            void setCurrSpeed(double curr_speed)   { m_curr_speed = curr_speed; }
            void setGoalTorque(double goal_torque) { m_goal_torque = goal_torque; }
            void setGoalAngle(double goal_angle)   { m_goal_angle = goal_angle; }
            void setGoalSpeed(double goal_speed)   { m_goal_speed = goal_speed; }

            double getGoalSpeedWithUpdate(){ 
                adoptedChange(CHANGE_SPEED);
                return m_goal_speed; 
            }

            double getGoalAngleWithUpdate(){ 
                adoptedChange(CHANGE_ANGLE);
                return m_goal_angle; 
            }

            double getGoalTorqueWithUpdate(){ 
                adoptedChange(CHANGE_TORQUE);
                return m_goal_torque; 
            } 

            bool getEnableWithUpdate(){
                adoptedChange(CHANGE_ENABLE);
                return m_enable;
            }

            bool getLEDWithUpdate(){
                adoptedChange(CHANGE_LED);
                return m_led;
            }

            double getCurrSpeed()  const { return m_curr_speed; }
            double getCurrAngle()  const { return m_curr_angle; }
            double getCurrTorque() const { return m_curr_torque; }
            double getGoalSpeed()  const { return m_goal_speed; }
            double getGoalAngle()  const { return m_goal_angle; }
            double getGoalTorque() const { return m_goal_torque; }
            bool   getEnable()     const { return m_enable; }
            bool   getLED()        const { return m_led;}


            MotorChangedAttributeType getCurrentMotorAttribute() const {  return m_motor_change_attr; }


        private:
            void applyChange(MotorChangedAttributeType motor_change_attr){
                m_motor_change_attr = motor_change_attr;
                m_is_change = true;
            }

            void adoptedChange(MotorChangedAttributeType motor_change_attr){
                m_is_change = false;
            }



        private:
            bool m_is_change;
            bool m_enable;
            bool m_led;

            MotorChangedAttributeType m_motor_change_attr;

            int m_motor_id;


            MotorType m_motor_type;
            
            double m_curr_torque;
            double m_curr_angle;
            double m_curr_speed;
            
            double m_goal_torque;
            double m_goal_angle;
            double m_goal_speed;




    };

}
#endif
