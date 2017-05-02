#ifndef __PACKET_HPP__
#define __PACKET_HPP__

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

#include "motor.hpp"


namespace goodguy{

    const int BROADCAST_ID = 254;

    enum DynamixelMX28Address{
        DYNAMIXEL_MX28_ADDRESS_BAUDRATE     = 0x04,
        DYNAMIXEL_MX28_ADDRESS_ENABLE       = 0x18,
        DYNAMIXEL_MX28_ADDRESS_LED          = 0x19,
        DYNAMIXEL_MX28_ADDRESS_PID          = 0x1A,
        DYNAMIXEL_MX28_ADDRESS_GOAL_ANGLE   = 0x1E,
        DYNAMIXEL_MX28_ADDRESS_GOAL_SPEED   = 0x20,
        DYNAMIXEL_MX28_ADDRESS_CURR_ANGLE   = 0x24,
        DYNAMIXEL_MX28_ADDRESS_CURR_SPEED   = 0x26,
        DYNAMIXEL_MX28_ADDRESS_CURR_TORQUE  = 0x28,
        DYNAMIXEL_MX28_ADDRESS_MOVING       = 0x2E,
        DYNAMIXEL_MX28_ADDRESS_RETURN_LEVEL = 0x10
    };
    enum DynamixelMX64Address{
        DYNAMIXEL_MX64_ADDRESS_BAUDRATE     = 0x04,
        DYNAMIXEL_MX64_ADDRESS_ENABLE       = 0x18,
        DYNAMIXEL_MX64_ADDRESS_LED          = 0x19,
        DYNAMIXEL_MX64_ADDRESS_PID          = 0x1A,
        DYNAMIXEL_MX64_ADDRESS_GOAL_ANGLE   = 0x1E,
        DYNAMIXEL_MX64_ADDRESS_GOAL_SPEED   = 0x20,
        DYNAMIXEL_MX64_ADDRESS_GOAL_TORQUE  = 0x47,
        DYNAMIXEL_MX64_ADDRESS_CURR_ANGLE   = 0x24,
        DYNAMIXEL_MX64_ADDRESS_CURR_SPEED   = 0x26,
        DYNAMIXEL_MX64_ADDRESS_CURR_TORQUE  = 0x28,
        DYNAMIXEL_MX64_ADDRESS_MOVING       = 0x2E,
        DYNAMIXEL_MX64_ADDRESS_RETURN_LEVEL = 0x10
    };

    enum DynamixelProAddress{
        DYNAMIXEL_PRO_ADDRESS_BAUDRATE     = 8,
        DYNAMIXEL_PRO_ADDRESS_ENABLE       = 562,
        DYNAMIXEL_PRO_ADDRESS_LED          = 563,
        DYNAMIXEL_PRO_ADDRESS_PID          = 586,
        DYNAMIXEL_PRO_ADDRESS_GOAL_ANGLE   = 596,
        DYNAMIXEL_PRO_ADDRESS_GOAL_SPEED   = 600,
        DYNAMIXEL_PRO_ADDRESS_GOAL_TORQUE  = 604,
        DYNAMIXEL_PRO_ADDRESS_CURR_ANGLE   = 611,
        DYNAMIXEL_PRO_ADDRESS_CURR_SPEED   = 615,
        DYNAMIXEL_PRO_ADDRESS_CURR_TORQUE  = 621,
        DYNAMIXEL_PRO_ADDRESS_MOVING       = 610,
        DYNAMIXEL_PRO_ADDRESS_RETURN_LEVEL = 891

    };
    enum DynamixelMXPacketType{
        DYNAMIXEL_MX_PACKET_PING             = 0x01,
        DYNAMIXEL_MX_PACKET_READ             = 0x02,
        DYNAMIXEL_MX_PACKET_WRITE            = 0x03,
        DYNAMIXEL_MX_PACKET_REG_WRITE        = 0x04,
        DYNAMIXEL_MX_PACKET_ACTION           = 0x05,
        DYNAMIXEL_MX_PACKET_FACTORY_RESET    = 0x06,
        DYNAMIXEL_MX_PACKET_SYNC_WRITE       = 0x83,
        DYNAMIXEL_MX_PACKET_BULK_READ        = 0x92,
    };


    enum DynamixelProPacketType{
        DYNAMIXEL_PRO_PACKET_PING             = 0x01,
        DYNAMIXEL_PRO_PACKET_READ             = 0x02,
        DYNAMIXEL_PRO_PACKET_WRITE            = 0x03,
        DYNAMIXEL_PRO_PACKET_REG_WRITE        = 0x04,
        DYNAMIXEL_PRO_PACKET_ACTION           = 0x05,
        DYNAMIXEL_PRO_PACKET_FACTORY_RESET    = 0x06,
        DYNAMIXEL_PRO_PACKET_REBOOT           = 0x08,
        DYNAMIXEL_PRO_PACKET_STATUS           = 0x55,
        DYNAMIXEL_PRO_PACKET_SYNC_READ        = 0x82,
        DYNAMIXEL_PRO_PACKET_SYNC_WRITE       = 0x83,
        DYNAMIXEL_PRO_PACKET_BULK_READ        = 0x92,
        DYNAMIXEL_PRO_PACKET_BULK_WRITE       = 0x93
    };

    enum DynamixelMXStatusType{
        DYNAMIXEL_MX_STATUS_RESULT_SUCCESS    = 0x00,
        DYNAMIXEL_MX_STATUS_RESULT_VOL_ERR    = 0x01,
        DYNAMIXEL_MX_STATUS_RESULT_LIMIT_ERR  = 0x02,
        DYNAMIXEL_MX_STATUS_RESULT_HEAT_ERR   = 0x04,
        DYNAMIXEL_MX_STATUS_RESULT_RANGE_ERR  = 0x08,
        DYNAMIXEL_MX_STATUS_RESULT_CHECK_ERR  = 0x10,
        DYNAMIXEL_MX_STATUS_RESULT_OVER_ERR   = 0x20,
        DYNAMIXEL_MX_STATUS_RESULT_INS_ERR    = 0x40
    };
    enum DynamixelProStatusType{
        DYNAMIXEL_PRO_STATUS_RESULT_SUCCESS    = 0x00,
        DYNAMIXEL_PRO_STATUS_RESULT_FAIL       = 0x01,
        DYNAMIXEL_PRO_STATUS_RESULT_INST_ERR   = 0x02,
        DYNAMIXEL_PRO_STATUS_RESULT_CRC_ERR    = 0x04,
        DYNAMIXEL_PRO_STATUS_RESULT_RANGE_ERR  = 0x08,
        DYNAMIXEL_PRO_STATUS_RESULT_LEN_ERR    = 0x10,
        DYNAMIXEL_PRO_STATUS_RESULT_LIMIT_ERR  = 0x20,
        DYNAMIXEL_PRO_STATUS_RESULT_ACC_ERR    = 0x40
    };

    struct ShortHeaderPacketException{ };
    struct ShortDataPacketException{ };
    struct LongDataPacketException{ };
    struct InvalidPacketException{ };

    struct BulkReadElement{
        BulkReadElement(): m_id(0), m_start_address(0), m_data_length(0) { }
        BulkReadElement(int id, unsigned int start_address, unsigned int data_length)
            : m_id(id), m_start_address(start_address), m_data_length(data_length) 
        { }

        int m_id;
        unsigned int m_start_address;
        unsigned int m_data_length;
    };

    struct DefaultBulkState{
        DefaultBulkState()
            : m_is_moving(0), m_angle(0), m_speed(0), m_torque(0), m_voltage(0), m_current(0), m_temperature(0)
        { }
        DefaultBulkState(bool is_moving, double angle, double speed, double torque, double voltage, double current, double temperature)
            : m_is_moving(is_moving), m_angle(angle), m_speed(speed), m_torque(torque), m_voltage(voltage), m_current(current), m_temperature(temperature)
        { }
        bool   m_is_moving;
        double m_angle;
        double m_speed;
        double m_torque;
        double m_voltage;
        double m_current;
        double m_temperature;
    };



    // print packet information
    template <class Iterator>
        void printPacket(const Iterator begin, const Iterator end){

            using value_type = typename std::iterator_traits<Iterator>::value_type;

            for(Iterator it = begin; it != end; ++it){

                value_type s = *it;
                unsigned char upper = (s >> 4) & 0xf;
                unsigned char lower = (s >> 0) & 0xf;
                const char ch[] = "0123456789ABCDEF";

                std::cout << ch[upper] << ch[lower] << " " ;
            }

            if(begin != end)
                std::cout << std::endl;
        }


    template <class RandomAccessCharIterator>
        unsigned short calculateCRC(const RandomAccessCharIterator begin, const RandomAccessCharIterator end) {
            unsigned short crc_accum = 0;
            unsigned short i, j;
            unsigned short crc_table[256] = {
                0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
                0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
                0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
                0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
                0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
                0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
                0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
                0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
                0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
                0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
                0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
                0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
                0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
                0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
                0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
                0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
                0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
                0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
                0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
                0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
                0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
                0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
                0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
            };

            for(auto it = begin; it != end; ++it)
            {
                i = ((unsigned short)(crc_accum >> 8) ^ (*it)) & 0xFF;
                crc_accum = (crc_accum << 8) ^ crc_table[i];
            }

            return crc_accum;
        }

    bool getDataForDynamixelPro(std::vector<unsigned char>& data, int& id, DynamixelProStatusType& error, const std::vector<unsigned char>& packet){
        if(packet.size() < 11){
            throw InvalidPacketException();
            return false;
        }

        std::vector<unsigned char> header(packet.begin()+0, packet.begin()+2);
        id = (int)packet[4];
        unsigned short length = (((unsigned short)packet[6]) << 8) | ((unsigned short)packet[5]);
        unsigned char instruction = packet[7];

        if(instruction != DYNAMIXEL_PRO_PACKET_STATUS){
            throw InvalidPacketException();
            return false;
        }
        error = (DynamixelProStatusType)packet[8];

        unsigned short crc = (((unsigned short)(*(packet.end()-1))) << 8) | ((unsigned short)(*(packet.end()-2)));

        if(calculateCRC(packet.begin(), packet.end()-2) == crc){
            data.clear();
            data.insert(data.begin(), packet.begin()+9,packet.end()-2);
            return true;
        }
        else{
            throw InvalidPacketException();
            return false;
        }
    }

    bool getDataForDynamixelMX(std::vector<unsigned char>& data, int& id, DynamixelMXStatusType& error, const std::vector<unsigned char>& packet){
        if(packet.size() < 6){
            throw InvalidPacketException();
            return false;
        }

        std::vector<unsigned char> header(packet.begin()+0, packet.begin()+1);
        id = (int)packet[2];
        unsigned char length = packet[3];
        error = (DynamixelMXStatusType)packet[4];

        unsigned check_sum = 0;
        for(auto it = (packet.begin()+2); it != (packet.end()-1); ++it){
            check_sum += *it;
        }
        check_sum = ~check_sum;

        if((check_sum & 0xff) == *(packet.end()-1)){
            data.clear();
            data.insert(data.begin(), packet.begin()+5,packet.end()-1);
            return true;
        }
        else{
            throw InvalidPacketException();
            return false;
        }
    }

    DefaultBulkState getDefaultBulkStateFromStream(std::vector<unsigned char>& data, Motor::MotorType motor_type){

        DefaultBulkState state;


        if(motor_type == Motor::DYNAMIXEL_MX64 || motor_type == Motor::DYNAMIXEL_MX28){

            if(data.size() != 15) return state;

            short angle_val = (((short)data[0])<<0)| (((int)data[1])<<8);
            state.m_angle = ((double)(angle_val-2048))*0.088;

            unsigned short speed_val = (((unsigned short)data[2])<<0)| (((unsigned short)data[3])<<8);
            if(speed_val <= 0 && speed_val < 1024)  state.m_speed = ((double)speed_val)*0.114;
            else if(speed_val <= 1024 && speed_val < 2048)  state.m_speed = -((double)(speed_val-1024))*0.114;

            int voltage_val = (int)data[6];
            state.m_voltage = ((double)voltage_val)/10.0;

            int temperature_val = (int)data[7];
            state.m_temperature = (double)temperature_val;

            if(data[9]) state.m_is_moving = true;
            else        state.m_is_moving = false;

            int current_val = (((int)data[13])<<0)| (((int)data[14])<<8);
            state.m_current = ((double)(current_val-2048))*4.5;
            if(motor_type == Motor::DYNAMIXEL_MX64){
                state.m_torque = state.m_current*(1.8+2.1)/2.0/1.8;
            }
            else{
                state.m_torque = state.m_current*(0.98+1.12)/2.0/0.9;
            }

        }
        else if(motor_type == Motor::DYNAMIXEL_PRO){
            if(data.size() != 16) return state;

            if(data[0]) state.m_is_moving = true;
            else        state.m_is_moving = false;

            int angle_val = (((int)data[1])<<0)| (((int)data[2])<<8) | (((int)data[3])<<16) |(((int)data[4])<<24);
            state.m_angle = ((double)angle_val)*180.0/250950.0;

            int speed_val = (((int)data[5])<<0)| (((int)data[6])<<8) | (((int)data[7])<<16) |(((int)data[8])<<24);
            state.m_speed = ((double)speed_val)/502.0;

            short current_val = (((short)data[11])<<0)| (((short)data[12])<<8);
            state.m_current = ((double)current_val)/2048.0;

            int voltage_val = (((int)data[13])<<0)| (((int)data[14])<<8);
            state.m_voltage = ((double)voltage_val)/10.0;

            int temperature_val = (int)data[15];
            state.m_temperature = (double)temperature_val;

            state.m_torque = state.m_current*33000.0*(51.0+42.5)/2.0/10.0;

            
            //For Debuging
            /*
            if(state.m_is_moving){
                std::cout << "MOVING" << std::endl;
            }
            else{
                std::cout << "STOP" << std::endl;
            }
            std::cout << "ANGLE: " << angle_val << "\t" << state.m_angle<< std::endl;
            std::cout << "SPEED: " << speed_val << "\t" << state.m_speed<< std::endl;
            std::cout << "CURRENT: " << current_val << "\t" << state.m_current<< std::endl;
            std::cout << "VOLTAGE: " << voltage_val << "\t" << state.m_voltage<< std::endl;
            std::cout << "TEMPERATURE: " << temperature_val << "\t" << state.m_temperature<< std::endl;
            */
        }

        return state;
    }




    bool makePacketForDynamixelPro(
            std::vector<unsigned char>& packet,
            int id, 
            DynamixelProPacketType type,
            const std::vector<unsigned char>& data = std::vector<unsigned char>())
    {

        unsigned short length = 3 + (short)data.size();
        packet.reserve(length + 7);

        packet.push_back(0xff);
        packet.push_back(0xff);
        packet.push_back(0xfd);
        packet.push_back(0x00);

        packet.push_back(id);
        packet.push_back((unsigned char)(length>>0)&0xff);
        packet.push_back((unsigned char)(length>>8)&0xff);

        packet.push_back((unsigned char)type);

        packet.insert(packet.end(), data.begin(), data.end());

        unsigned short crc = calculateCRC(packet.begin(), packet.end());
        packet.push_back((unsigned char)(crc>>0)&0xff);
        packet.push_back((unsigned char)(crc>>8)&0xff);

        return true;
    }

    bool makePacketForDynamixelMX64(
            std::vector<unsigned char>& packet,
            int id, 
            DynamixelMXPacketType type,
            const std::vector<unsigned char>& data = std::vector<unsigned char>())
    {
        unsigned char length = data.size() + 2;

        packet.push_back(0xff);
        packet.push_back(0xff);
        packet.push_back((unsigned char)id);
        packet.push_back(length);
        packet.push_back((unsigned char)type);
        packet.insert(packet.end(), data.begin(), data.end());

        unsigned check_sum = 0;
        for(auto it = (packet.begin()+2); it != packet.end(); ++it){
            check_sum += *it;
        }
        check_sum = ~check_sum;
        packet.push_back(check_sum);

        return true;
    }
    bool makePacketForDynamixelMX28(
            std::vector<unsigned char>& packet,
            int id, 
            DynamixelMXPacketType type,
            const std::vector<unsigned char>& data = std::vector<unsigned char>())
    {
        return makePacketForDynamixelMX64(packet, id, type, data);
    }

    bool makeMotorPacketForDynamixelMX64(std::vector<unsigned char>& packet, Motor& motor){
        int id = motor.getMotorId();

        Motor::MotorChangedAttributeType changed_attribute = motor.getCurrentMotorAttribute();

        std::vector<unsigned char> data;

        switch(changed_attribute){
            case Motor::CHANGE_ANGLE:
                {
                    data.push_back(DYNAMIXEL_MX64_ADDRESS_GOAL_ANGLE);
                    double angle = motor.getGoalAngleWithUpdate();
                    unsigned short value = (unsigned short)(angle/0.088 + 2048.0);
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);
                }
                break;
            case Motor::CHANGE_TORQUE:
                {
                    data.push_back(DYNAMIXEL_MX64_ADDRESS_GOAL_TORQUE);
                    short torque = (short)motor.getGoalTorqueWithUpdate();
                    short value = (short)torque;
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);
                }
                break;
            case Motor::CHANGE_SPEED:
                {
                    data.push_back(DYNAMIXEL_MX64_ADDRESS_GOAL_SPEED);
                    double rpm = motor.getGoalSpeedWithUpdate();
                    unsigned short value = (unsigned short)(rpm/0.114);
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);

                }
                break;
            case Motor::CHANGE_LED:
                {
                    data.push_back(DYNAMIXEL_MX64_ADDRESS_LED);
                    bool led = motor.getLEDWithUpdate();
                    if(led) data.push_back(1);
                    else    data.push_back(0);

                }
                break;
            case Motor::CHANGE_ENABLE:
                {
                    data.push_back(DYNAMIXEL_MX64_ADDRESS_ENABLE);
                    bool enable = motor.getEnableWithUpdate();
                    if(enable) data.push_back(1);
                    else       data.push_back(0);
                }
                break;
        }

        if(makePacketForDynamixelMX64(packet, id, DYNAMIXEL_MX_PACKET_REG_WRITE, data)){
            return true;
        }
        else{
            return false;
        }

    }
    bool makeMotorPacketForDynamixelMX28(std::vector<unsigned char>& packet, Motor& motor){
        int id = motor.getMotorId();

        Motor::MotorChangedAttributeType changed_attribute = motor.getCurrentMotorAttribute();

        std::vector<unsigned char> data;

        switch(changed_attribute){
            case Motor::CHANGE_ANGLE:
                {
                    data.push_back(DYNAMIXEL_MX28_ADDRESS_GOAL_ANGLE);
                    double angle = motor.getGoalAngleWithUpdate();
                    unsigned short value = (unsigned short)(2048.0*(angle + 180.0)/180.0);
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);
                }
                break;
            case Motor::CHANGE_TORQUE:
                {
                    // Not supported
                }
                break;
            case Motor::CHANGE_SPEED:
                {
                    data.push_back(DYNAMIXEL_MX28_ADDRESS_GOAL_SPEED);
                    double rpm = motor.getGoalSpeedWithUpdate();
                    unsigned short value = (unsigned short)(rpm/0.114);
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);

                }
                break;
            case Motor::CHANGE_LED:
                {
                    data.push_back(DYNAMIXEL_MX28_ADDRESS_LED);
                    bool led = motor.getLEDWithUpdate();
                    if(led) data.push_back(1);
                    else    data.push_back(0);

                }
                break;
            case Motor::CHANGE_ENABLE:
                {
                    data.push_back(DYNAMIXEL_MX28_ADDRESS_ENABLE);
                    bool enable = motor.getEnableWithUpdate();
                    if(enable) data.push_back(1);
                    else       data.push_back(0);
                }
                break;
        }

        if(makePacketForDynamixelMX28(packet, id, DYNAMIXEL_MX_PACKET_REG_WRITE, data)){
            return true;
        }
        else{
            return false;
        }

    }

    bool makeMotorPacketForDynamixelPro(std::vector<unsigned char>& packet, Motor& motor){
        int id = motor.getMotorId();

        Motor::MotorChangedAttributeType changed_attribute = motor.getCurrentMotorAttribute();

        std::vector<unsigned char> data;

        switch(changed_attribute){
            case Motor::CHANGE_ANGLE:
                {
                    unsigned short address = DYNAMIXEL_PRO_ADDRESS_GOAL_ANGLE;
                    data.push_back((unsigned char)address & 0xff);
                    data.push_back((unsigned char)(address >> 8) & 0xff);

                    double angle = motor.getGoalAngleWithUpdate();
                    int value = (int)(250950.0*(angle)/180.0);
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);
                    data.push_back((unsigned char)(value >> 16) & 0xff);
                    data.push_back((unsigned char)(value >> 24) & 0xff);
                }
                break;
            case Motor::CHANGE_TORQUE:
                {

                    unsigned short address = DYNAMIXEL_PRO_ADDRESS_GOAL_TORQUE;
                    data.push_back((unsigned char)address & 0xff);
                    data.push_back((unsigned char)(address >> 8) & 0xff);

                    double torque = motor.getGoalTorqueWithUpdate();
                    short value = (short)torque;
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);

                }
                break;
            case Motor::CHANGE_SPEED:
                {
                    unsigned short address = DYNAMIXEL_PRO_ADDRESS_GOAL_SPEED;
                    data.push_back((unsigned char)address & 0xff);
                    data.push_back((unsigned char)(address >> 8) & 0xff);
                    double rpm = motor.getGoalSpeedWithUpdate();
                    int value = (int)(rpm*502);
                    data.push_back((unsigned char)value & 0xff);
                    data.push_back((unsigned char)(value >> 8) & 0xff);
                    data.push_back((unsigned char)(value >> 16) & 0xff);
                    data.push_back((unsigned char)(value >> 24) & 0xff);

                }
                break;
            case Motor::CHANGE_LED:
                {
                    unsigned short address = DYNAMIXEL_PRO_ADDRESS_LED;
                    data.push_back((unsigned char)address & 0xff);
                    data.push_back((unsigned char)(address >> 8) & 0xff);
                    bool led = motor.getLEDWithUpdate();
                    if(led){
                        data.push_back(0xff);
                        data.push_back(0xff);
                        data.push_back(0xff);
                    }
                    else{
                        data.push_back(0);
                        data.push_back(0);
                        data.push_back(0);
                    }

                }
                break;
            case Motor::CHANGE_ENABLE:
                {
                    unsigned short address = DYNAMIXEL_PRO_ADDRESS_ENABLE;
                    data.push_back((unsigned char)address & 0xff);
                    data.push_back((unsigned char)(address >> 8) & 0xff);
                    bool enable = motor.getEnableWithUpdate();
                    if(enable) data.push_back(1);
                    else       data.push_back(0);
                }
                break;
        }

        if(makePacketForDynamixelPro(packet, id, DYNAMIXEL_PRO_PACKET_REG_WRITE, data)){
            return true;
        }
        else{
            return false;
        }

    }


    bool makeBulkReadPacket(std::vector<unsigned char>& packet, const std::vector<BulkReadElement>& bulk_read_set, Motor::MotorType motor_type){
        packet.clear();

        std::vector<unsigned char> data;

        auto lambda_bulk_read_for_dynamixel_pro = [&data](const BulkReadElement& element){
            data.push_back((unsigned char)element.m_id);
            data.push_back((unsigned char)(element.m_start_address & 0xff));
            data.push_back((unsigned char)((element.m_start_address >> 8) & 0xff));
            data.push_back((unsigned char)(element.m_data_length & 0xff));
            data.push_back((unsigned char)((element.m_data_length >> 8) & 0xff));
        };

        auto lambda_bulk_read_for_dynamixel_mx = [&data](const BulkReadElement& element){
            data.push_back((unsigned char)(element.m_data_length & 0xff));
            data.push_back((unsigned char)element.m_id);
            data.push_back((unsigned char)(element.m_start_address & 0xff));
        };

        if(bulk_read_set.size() == 0) return false;

        if(motor_type == Motor::DYNAMIXEL_MX64 || motor_type == Motor::DYNAMIXEL_MX28){
            data.push_back(0x00);
            for_each(bulk_read_set.begin(), bulk_read_set.end(), lambda_bulk_read_for_dynamixel_mx); 
            makePacketForDynamixelMX64(packet, BROADCAST_ID, DYNAMIXEL_MX_PACKET_BULK_READ, data);
        }
        else if(motor_type == Motor::DYNAMIXEL_PRO){
            for_each(bulk_read_set.begin(), bulk_read_set.end(), lambda_bulk_read_for_dynamixel_pro); 
            makePacketForDynamixelPro(packet, BROADCAST_ID, DYNAMIXEL_PRO_PACKET_BULK_READ, data);
        }
    }

    bool makeDefaultBulkReadPacket(std::vector<unsigned char>& packet, const std::vector<Motor>& motors, Motor::MotorType motor_type){
        std::vector<BulkReadElement> bulk_read_set_for_mx64;
        std::vector<BulkReadElement> bulk_read_set_for_pro;

        for(auto it = motors.begin(); it != motors.end(); ++it){
            if(it->getMotorType() == Motor::DYNAMIXEL_PRO){
                bulk_read_set_for_pro.push_back(BulkReadElement((int)it->getMotorId(), (unsigned int)DYNAMIXEL_PRO_ADDRESS_MOVING, 16));
            }
            else if(it->getMotorType() == Motor::DYNAMIXEL_MX64 || it->getMotorType() == Motor::DYNAMIXEL_MX28){
                bulk_read_set_for_mx64.push_back(BulkReadElement((int)it->getMotorId(), (unsigned int)DYNAMIXEL_MX64_ADDRESS_CURR_ANGLE, 15));
            }
            else{
                std::cout << "It is not Supported motor series (" << it->getMotorId() << ")" << std::endl;
            }
        }

        std::vector<unsigned char> packet_for_mx64;
        std::vector<unsigned char> packet_for_pro;

        makeBulkReadPacket(packet_for_pro, bulk_read_set_for_pro, Motor::DYNAMIXEL_PRO);
        makeBulkReadPacket(packet_for_mx64, bulk_read_set_for_mx64, Motor::DYNAMIXEL_MX64);
        packet.clear();
        if(motor_type == Motor::DYNAMIXEL_PRO)       packet.insert(packet.end(), packet_for_pro.begin(), packet_for_pro.end());
        else if(motor_type == Motor::DYNAMIXEL_MX64)  packet.insert(packet.end(), packet_for_mx64.begin(), packet_for_mx64.end());
    }


    bool makeStatusReturnSettingPacket(std::vector<unsigned char>& packet, Motor::MotorType motor_type){

        packet.clear();

        std::vector<unsigned char> data;

        if(motor_type == Motor::DYNAMIXEL_MX64){
            //makePacketForDynamixelMX64(packet, BROADCAST_ID, DYNAMIXEL_MX_PACKET_WRITE, data);
        }
        else if(motor_type == Motor::DYNAMIXEL_PRO){
            data.push_back((unsigned char)(DYNAMIXEL_PRO_ADDRESS_RETURN_LEVEL)&0xff);
            data.push_back((unsigned char)(DYNAMIXEL_PRO_ADDRESS_RETURN_LEVEL>>8)&0xff);
            data.push_back(0x01);
            makePacketForDynamixelPro(packet, BROADCAST_ID, DYNAMIXEL_PRO_PACKET_WRITE, data);
        }
    }


    bool makeActionPacket(std::vector<unsigned char>& packet, Motor::MotorType motor_type){

        packet.clear();

        if(motor_type == Motor::DYNAMIXEL_MX64){
            makePacketForDynamixelMX64(packet, BROADCAST_ID, DYNAMIXEL_MX_PACKET_ACTION);
        }
        else if(motor_type == Motor::DYNAMIXEL_MX28){
            makePacketForDynamixelMX28(packet, BROADCAST_ID, DYNAMIXEL_MX_PACKET_ACTION);
        }
        else if(motor_type == Motor::DYNAMIXEL_PRO){
            makePacketForDynamixelPro(packet, BROADCAST_ID, DYNAMIXEL_PRO_PACKET_ACTION);
        }
    }

    bool makeMotorPacket(std::vector<unsigned char>& packet, Motor& motor){

        packet.clear();

        if(motor.isChanged()){
            Motor::MotorType motor_type = motor.getMotorType();

            if(motor_type == Motor::DYNAMIXEL_MX64){
                makeMotorPacketForDynamixelMX64(packet, motor);
            }
            else if(motor_type == Motor::DYNAMIXEL_MX28){
                makeMotorPacketForDynamixelMX28(packet, motor);
            }
            else if(motor_type == Motor::DYNAMIXEL_PRO){
                makeMotorPacketForDynamixelPro(packet, motor);
            }

        }
    }


}
#endif
