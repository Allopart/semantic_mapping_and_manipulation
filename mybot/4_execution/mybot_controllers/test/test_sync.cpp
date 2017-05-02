#include "../modules/controller/motor.hpp"
#include "../modules/controller/packet.hpp"

int main(){

    goodguy::Motor m1(1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::Motor m2(99, goodguy::Motor::DYNAMIXEL_PRO);


    std::vector<unsigned char> p1;
    std::vector<unsigned char> p2;


    std::cout << "start serial communication" << std::endl << std::endl;
    boost::asio::io_service io;
    boost::asio::serial_port s1(io);
    try{
        s1.open("/dev/ttyUSB0");
    }catch(std::exception& e){
        std::cout << "Serial port open failed" << std::endl;
        return 0;
    }
    s1.set_option(boost::asio::serial_port::baud_rate(2000000));
    boost::thread thread_for_io(boost::bind(&boost::asio::io_service::run, &io));


    unsigned char send_buffer[128];

    std::cout << "torque on" << std::endl;
    m1.enable();
    m2.enable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::makeMotorPacket(p2, m2);
    goodguy::printPacket(p1.begin(), p1.end());
    goodguy::printPacket(p2.begin(), p2.end());
    p1.insert(p1.end(), p2.begin(), p2.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    

    bool is_quit = false;
    while(!is_quit){

        char key = 0x00;

        std::cout << "Set Velocity (v), Set Angle (a), Quit (q), Led Blink (l): ";
        std::cin >> key;

        if(key == 'v'){
            std::cout << "set velocity: " << std::endl;
            float vel = 1;
            std::cin  >> vel;

            m1.setGoalSpeedWithUpdate(vel);
            m2.setGoalSpeedWithUpdate(vel);
            goodguy::makeMotorPacket(p1, m1);
            goodguy::makeMotorPacket(p2, m2);

            goodguy::printPacket(p1.begin(), p1.end());
            goodguy::printPacket(p2.begin(), p2.end());
            p1.insert(p1.end(), p2.begin(), p2.end());
            std::copy(p1.begin(), p1.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
            goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
            goodguy::printPacket(p1.begin(), p1.end());
            std::copy(p1.begin(), p1.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p1.size()));

        }
        else if(key == 'a'){
            std::cout << "set angle: " << std::endl;
            float angle = 90;
            std::cin  >> angle;

            m1.setGoalAngleWithUpdate(angle);
            m2.setGoalAngleWithUpdate(-angle);
            goodguy::makeMotorPacket(p1, m1);
            goodguy::makeMotorPacket(p2, m2);
            goodguy::printPacket(p1.begin(), p1.end());
            goodguy::printPacket(p2.begin(), p2.end());
            //p1.insert(p1.end(), p2.begin(), p2.end());
            std::copy(p1.begin(), p1.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
            std::copy(p2.begin(), p2.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
            goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
            goodguy::printPacket(p1.begin(), p1.end());
            std::copy(p1.begin(), p1.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p1.size()));

        }
        else if(key == 'l'){
            static bool is_led_on = false;
            if(is_led_on){
                is_led_on = false;
                m1.led_disable();
                m2.led_disable();
            }
            else{
                is_led_on = true;
                m1.led_enable();
                m2.led_enable();
            }
            goodguy::makeMotorPacket(p1, m1);
            goodguy::makeMotorPacket(p2, m2);
            goodguy::printPacket(p1.begin(), p1.end());
            goodguy::printPacket(p2.begin(), p2.end());
            p1.insert(p1.end(), p2.begin(), p2.end());
            std::copy(p1.begin(), p1.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
            goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
            goodguy::printPacket(p1.begin(), p1.end());
            std::copy(p1.begin(), p1.end(), send_buffer);
            s1.write_some(boost::asio::buffer(send_buffer, p1.size()));

        }
        else if(key == 'q'){
            is_quit = true;
        }
        else{
            std::cout << "Wrong Input!!" << std::endl;
        }


    }
    


    return 0;
}
