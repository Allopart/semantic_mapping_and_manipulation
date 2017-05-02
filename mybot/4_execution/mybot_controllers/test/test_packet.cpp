#include "../modules/controller/motor.hpp"
#include "../modules/controller/packet.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>


int main(){

    goodguy::Motor m1(1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::Motor m2(99, goodguy::Motor::DYNAMIXEL_PRO);


    std::vector<unsigned char> p1;
    std::vector<unsigned char> p2;
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());




    std::cout << "test motor packet set angle " << std::endl;
    m1.setGoalAngleWithUpdate(120);
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());


    std::cout << "test motor packet after set angle " << std::endl;
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());

    std::cout << "test motor packet torque on" << std::endl;
    m1.enable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());

    std::cout << "test motor packet led on" << std::endl;
    m1.led_enable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());


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

    std::cout << "send status setting  packet" << std::endl;
    goodguy::makeStatusReturnSettingPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::printPacket(p1.begin(), p1.end());
    unsigned char send_buffer[128];
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));


    

    std::cout << "send led on packet" << std::endl << std::endl;;
    m1.led_enable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));

    std::cout << "send action packet" << std::endl << std::endl;
    goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));

    std::cout << "torque on" << std::endl;
    m1.enable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    
    m2.enable();
    goodguy::makeMotorPacket(p2, m2);
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
    goodguy::makeActionPacket(p2, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
    std::cout << "SLEEP              " << std::endl;;
    sleep(1);

    /*
    std::cout << "send led off packet" << std::endl;;
    m1.led_disable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));



    std::cout << "send led on packet" << std::endl << std::endl;;
    m1.led_enable();
    goodguy::makeMotorPacket(p1, m1);
    goodguy::printPacket(p1.begin(), p1.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    */

    
   




    std::cout << "send led blinky to motor2" << std::endl << std::endl;;
    m2.led_disable();
    goodguy::makeMotorPacket(p2, m2);
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
    goodguy::makeActionPacket(p2, goodguy::Motor::DYNAMIXEL_MX64);
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));

    sleep(1);

    m2.led_enable();
    goodguy::makeMotorPacket(p2, m2);
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
    goodguy::makeActionPacket(p2, goodguy::Motor::DYNAMIXEL_MX64);
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));

    /*
    std::cout << "send velocity to both motor" << std::endl;
    std::cout << "set velocity: " << std::endl;
    float vel1 = 1;
    std::cin  >> vel1;

    m1.setGoalSpeedWithUpdate(vel1);
    m2.setGoalSpeedWithUpdate(vel1);
    goodguy::makeMotorPacket(p1, m1);
    goodguy::makeMotorPacket(p2, m2);
    goodguy::printPacket(p1.begin(), p1.end());
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
    goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::makeActionPacket(p2, goodguy::Motor::DYNAMIXEL_MX64);
    goodguy::printPacket(p1.begin(), p1.end());
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));

    std::cout << "set angle: " << std::endl;
    float angle1 = 0;
    std::cin  >> angle1;

    m1.setGoalAngleWithUpdate(angle1);
    m2.setGoalAngleWithUpdate(angle1);
    goodguy::makeMotorPacket(p1, m1);
    goodguy::makeMotorPacket(p2, m2);
    goodguy::printPacket(p1.begin(), p1.end());
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
    goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
    goodguy::makeActionPacket(p2, goodguy::Motor::DYNAMIXEL_MX64);
    goodguy::printPacket(p1.begin(), p1.end());
    goodguy::printPacket(p2.begin(), p2.end());
    std::copy(p1.begin(), p1.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
    std::copy(p2.begin(), p2.end(), send_buffer);
    s1.write_some(boost::asio::buffer(send_buffer, p2.size()));

    io.stop();
    thread_for_io.join();
    std::cout << "test is quit" << std::endl;
    */

    goodguy::DefaultBulkState state1;
        std::cout << "set velocity: " << std::endl;
        float vel1 = 1;
        std::cin  >> vel1;

        m1.setGoalSpeedWithUpdate(vel1);
        goodguy::makeMotorPacket(p1, m1);
        goodguy::printPacket(p1.begin(), p1.end());
        std::copy(p1.begin(), p1.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
        goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
        goodguy::printPacket(p1.begin(), p1.end());
        std::copy(p1.begin(), p1.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p1.size()));

        std::cout << "set angle: " << std::endl;
        float angle1 = 90;
        std::cin  >> angle1;

        m1.setGoalAngleWithUpdate(angle1);
        goodguy::makeMotorPacket(p1, m1);
        goodguy::printPacket(p1.begin(), p1.end());
        std::copy(p1.begin(), p1.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p1.size()));
        goodguy::makeActionPacket(p1, goodguy::Motor::DYNAMIXEL_PRO);
        goodguy::printPacket(p1.begin(), p1.end());
        std::copy(p1.begin(), p1.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p1.size()));


    while(1)
    {
        float torque = 0;
        if(state1.m_is_moving){
            if(std::abs(state1.m_angle-m1.getGoalAngle()) < 10){
                torque = 0;
            }
            else if(state1.m_speed < 0){
                torque = 100;
            }
            else{
                torque = -100;
            }
        }

        std::cout << "set torque: " << torque << " " << state1.m_current*33000  << std::endl;
        m2.setGoalTorqueWithUpdate(torque);
        goodguy::makeMotorPacket(p2, m2);
        std::copy(p2.begin(), p2.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
        m1.setGoalTorqueWithUpdate(100);
        goodguy::makeMotorPacket(p2, m1);
        std::copy(p2.begin(), p2.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p2.size()));
        goodguy::makeActionPacket(p2, goodguy::Motor::DYNAMIXEL_PRO);
        std::copy(p2.begin(), p2.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, p2.size()));


        usleep(5000);

        //std::cout << "Send Default Bulk Read Packet" << std::endl;

        std::vector<goodguy::Motor> motors;
        motors.push_back(m1);
        motors.push_back(m2);

        std::vector<unsigned char> packet;
        goodguy::makeDefaultBulkReadPacket(packet, motors);

        std::copy(packet.begin(), packet.end(), send_buffer);
        s1.write_some(boost::asio::buffer(send_buffer, packet.size()));

        
        std::vector<unsigned char> received_buffer;

        std::string header;
        header.push_back(0xff);
        header.push_back(0xff);
        header.push_back(0xfd);

        
        
        

        boost::array<char, 128> buf = {0};
        std::size_t received_size = s1.read_some(boost::asio::buffer(buf));
        received_buffer.insert(received_buffer.end(), buf.begin(), buf.begin()+received_size);
        std::vector<std::string> splited_packets;

        boost::split_regex(splited_packets, received_buffer, boost::regex(header));
        received_buffer.clear();

        for(auto it = splited_packets.begin(); it != splited_packets.end(); ++it){

            std::vector<unsigned char> pk(header.begin(), header.end());
            pk.insert(pk.end(), it->begin(), it->end());

            std::vector<unsigned char> packet_data;
            int packet_id = 0;
            goodguy::DynamixelProStatusType packet_error;
            try{
                if(goodguy::getDataForDynamixelPro(packet_data, packet_id, packet_error, pk)){
                    //std::cout << "[OK] Is Valid: " << packet_id << "\t" << packet_error << std::endl;
                    //goodguy::printPacket(packet_data.begin(), packet_data.end());
                    //goodguy::printPacket(pk.begin(), pk.end());

                    goodguy::DefaultBulkState state = getDefaultBulkStateFromStream(packet_data, goodguy::Motor::DYNAMIXEL_PRO);

                    if(packet_id == 1){
                        state1 = state;
                    }
                }
            } catch (goodguy::InvalidPacketException& e){
                //std::cout << "[FAIL] Is not Valid" << std::endl;
                if(it == (splited_packets.end()-1)){
                    received_buffer.insert(received_buffer.begin(), pk.begin(), pk.end());
                }

            }
        }


        //sleep(1);

    }


    std::cout << "EXIT TEST" << std::endl;


    return 0;
}
