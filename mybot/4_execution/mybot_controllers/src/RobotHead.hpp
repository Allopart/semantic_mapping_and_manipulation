#pragma once

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <string>
#include <iostream>

#define BUFFER_LENGTH 256

namespace plaincode {

    template <class SOCKET>
        class RobotHead {
            public:
                typedef std::vector<unsigned char> Packet;
                typedef std::vector<unsigned short> Angle;
            public:
                RobotHead(SOCKET& socket):m_socket(socket), active_(false) {
                    if (m_socket.is_open()) {
                        active_ = true;
                    }

                    minAngle.resize(4);
                    maxAngle.resize(4);
                    minAngle = {1650,1650,1650,1000};
                    maxAngle = {1950,2400,1950,2600};
                    reset();
                    start();
                }


                void reset() {
                    sendPacket.resize(12);
                    sendAngle.resize(4);
                    recvAngle.resize(4);
                    header.resize(3);

                    fullPacket.clear();
                    recvPacket.clear();

                    // sendAngle, recvAngle을 1800, 1800, 1800, 1800으로 초기화
                    std::fill(sendAngle.begin(), sendAngle.end(), 1800);
                    std::fill(recvAngle.begin(), recvAngle.end(), 1800);

                    header[0]=0xff;
                    header[1]=0xff;
                    header[2]=0x01;

                    sendPacket[0] = 0xff;
                    sendPacket[1] = 0xff;
                    sendPacket[2] = 0x01;
                    sendPacket[11]= 0x01;

                    for (auto i=0;i<sendAngle.size(); i++) {
                        sendPacket[3+i*2]=sendAngle[i]&0xff;
                        sendPacket[4+i*2]=sendAngle[i]>>8;
                    }

                    write(sendPacket);
                }

                void setCurrentAngles(const std::vector<double>& angles){
                    Angle setting_angles(4,0);
                    for(auto it = setting_angles.begin(); it != setting_angles.end(); ++it){
                        *it = (short)((angles[it-setting_angles.begin()]*180.0*10/3.14)+1800.0);
                    }

                    Angle remapped_setting_angles(4,0);
                    remapped_setting_angles[0] = setting_angles[2];
                    remapped_setting_angles[1] = setting_angles[1];
                    remapped_setting_angles[2] = setting_angles[3];
                    remapped_setting_angles[3] = setting_angles[0];
                    manualControl(remapped_setting_angles);

                }

                std::vector<double> getCurrentAngles() const{
                    std::vector<double> angles(4,0);
                    for(auto it = recvAngle.begin(); it != recvAngle.end(); ++it){
                        angles[it-recvAngle.begin()] = (((double)(*it))-1800.0)/10.0/180.0*3.14;
                    }
                    std::vector<double> remapped_angles(4,0);
                    remapped_angles[0] = angles[3];
                    remapped_angles[1] = -angles[1];
                    remapped_angles[2] = -angles[0];
                    remapped_angles[3] = angles[2];

                    return remapped_angles;
                }

            private:

                void write(const std::vector<unsigned char>& data){

                    for (std::size_t i=0; i<data.size(); i++) {
                        send_buffer_[i] = data[i];
                    }
                    m_socket.async_write_some(boost::asio::buffer(send_buffer_, data.size()),
                            boost::bind(&RobotHead::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                }

                void manualControl(Angle& angle){

                    for (auto i=0; i<angle.size(); i++) if (angle[i]<minAngle[i]) angle[i]=minAngle[i];
                    for (auto i=0; i<angle.size(); i++) if (angle[i]>maxAngle[i]) angle[i]=maxAngle[i];


                    for (auto i=0;i<sendAngle.size(); i++) {
                        sendPacket[3+i*2]=angle[i]&0xff;
                        sendPacket[4+i*2]=angle[i]>>8;
                    }

                    write(sendPacket);
                }


                bool validatePacket(Packet& packet, int headerSize = 3){
                    if (packet.size()!=12) return false;

                    // check sum
                    int sum = std::accumulate(packet.begin()+headerSize, packet.end(), 0);
                    if (sum%256==0) return true;
                    return false;


                }
                bool readPacket(Packet& fullPacket, Packet& packet, Packet& header){
                    if (fullPacket.size()<header.size()) return false;

                    for (int i=0; i<=fullPacket.size()-header.size(); i++) {
                        if (std::equal(fullPacket.begin()+i, fullPacket.begin()+i+header.size(),
                                    header.begin())) {
                            packet.clear();
                            packet.assign(fullPacket.begin(), fullPacket.begin()+i);
                            fullPacket.erase(fullPacket.begin(), fullPacket.begin()+i);
                            if (validatePacket(packet)) return true;
                        }
                    }
                    return false;
                }
            private:
                void start(){
                    std::cout << "Start, Serial Communication!" << std::endl;

                    // serial 통신이 시작되었으면, 언제든 얼굴로봇으로 부터 값을 받아올 준비가 되어 있도록 설정.
                    m_socket.async_read_some(boost::asio::buffer(recv_buffer_, BUFFER_LENGTH),
                            boost::bind(&RobotHead::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

                }
                void handle_write(const boost::system::error_code& error, size_t bytes_transferred){
                    if (!error) {
                        memset(send_buffer_, 0, BUFFER_LENGTH);
                    }
                    else{
                    }


                }
                void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred){
                    fullPacket.insert(fullPacket.end(), recv_buffer_, recv_buffer_+bytes_transferred);

                    if (readPacket(fullPacket, recvPacket, header)) {

                        for (auto i=0; i<recvAngle.size(); i++){
                            recvAngle[i] = (recvPacket[3+i*2])+(recvPacket[4+i*2]<<8);
                        }
                    }

                    m_socket.async_read_some(boost::asio::buffer(recv_buffer_, BUFFER_LENGTH),
                            boost::bind(&RobotHead::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));


                }
            private:
                SOCKET& m_socket;
                bool active_;
                unsigned char send_buffer_[BUFFER_LENGTH];
                unsigned char recv_buffer_[BUFFER_LENGTH];
                std::string buffer;

                Packet fullPacket;
                Packet sendPacket;
                Packet recvPacket;
                Packet header;

                Angle sendAngle;
                Angle recvAngle;
                Angle minAngle; // robotic head의 DC모터가 가질 수 있는 최소각도
                Angle maxAngle; // robotic head의 DC모터가 가질 수 있는 최대각도
                boost::thread thread;
        };
}

