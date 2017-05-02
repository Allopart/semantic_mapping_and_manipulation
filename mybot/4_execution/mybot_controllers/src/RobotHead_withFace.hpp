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

//                    minAngle.resize(4);
//                    maxAngle.resize(4);
//                    minAngle = {1650,1650,1650,1200};
//                    maxAngle = {1950,2400,1950,2400};

                    min_angles_dc.resize(4);
                    max_angles_dc.resize(4);
                    min_angles_dc = {-60,-15,-15,-15};
                    max_angles_dc = {60,60,15,15};

                    min_angles_servo.resize(4);
                    max_angles_servo.resize(4);
                    min_angles_servo = {-45,-45,-45,-45};
                    max_angles_servo = {45,45,45,45};

                    init_face_0.resize(4);
                    init_face_1.resize(4);
                    init_face_2.resize(4);
                    init_face_0 = {39680, 28160, 35072, 27392};
                    init_face_1 = {35840, 13568, 13568, 13568};
                    init_face_2 = {32000, 38912, 35072, 30310.4};

                    reset();
                    //start();
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

					//reset facial expression

					// packet 0 for lip
                    sendPacket_face_0.resize(12);
                    sendAngle_face_0.resize(4);

                    sendPacket_face_0[0] = 0xff;
                    sendPacket_face_0[1] = 0xff;
                    sendPacket_face_0[2] = 0x01;
	
					// initial angles for facial expression
					sendPacket_face_0[3] = 0x00;
					sendPacket_face_0[4] = 0x9B;
					sendPacket_face_0[5] = 0x00;
					sendPacket_face_0[6] = 0x6E;
					sendPacket_face_0[7] = 0x00;
					sendPacket_face_0[8] = 0x89;
					sendPacket_face_0[9] = 0x00;
					sendPacket_face_0[10] = 0x6B;

                    sendPacket_face_0[11]= 0x02;

					// packet 1 for chin
                    sendPacket_face_1.resize(12);
                    sendAngle_face_1.resize(4);

                    sendPacket_face_1[0] = 0xff;
                    sendPacket_face_1[1] = 0xff;
                    sendPacket_face_1[2] = 0x01;
	
					// initial angles for facial expression
					sendPacket_face_1[3] = 0x00;
					sendPacket_face_1[4] = 0x8C;
					sendPacket_face_1[5] = 0x00;
					sendPacket_face_1[6] = 0x35;
					sendPacket_face_1[7] = 0x00;
					sendPacket_face_1[8] = 0x35;
					sendPacket_face_1[9] = 0x00;
					sendPacket_face_1[10] = 0x35;

                    sendPacket_face_1[11]= 0x03;

					// packet 2 for eyebrow
                    sendPacket_face_2.resize(12);
                    sendAngle_face_2.resize(4);

                    sendPacket_face_2[0] = 0xff;
                    sendPacket_face_2[1] = 0xff;
                    sendPacket_face_2[2] = 0x01;
	
					// initial angles for facial expression
					sendPacket_face_2[3] = 0x00;
					sendPacket_face_2[4] = 0x7D;
					sendPacket_face_2[5] = 0x00;
					sendPacket_face_2[6] = 0x98;
					sendPacket_face_2[7] = 0x00;
					sendPacket_face_2[8] = 0x89;
					sendPacket_face_2[9] = 0x66;
					sendPacket_face_2[10] = 0x76;

                    sendPacket_face_2[11]= 0x04;

                    //write(sendPacket);
                }

                void setCurrentAngles(const std::vector<double>& angles, std::vector<double>& min_angles, std::vector<double>& max_angles){
                    Angle setting_angles(4,0);
                    for(auto it = setting_angles.begin(); it != setting_angles.end(); ++it){
                        if (angles[it-setting_angles.begin()]*180.0/3.14 < min_angles[it-setting_angles.begin()]) {
                            *it = (short)((min_angles[it-setting_angles.begin()]*180.0*10/3.14)+1800.0);
                        } 
                        else if (angles[it-setting_angles.begin()]*180.0/3.14 > max_angles[it-setting_angles.begin()]) {
                            *it = (short)((max_angles[it-setting_angles.begin()]*180.0*10/3.14)+1800.0);
                        } 
                        else {
                            *it = (short)((angles[it-setting_angles.begin()]*180.0*10/3.14)+1800.0);
                        }
                    }

                    Angle remapped_setting_angles(4,0);
                    remapped_setting_angles[0] = setting_angles[2];
                    remapped_setting_angles[1] = setting_angles[1];
                    remapped_setting_angles[2] = setting_angles[3];
                    remapped_setting_angles[3] = setting_angles[0];
                    manualControl(remapped_setting_angles, sendAngle, sendPacket);

                }

				//set current angles func for facial expression
				void setCurrentAngles_face(const std::vector<double>& angles, std::vector<double>& init_angles, std::vector<double>& min_angles, std::vector<double>& max_angles, Angle& sendAngle_, Packet& sendPacket_) {
					Angle setting_angles(4, 0);
					for(auto it = setting_angles.begin(); it != setting_angles.end(); ++it) {
	                    if (angles[it-setting_angles.begin()]*180.0/3.14 < min_angles[it-setting_angles.begin()]) {
                            *it = (short)((min_angles[it-setting_angles.begin()]*180.0/3.14 + 128) * 256 - 32768 + init_angles[it-setting_angles.begin()]);
                        } 
                        else if (angles[it-setting_angles.begin()]*180.0/3.14 > max_angles[it-setting_angles.begin()]) {
                            *it = (short)((max_angles[it-setting_angles.begin()]*180.0/3.14 + 128) * 256 - 32768 + init_angles[it-setting_angles.begin()]);
                        } 
                        else {
             				*it = (short)((angles[it-setting_angles.begin()]*180.0/3.14 + 128) * 256 - 32768 + init_angles[it-setting_angles.begin()]);
                        }
					}

					manualControl(setting_angles, sendAngle_, sendPacket_);
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
                /*
                   void write(const std::vector<unsigned char>& data){

                   for (std::size_t i=0; i<data.size(); i++) {
                   send_buffer_[i] = data[i];
                   }
                   m_socket.async_write_some(boost::asio::buffer(send_buffer_, data.size()),
                   boost::bind(&RobotHead::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                   }
                 */

				// editted for facial expression
                void manualControl(Angle& angle, Angle& sendAngle_, Packet& sendPacket_){

//                    for (auto i=0; i<angle.size(); i++) if (angle[i]<minAngle_[i]) angle[i]=minAngle_[i];
//                    for (auto i=0; i<angle.size(); i++) if (angle[i]>maxAngle_[i]) angle[i]=maxAngle_[i];


                    for (auto i=0;i<sendAngle_.size(); i++) {
                        sendPacket_[3+i*2]=angle[i]&0xff;
                        sendPacket_[4+i*2]=angle[i]>>8;
                    }

                    //write(sendPacket);
                }

                bool validatePacket(Packet& packet, int headerSize = 3){
                    if (packet.size()!=12) return false;

                    // check sum
                    int sum = std::accumulate(packet.begin()+headerSize, packet.end(), 0);
                    if (sum%256==0) return true;
                    return false;


                }
                /*		
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


                }*/
            private:
                SOCKET& m_socket;
                bool active_;
                //unsigned char send_buffer_[BUFFER_LENGTH];
                //unsigned char recv_buffer_[BUFFER_LENGTH];
                //std::string buffer;

            public:
				// for neck DC motor
                Packet fullPacket; 
                Packet sendPacket;
                Packet recvPacket;
                
				Packet header;

				// for facial expression
				Packet sendPacket_face_0;
				Packet sendPacket_face_1;
				Packet sendPacket_face_2;

				// for neck DC motor
                Angle sendAngle;
                Angle recvAngle;
//                Angle minAngle; // robotic head의 DC모터가 가질 수 있는 최소각도
 //               Angle maxAngle; // robotic head의 DC모터가 가질 수 있는 최대각도
                std::vector<double> min_angles_dc;
                std::vector<double> max_angles_dc;

				// for facial expression
				Angle sendAngle_face_0;
    			Angle sendAngle_face_1;
        		Angle sendAngle_face_2;
                
                std::vector<double> min_angles_servo;
                std::vector<double> max_angles_servo;

                std::vector<double> init_face_0;
                std::vector<double> init_face_1;
                std::vector<double> init_face_2;
 
				//boost::thread thread;
        };
}

