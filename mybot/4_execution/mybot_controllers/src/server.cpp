#include "server.hpp"

namespace goodguy{

    void session::write(const std::vector<unsigned char>& src){

        //boost::lock_guard<boost::mutex> lck(mutex_for_send);
        std::string end_indicator("!!!END!!!");

        mutex_for_read.lock();
        is_received_ack = false;
        mutex_for_read.unlock();

        send_buffer.clear();

        send_buffer.insert(send_buffer.end(), src.begin(), src.end());
        send_buffer.insert(send_buffer.end(), end_indicator.begin(), end_indicator.end());

        std::size_t remained_size  = send_buffer.size();

        int start = 0;
        int end = -1;

        while( remained_size != 0 ){
            start = end + 1;

            if(remained_size > BUFFER_LENGTH){
                end = start + BUFFER_LENGTH -1; 
                remained_size -= BUFFER_LENGTH;
            }
            else{
                end = start + remained_size -1;
                remained_size -= remained_size;
            }

            std::vector<unsigned char> send_buffer_slice(send_buffer.begin()+start, send_buffer.begin()+end+1);
            socket.write_some(boost::asio::buffer(send_buffer_slice.data(), send_buffer_slice.size()));
        }



    }



    void session::handle_write(const boost::system::error_code& error){
        boost::lock_guard<boost::mutex> lck(mutex_for_send);
        if(!error){

            
            std::size_t length = send_buffer.size();

            //std::cout << "WRITE: " << offset << "/ " << length << std::endl;

            if(((length - offset) < BUFFER_LENGTH) && ((length - offset) > 0)){

                for( std::size_t i = 0; i < (length - offset); ++i){
                    send_data[i] = send_buffer[i+offset];
                }


                offset += length - offset;	
                socket.async_write_some(boost::asio::buffer(send_data,(length - offset)),
                        boost::bind(&session::handle_write, this, boost::asio::placeholders::error));

            }
            else if(((length - offset) >= BUFFER_LENGTH)){

                for( std::size_t i = 0; i < BUFFER_LENGTH; ++i){
                    send_data[i] = send_buffer[i+offset];
                }


                offset += BUFFER_LENGTH;	
                socket.async_write_some(boost::asio::buffer(send_data,BUFFER_LENGTH),
                        boost::bind(&session::handle_write, this, boost::asio::placeholders::error));

            }
            else{

                send_buffer.clear();
                offset = 0;
                is_sending = false;
                std::cout << "Transmission Success !" << std::endl;
            }

        }
        else{

        }

    }

    void session::handle_read(const boost::system::error_code& error, size_t bytes_transferred){
        //std::cout<< "HANDLE READ"<< std::endl;
        boost::lock_guard<boost::mutex> lck(mutex_for_read);
        if(!error){

            for(std::size_t i = 0; i < bytes_transferred; ++i){
                received_buffer.push_back(data_[i]);
                data_[i] = 0;
            }

            std::string end_indicator("!!!END!!!");
            std::vector<std::string> splited_packet;
            boost::split_regex(splited_packet, received_buffer, boost::regex(end_indicator));

            if(splited_packet.size() > 1){

                std::cout << splited_packet.size() << std::endl;
                received_buffer.clear();

                for(auto s = splited_packet.begin(); s!= splited_packet.end(); ++s){
                    if(s == (splited_packet.end())){
                        received_buffer.insert(received_buffer.end(), s->begin(), s->end());
                    }
                    else{
                        if(*s == std::string("ACK")){
                            is_received_ack = true;
                        }
                    }
                }
            }


            socket.async_read_some(boost::asio::buffer(data_,BUFFER_LENGTH),
                    boost::bind(&session::handle_read, this, 
                        boost::asio::placeholders::error, 
                        boost::asio::placeholders::bytes_transferred));
        }
        else{
            *target_ = NULL;
            //delete this;
        }


    }


};

