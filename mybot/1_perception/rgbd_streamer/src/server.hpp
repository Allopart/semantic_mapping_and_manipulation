#ifndef	__SERVER_HPP__
#define __SERVER_HPP__

#include <iostream>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <list>
#include <thread>
#include <mutex>
#include <string>


namespace goodguy{

    const std::size_t BUFFER_LENGTH = 1024*1024;
    const std::size_t MAX_CONNECTIONS = 1024;


    using boost::asio::ip::tcp;

    class session{

    public:
        session(boost::asio::io_service& io_service, session** target)
            : target_(target), is_sending(false), is_received_ack(true), socket(io_service), offset(0)
        {

        }

        ~session(){

        }

        tcp::socket& getLocalSocket(){
            return socket;
        }


        void clear(){

            try{

            }catch(std::exception& e){
                //	std::cout << "ERROR on termination session: " << e.what() << std::endl;

            }

        }



        void start(){

            socket.async_read_some(boost::asio::buffer(data_,BUFFER_LENGTH),
                boost::bind(&session::handle_read, this, 
                boost::asio::placeholders::error, 
                boost::asio::placeholders::bytes_transferred));

        }

        void write(const std::vector<unsigned char>& src);


        std::vector<unsigned char> getReceivedBuffer(){
            std::vector<unsigned char> temp;
            mutex_for_read.lock();
            temp.insert(temp.begin(), received_buffer.begin(), received_buffer.end());
            received_buffer.clear();
            mutex_for_read.unlock();

            return temp;
        }

        bool isAck(){
            mutex_for_read.lock();
            bool is_ack = is_received_ack;
            mutex_for_read.unlock();
            return is_ack;
        }




    private:

        void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
        void handle_write(const boost::system::error_code& error);

        void close(){
            socket.close();
        }


    private:

        boost::mutex mutex_for_send;
        boost::mutex mutex_for_read;

        session** target_;

        bool is_sending;

        bool is_received_ack;


        tcp::socket socket;

        int offset;

        unsigned char data_[BUFFER_LENGTH];
        unsigned char send_data[BUFFER_LENGTH];

        std::vector<unsigned char> received_buffer;
        std::vector<unsigned char> send_buffer;
    };



    class server{

    public:
        server(boost::asio::io_service& io_service, unsigned int port_num, session** target)
            : io_service_(io_service), target_(target), acceptor_(io_service, tcp::endpoint(tcp::v4(), port_num)), port_num_(port_num)
        {
            startAccept();
        }

    private:
        void startAccept(){
            std::cout << "START SERVER, Port NUM: " << port_num_ << std::endl;
            session* new_session = new session(io_service_, target_);
            acceptor_.async_accept(new_session->getLocalSocket(),
                boost::bind(&server::handleAccept, this, new_session, boost::asio::placeholders::error));
        }


        void handleAccept(session* new_session, const boost::system::error_code& error){
            if(!error){
                new_session->start();
                *target_ = new_session;
            }
            else{
                delete new_session;
            }

            startAccept();
        }

    private:

        boost::asio::io_service& io_service_;
        session** target_;
        tcp::acceptor acceptor_;
        unsigned int port_num_;
    };

};

#endif
