#include "pugixml.hpp"
#include "server.hpp"
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>


using boost::asio::ip::tcp;
boost::asio::io_service io;
tcp::socket socket_for_mobile(io);

char recv_buffer[5];

struct xml_string_writer: pugi::xml_writer{
    std::string result;

    virtual void write(const void* data, size_t size){
        result.append(static_cast<const char*>(data), size);
    }
};


void handle_read(const boost::system::error_code& error, size_t bytes_transferred){
    std::cout << "HANDLE READ: " << bytes_transferred  << std::endl;
    if(!error){

        std::cout << "RECEIVED: " << bytes_transferred << std::endl;

        socket_for_mobile.async_read_some(boost::asio::buffer(recv_buffer,5),
                boost::bind(&handle_read, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else{
        std::cout << "READ ERROR: " << error << std::endl;

    }
}



int main(){

    int port_mobile = 50010;


    tcp::resolver resolver_mobile(io);
    tcp::resolver::query query_for_mobile(tcp::v4(), "192.168.51.10", boost::lexical_cast<std::string>(port_mobile));
    tcp::resolver::iterator iterator_for_mobile = resolver_mobile.resolve(query_for_mobile);

    boost::thread thread_for_asio([&]() { io.run(); });

    while(1){

        char key;
        std::cout << "1. Servo On, 2. Servo Off, 3. Move Forward, 4. Stop, 5. Move Backward, 6. Left Rotate, 7. Right Rotate" << std::endl;
        std::cin >> key;
        socket_for_mobile.close();
        try{
            boost::asio::connect(socket_for_mobile, iterator_for_mobile);
            std::cout << "Connection Success" << std::endl;
            socket_for_mobile.async_read_some(boost::asio::buffer(recv_buffer,5),
                    boost::bind(handle_read, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        }catch(std::exception& e){
            std::cerr << "Exception: " << e.what() << std::endl;
            std::cout << "Connection failed" << std::endl;
            return 0;
        }

        if(key == '1'){
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("ServoOn");
            xml_string_writer string_writer;
            doc.save(string_writer); 

            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));

        }
        else if(key == '2'){

            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("ServoOff");
            xml_string_writer string_writer;
            doc.save(string_writer); 

            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
        }
        else if(key == '3'){

            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("VelocityControl");
            node = node.parent();
            node = node.append_child("method_datalist_arg"); 
            node = node.append_child("datalist"); 
            node = node.append_child("data").append_child("int");
            node.text().set("100");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("100");

            xml_string_writer string_writer;
            doc.save(string_writer); 
            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
        }
        else if(key == '4'){
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("VelocityControl");
            node = node.parent();
            node = node.append_child("method_datalist_arg"); 
            node = node.append_child("datalist"); 
            node = node.append_child("data").append_child("int");
            node.text().set("0");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("0");

            xml_string_writer string_writer;
            doc.save(string_writer); 
            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
        }
        else if(key == '5'){
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("VelocityControl");
            node = node.parent();
            node = node.append_child("method_datalist_arg"); 
            node = node.append_child("datalist"); 
            node = node.append_child("data").append_child("int");
            node.text().set("-100");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("-100");

            xml_string_writer string_writer;
            doc.save(string_writer); 
            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
        }
        else if(key == '6'){
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("VelocityControl");
            node = node.parent();
            node = node.append_child("method_datalist_arg"); 
            node = node.append_child("datalist"); 
            node = node.append_child("data").append_child("int");
            node.text().set("-100");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("100");

            xml_string_writer string_writer;
            doc.save(string_writer); 
            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
        }
        else if(key == '7'){
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("VelocityControl");
            node = node.parent();
            node = node.append_child("method_datalist_arg"); 
            node = node.append_child("datalist"); 
            node = node.append_child("data").append_child("int");
            node.text().set("100");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("-100");

            xml_string_writer string_writer;
            doc.save(string_writer); 
            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> sended_packet_buffer_for_mobile;
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), start.begin(), start.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
            sended_packet_buffer_for_mobile.clear();
            sended_packet_buffer_for_mobile.insert(sended_packet_buffer_for_mobile.end(), string_writer.result.begin(), string_writer.result.end());
            socket_for_mobile.write_some(boost::asio::buffer(sended_packet_buffer_for_mobile.data(), sended_packet_buffer_for_mobile.size()));
        }
        else if(key == 'q'){
            break;
        }
        else{
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("add");
            node = node.parent();
            node = node.append_child("method_datalist_arg"); 
            node = node.append_child("datalist"); 
            node = node.append_child("data").append_child("int");
            node.text().set("1");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("1");
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set("1");
            node = node.root();
            xml_string_writer string_writer;
            node.print(string_writer);
            //node.print(string_writer, "\r\n",  pugi::format_raw | pugi::format_no_declaration);

            std::cout << string_writer.result << std::endl;

        }


    }



    thread_for_asio.join();


    return 0;
}
