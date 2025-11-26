#ifndef ORIGIN_HARDWARE_HARDWARE_COMMS_HPP
#define ORIGIN_HARDWARE_HARDWARE_COMMS_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
        case 1200:   return LibSerial::BaudRate::BAUD_1200;
        case 1800:   return LibSerial::BaudRate::BAUD_1800;
        case 2400:   return LibSerial::BaudRate::BAUD_2400;
        case 4800:   return LibSerial::BaudRate::BAUD_4800;
        case 9600:   return LibSerial::BaudRate::BAUD_9600;
        case 19200:  return LibSerial::BaudRate::BAUD_19200;
        case 38400:  return LibSerial::BaudRate::BAUD_38400;
        case 57600:  return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cerr << "Unsupported baud rate " << baud_rate << ", defaulting to 57600." << std::endl;
            return LibSerial::BaudRate::BAUD_57600;
    }
}

class HardwareComms
{
public:
    HardwareComms() = default;

    // Attempt to open the serial connection. Exceptions from the underlying
    // library are caught and converted to a false return value so callers
    // can handle failures without the process aborting.
    bool connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        try {
            serial_conn_.Open(serial_device);
            serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
            connected_ = true;
            return true;
        } catch (const LibSerial::OpenFailed & e) {
            std::cerr << "HardwareComms: failed to open serial device '" << serial_device << "': " << e.what() << std::endl;
            connected_ = false;
            return false;
        } catch (const std::exception & e) {
            std::cerr << "HardwareComms: unexpected error opening serial device '" << serial_device << "': " << e.what() << std::endl;
            connected_ = false;
            return false;
        } catch (...) {
            std::cerr << "HardwareComms: unknown error opening serial device '" << serial_device << "'" << std::endl;
            connected_ = false;
            return false;
        }
    }

    void disconnect()
    {
        try {
            if (serial_conn_.IsOpen()) {
                serial_conn_.Close();
            }
        } catch (const std::exception & e) {
            std::cerr << "HardwareComms: error while closing serial device: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "HardwareComms: unknown error while closing serial device" << std::endl;
        }
        connected_ = false;
    }

    bool connected() const
    {
        return connected_ || serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        std::string response;
        try {
            serial_conn_.FlushIOBuffers();
            serial_conn_.Write(msg_to_send);
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        } catch (const LibSerial::ReadTimeout & ) {
            std::cerr << "HardwareComms: serial read timed out." << std::endl;
        } catch (const std::exception & e) {
            std::cerr << "HardwareComms: serial IO error: " << e.what() << std::endl;
            connected_ = false;
        } catch (...) {
            std::cerr << "HardwareComms: unknown serial IO error" << std::endl;
            connected_ = false;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " | Received: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg()
    {
        send_msg("\r");
    }

    void read_encoder_values(int &val_1, int &val_2, int &val_3)
    {
        std::string response = send_msg("e\r");

        std::string delimiter = " ";
        size_t first_del_pos  = response.find(delimiter);
        size_t second_del_pos = response.find(delimiter, first_del_pos + 1);

        std::string token_1 = response.substr(0, first_del_pos);
        std::string token_2 = response.substr(first_del_pos + 1, second_del_pos - first_del_pos - 1);
        std::string token_3 = response.substr(second_del_pos + 1);

        val_1 = std::atoi(token_1.c_str());
        val_2 = std::atoi(token_2.c_str());
        val_3 = std::atoi(token_3.c_str());
    }

    void set_motor_values(int val_1, int val_2, int val_3)
    {
        std::stringstream ss;
        ss << "m " << val_1 << " " << val_2 << " " << val_3 << "\r";
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_{100};
    bool connected_{false};
};

#endif // ORIGIN_HARDWARE_HARDWARE_COMMS_HPP
