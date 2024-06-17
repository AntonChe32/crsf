#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <array>
#include <cstdint>
#include <cstring>

using namespace boost::asio;

#define PACKED __attribute__((packed))
#define CRSF_SYNC_BYTE 0xC8
#define RCframeLength 22 // 16 channels in 11 bits each
#define CRSF_PAYLOAD_SIZE_MAX 512

struct crsf_header_t {
//    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
} PACKED;

struct crsf_channels_t {
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED;

struct rcPacket_t {
    crsf_header_t header;
    crsf_channels_t channels;
} PACKED;

class SerialReader {
public:
    SerialReader(io_service& io, const std::string& port)
        : serial_(io, port),
          read_position_(0)
    {
        serial_.set_option(serial_port_base::baud_rate(460800));
        readStart();
    }

    void readStart() {
        serial_.async_read_some(
            buffer(read_msg_),
            boost::bind(
                &SerialReader::readComplete,
                this,
                placeholders::error,
                placeholders::bytes_transferred
            )
        );
    }

    void readComplete(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!error) {
            for (size_t i = 0; i < bytes_transferred; ++i) {
                processByte(read_msg_[i]);
            }
            readStart();
        } else {
            std::cerr << "Error: " << error.message() << std::endl;
        }
    }
    // Парсим прото
    void processByte(uint8_t byte) {
        switch (state_) {
            case State::WAIT_SYNC:
                // Пришедший байт является байтом синхронизации?
                if (byte == CRSF_SYNC_BYTE) {
                    // Если это так то перейти в фазу чтения хедера пакета
                    state_ = State::READ_HEADER;
                    header_position_ = 0;
                }
                break;
            case State::READ_HEADER:
                // Читаем хедер
                ((u_int8_t*)&current_header_)[header_position_++] = byte;
                if (header_position_ == sizeof(crsf_header_t)) {
                    // если хедер считан, инициализируем длину активной части пакета
                    payload_length_ = current_header_.frame_size - 2; // frame_size - type - crc
                    // Переходим к фазе загрузки каналов
                    state_ = State::READ_PAYLOAD;
                    payload_position_ = 0;
                }
                break;
            case State::READ_PAYLOAD:
                // Если количество считанных байт меньше размера загружаемой полезной части
                if (payload_position_ < payload_length_) {
                    // продолжаем сохронять данные в буфер
                    payload_buffer_[payload_position_++] = byte;
                }
                // Если количество считанных байт равно размеру загружаемой полезной части
                if (payload_position_ == payload_length_) {
                    // Если тип кадра есть несущий каналы
                    if (current_header_.type == 0x16) { // CRSF_FRAMETYPE_RC_CHANNELS_PACKED
                        // Перейти к обработке каналов
                        processChannels(payload_buffer_, payload_length_);
                    }
                    // Вернуться в состояние ожидания синхронизации
                    state_ = State::WAIT_SYNC;
                }
                break;
        }
    }

    void processChannels(const uint8_t* payload, size_t length) {
        if (length >= sizeof(crsf_channels_t)) {
            crsf_channels_t channels;
            memcpy(&channels, payload, sizeof(crsf_channels_t));

            std::cout << "Channel 1 value: " << channels.ch0 << std::endl;
            std::cout << "Channel 2 value: " << channels.ch1 << std::endl;
            std::cout << "Channel 3 value: " << channels.ch2 << std::endl;
            std::cout << "Channel 4 value: " << channels.ch3 << std::endl;
            std::cout << "Channel 5 value: " << channels.ch4 << std::endl;
            std::cout << "Channel 6 value: " << channels.ch5 << std::endl;
            std::cout << "Channel 7 value: " << channels.ch6 << std::endl;
            std::cout << "Channel 8 value: " << channels.ch7 << std::endl;
            std::cout << "Channel 9 value: " << channels.ch8 << std::endl;
            std::cout << "Channel 10 value: " << channels.ch9 << std::endl;
            std::cout << "Channel 11 value: " << channels.ch10 << std::endl;
            std::cout << "Channel 12 value: " << channels.ch11 << std::endl;
            std::cout << "Channel 13 value: " << channels.ch12 << std::endl;
            std::cout << "Channel 14 value: " << channels.ch13 << std::endl;
            std::cout << "Channel 15 value: " << channels.ch14 << std::endl;
            std::cout << "Channel 16 value: " << channels.ch15 << std::endl;
        }
    }

private:
    serial_port serial_;
    enum { max_length = 512 };
    char read_msg_[max_length];
    
    enum class State { WAIT_SYNC, READ_HEADER, READ_PAYLOAD };
    State state_ = State::WAIT_SYNC;

    crsf_header_t current_header_;
    uint8_t header_buffer_[sizeof(crsf_header_t)];
    size_t header_position_ = 0;

    size_t payload_length_ = 0;
    size_t payload_position_ = 0;
    uint8_t payload_buffer_[CRSF_PAYLOAD_SIZE_MAX];

    size_t read_position_;
};

int main() {
    try {
        io_service io;
        SerialReader reader(io, "/dev/ttyUSB0");
        io.run();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
