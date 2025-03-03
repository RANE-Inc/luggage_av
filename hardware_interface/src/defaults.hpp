#include <cstdint>
#include <termios.h>

namespace luggage_av {

    struct {
        char* dev = const_cast<char*>("/dev/ttyACM0");
        unsigned int baud = B115200;
        int32_t hw_cmd_min = -8312;
        int32_t hw_cmd_max = 8312;
    } luggage_av_default_parameters;

}  // namespace luggage_av
