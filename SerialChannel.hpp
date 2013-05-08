#ifndef __SERIALCHANNEL_HPP
#define __SERIALCHANNEL_HPP

#include <string>
#include <vector>
#include <cstddef>

class SerialChannel {
public:

    typedef unsigned char byte_t;

    SerialChannel(const std::string &def);

    void                        open     ();
    void                        close    ();

    void                        send     (const byte_t *buffer, size_t size);

    const std::vector<byte_t>   retrieve (size_t max_bytes = 0);

private:
    std::string                 def;
    void                       *_intern;
};

#endif // __SERIAL_CHANNEL_HPP