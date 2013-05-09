#ifndef __SERIALCHANNEL_HPP
#define __SERIALCHANNEL_HPP

#include <string>
#include <vector>
#include <cstddef>

class SerialChannel {
public:

    typedef unsigned int         uint;
    typedef unsigned char        byte_t;
    typedef std::vector<byte_t>  chunk_t;

    SerialChannel(const std::string &def, uint input_dgram_size = 0);

    void            open     ();
    void            close    ();
                    
    void            send     (const byte_t *buffer, size_t size);

    const chunk_t   retrieve (size_t max_bytes = 0);

private:
    std::string                 def;
    //uint                        input_dgram_size;
    void                       *_intern;
};

#endif // __SERIAL_CHANNEL_HPP