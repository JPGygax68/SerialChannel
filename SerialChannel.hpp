#ifndef __SERIALCHANNEL_HPP
#define __SERIALCHANNEL_HPP

/*
The MIT License (MIT)

Copyright (c) 2013 Jean-Pierre Gygax, Biel-Bienne, Switzerland

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <string>
#include <vector>
#include <cstddef>

/**
 *  The SerialChannel class implements bidirectional communication on a serial (COM) port.
 */
class SerialChannel {
public:

    typedef unsigned int         uint;
    typedef unsigned char        byte_t;
    typedef std::vector<byte_t>  chunk_t;

    /** 
     *  Constructor. 
     *  Note that the constructor does not attempt to open the serial port. That must be done explicitly
     *  by calling open().
     *  @param filename         Serial port filename + (optional) channel definition in Windows API format,
                                as seen in the MSDN documention about the BuildCommDCB() function (excerpt):
                                COMx[:][baud=b][parity=p][data=d][stop=s][to={on|off}][xon={on|off}][odsr={on|off}]
                                    [octs={on|off}][dtr={on|off|hs}][rts={on|off|hs|tg}][idsr={on|off}]
        @param input_dgram_size Specifies the number of characters that the Channel will attempt to read at a time.
                                Specifying a small enough number is important to reduce latency (between physical 
                                arrival of a character and the moment user code can access it). However, smaller
                                numbers also increase CPU usage, context switching and/or inter-thread 
                                synchronization.
     */
    SerialChannel (const std::string &filename, uint input_chunk_size = 0);

    ~SerialChannel();

    /**
     *  Opens the channel for both directions. The class will start reading and buffering
     *  incoming characters immediately (in the background) - be aware that this means you
     *  should start consuming incoming characters immediately, or else risk missing input 
     *  at some point (the input buffer does not grow).
     */
    void
    open ();

    /**
     *  Closes the channel. 
     *  Note that buffered but unconsumed input characters can still be retrieved after the 
     *  channel has been closed.
     */
    void 
    close ();
                    
    /**
     *  Queues the specified characters for sending.
     *  send() returns immediately; an exception is thrown if the internal output buffer 
     *  has insufficient space to queue all of the new characters.
     */
    void 
    send (const byte_t *buffer, size_t size);

    /**
     *  Retrieves characters waiting in the input buffer.
     *  @param max_chars    (optional) The maximum number of characters to retrieve in this call.
     *  @returns            a vector of unsigned char (binary, not null-terminated)
     */
    const chunk_t 
    retrieve (size_t max_chars = 0);

private:
    std::string                 filename;
    void                       *_intern;
};

#endif // __SERIAL_CHANNEL_HPP