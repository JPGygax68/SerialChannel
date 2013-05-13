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

#include <Windows.h>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "../SerialChannel.hpp"
#include "winapi_error.hpp"

using namespace std;
using namespace boost;

static const size_t DEFAULT_BUFFER_SIZE      = 2048;
static const size_t DEFAULT_INPUT_CHUNK_SIZE = 16;

class InternalStruct {
public:
    typedef SerialChannel::byte_t   byte_t;

    typedef circular_buffer<byte_t> output_buffer_t;
    typedef circular_buffer<byte_t> input_buffer_t;
    typedef vector<byte_t>          read_buffer_t;
    typedef vector<byte_t>          write_buffer_t;

    InternalStruct(SerialChannel *owner_) 
      : owner(owner_), 
        output_buffer(DEFAULT_BUFFER_SIZE),
        input_buffer (DEFAULT_BUFFER_SIZE),
        disconnected (false),
        terminating  (false)
    { 
        input_slave.owner  = this;
        output_slave.owner = this;
    }

private:

    struct OutputSlave {
        void operator() ();
        InternalStruct *owner;
    };
    struct InputSlave {
        void operator() ();
        InternalStruct *owner;
    };

    SerialChannel        *owner;
    HANDLE                hComm;
    output_buffer_t       output_buffer;
    input_buffer_t        input_buffer;
    read_buffer_t         read_buffer;
    OutputSlave           output_slave;
    InputSlave            input_slave;
    thread                output_thread;
    thread                input_thread;
    mutex                 input_mutex;
    mutex                 output_mutex;
    condition_variable    input_thread_cond;
    condition_variable    output_thread_cond;
    bool                  disconnected;
    bool                  terminating;
    string                input_error;
    string                output_error;

    friend class SerialChannel;
};

//--- MAIN CLASS IMPLEMENTATION -----------------------------------------------

SerialChannel::SerialChannel(const string &filename_, uint input_chunk_size_)
    :filename(filename_)
{
    auto intern = new InternalStruct(this);
    _intern = intern;
    intern->read_buffer.resize(input_chunk_size_ ? input_chunk_size_ : DEFAULT_INPUT_CHUNK_SIZE);
}

void
SerialChannel::open()
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    DCB dcb = { 0 };
    COMMTIMEOUTS commTimeouts = {0};

    intern->hComm = CreateFile( filename.c_str(),  
        GENERIC_READ | GENERIC_WRITE, 
        0, 
        0, 
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED |FILE_FLAG_NO_BUFFERING,
        0);
    if (intern->hComm == INVALID_HANDLE_VALUE) throw winapi_error("CreateFile()");

    // Set the com port parameters
    dcb.DCBlength = sizeof(DCB);
    string spec = filename.find(':') != string::npos ? filename : string("9600,n,8,1");
    if(!BuildCommDCB(spec.c_str(), &dcb)) throw winapi_error("BuildCommDCB()");
    dcb.fBinary = true;
	/*
    dcb.BaudRate     	= baud;
	dcb.fParity      	= (parity != 0) ? TRUE : FALSE;
	dcb.fBinary      	= TRUE;
	dcb.Parity       	= parity;
	dcb.StopBits     	= stop;
	dcb.fOutxCtsFlow 	= FALSE;
	dcb.fOutxDsrFlow 	= FALSE;
	dcb.fDtrControl	= DTR_CONTROL_ENABLE; //DTR_CONTROL_DISABLE ? (original DTR_CONTROL_ENABLE)
	dcb.fRtsControl	= RTS_CONTROL_ENABLE; //RTS_CONTROL_DISABLE ? (original RTS_CONTROL_ENABLE)
	dcb.fDsrSensitivity= FALSE;
	dcb.fAbortOnError	= FALSE;
	dcb.ByteSize     	= data;
    */
    if (!SetCommState(intern->hComm, &dcb)) throw winapi_error("SetCommState() on COM port");

    // Set the com port read/write timeouts
    DWORD serialBitsPerByte = dcb.ByteSize + 1;
    serialBitsPerByte += (dcb.Parity == NOPARITY  ) ? 0 : 1;
    serialBitsPerByte += (dcb.StopBits == ONESTOPBIT) ? 1 : 2;
    DWORD msPerByte = (dcb.BaudRate > 0) ? ((1000 * serialBitsPerByte+ dcb.BaudRate - 1) / dcb.BaudRate) : 1;
    commTimeouts.ReadIntervalTimeout         = msPerByte;   // Minimize chance of concatenating of separate serial port packets on read
    commTimeouts.ReadTotalTimeoutMultiplier  = 0;           // Do not allow big read timeout when big read buffer used
    commTimeouts.ReadTotalTimeoutConstant    = 1000;        // Total read timeout (period of read loop)
    commTimeouts.WriteTotalTimeoutConstant   = 1000;        // Const part of write timeout
    commTimeouts.WriteTotalTimeoutMultiplier = msPerByte;   // Variable part of write timeout (per byte)
    if(!SetCommTimeouts(intern->hComm, &commTimeouts)) throw winapi_error("SetCommTimeouts()");

    // Remove garbage data in RX/TX queues
    PurgeComm(intern->hComm, PURGE_RXCLEAR); 
    PurgeComm(intern->hComm, PURGE_TXCLEAR);

    // Start the input and output worker threads
    intern->output_thread = thread( boost::ref(intern->output_slave) );
    intern->input_thread  = thread( boost::ref(intern->input_slave ) );
}

void
SerialChannel::close()
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    intern->terminating = true;
    intern->input_thread_cond .notify_all();
    intern->output_thread_cond.notify_all();
    intern->input_thread .join();
    intern->output_thread.join();

    if (CloseHandle(intern->hComm) == 0) throw winapi_error("CloseHandle()");
}

void
SerialChannel::send(const byte_t *buffer, size_t size)
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    if (intern->output_error.size() > 0) throw runtime_error(string("SerialChannel sending error: ")+intern->input_error);

    mutex::scoped_lock(output_mutex);

    if (intern->output_buffer.size() + size >= intern->output_buffer.capacity())
        throw runtime_error("Output buffer capacity exceeded");
    
    for (unsigned int i = 0; i < size; i ++) intern->output_buffer.push_front(buffer[i]);

    intern->output_thread_cond.notify_all();
}

const vector<BYTE>
SerialChannel::retrieve(size_t max_bytes)
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    if (intern->input_error.size() > 0) throw runtime_error(string("SerialChannel receiving error: ")+intern->input_error);

    mutex::scoped_lock(input_mutex);

    size_t size = (max_bytes > 0) ? min(max_bytes, intern->input_buffer.size()) : intern->input_buffer.size();
    vector<BYTE> buffer(size);

    for (unsigned int i = 0; i < size; i++) {
        buffer[i] = intern->input_buffer.back();
        intern->input_buffer.pop_back();
    }

    // Inform reader thread that we just made room in the buffer
    intern->input_thread_cond.notify_all();
    
    return buffer;
}

/*
void
SerialChannel::sendNextQueuedOutputBuffer()
{
    buffer_t &buffer = output_queue.back();

    if (WriteFileEx(hComm, &buffer[0], buffer.size(), &overlapped_out, &writeComplete) == 0) 
        throw winapi_error("WriteFileEx() on COM port");
}
*/

//--- InternalStruct methods --------------------------------------------------

void
InternalStruct::OutputSlave::operator() ()
{
    //void log(int, char *,...); // TODO: remove once no longer needed

	// Send all datagrams currently in the queue
    while (true) {

        try {

            vector<byte_t> write_buffer;

            // Wait for outgoing characters to become available
            unique_lock<mutex> lock(owner->output_mutex);
            while (true) {
                if (owner->terminating) break;
                if (owner->output_buffer.size() > 0) break;
                owner->output_thread_cond.wait(lock);
            }

            // We're done here
            if (owner->terminating) break;

            // While we still hold the lock, transfer from output to write buffer
            if (owner->output_buffer.size() > 0) {
                write_buffer.reserve(owner->output_buffer.size());
                while (!owner->output_buffer.empty()) {
                    write_buffer.push_back( owner->output_buffer.back() );
                    owner->output_buffer.pop_back();
                }
            }

            // We don't need exclusive access to write
            lock.unlock(); 

            // Got stuff to send ?
            if (write_buffer.size() > 0) {
                OVERLAPPED ov = {0};
                ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
                // Start write operation - synchrounous or asynchronous
                DWORD bytesWrittenSync = 0;
                if (WriteFile(owner->hComm, &write_buffer[0], write_buffer.size(), &bytesWrittenSync, &ov) == 0) {
                    // Operation did not complete synchronously
                    DWORD last_error = GetLastError();
                    if (last_error != ERROR_IO_PENDING) throw winapi_error("Writing to COM port (WriteFile)", last_error);
                    DWORD bytesWrittenAsync = 0;
                    if (!GetOverlappedResult(owner->hComm, &ov, &bytesWrittenAsync, TRUE)) throw winapi_error("Writing to COM port (GetOverlappedResult)");
                }
                CloseHandle(ov.hEvent);
            }
        }
        catch(const std::exception &e) {
            owner->output_error = e.what();
            break;
        }

    } // working loop
}

void
InternalStruct::InputSlave::operator() ()
{
    OVERLAPPED ov = {0};
    ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    while (true) {

        try {

            // Wait until there is room in the input buffer (using a condition variable)
            unique_lock<mutex> lock(owner->input_mutex);
            while (true) {
                if (owner->terminating) break;
                if (owner->input_buffer.size() + owner->read_buffer.size() <= owner->input_buffer.capacity()) break;
                owner->input_thread_cond.wait(lock);
            }

            // If we're closing down, exit the loop immediately
            if (owner->terminating) break;

            // We don't need exclusive access while reading
            lock.unlock(); 

            // Start read operation - synchronous or asynchronous
            DWORD bytesRead = 0;
            if (ReadFile(owner->hComm, &owner->read_buffer[0], owner->read_buffer.size(), &bytesRead, &ov) == 0) {
                DWORD last_error = GetLastError();
                if (last_error != ERROR_IO_PENDING) {
                    // Read operation error
                    if (last_error == ERROR_OPERATION_ABORTED) {
                        owner->disconnected = true;
                    }
                    else throw winapi_error("Reading from COM port (ReadFile)", last_error);
                }

                // Wait for async read operation completion or timeout
                bytesRead = 0;
                if (!GetOverlappedResult(owner->hComm, &ov, &bytesRead, TRUE)) {
                    // Read operation error
                    last_error = GetLastError();
                    if (last_error == ERROR_OPERATION_ABORTED) {
                        owner->disconnected = true;
                    }
                    else throw winapi_error("Reading from COM port (GetOverlappedResult)", last_error);
                }
            }
            else {
                // Read operation completed synchronously
            }

            // Return data received if any
            if (bytesRead > 0) {
                if (owner->input_buffer.size() + bytesRead >= owner->input_buffer.capacity()) throw runtime_error("Input buffer capacity exceeded");
                // Lock the output buffer to transmit the received bytes
                lock_guard<mutex> guard(owner->input_mutex);
                for (unsigned int i = 0; i < bytesRead; i ++) owner->input_buffer.push_front(owner->read_buffer[i]);
            }

	    }
        catch (const std::exception &e) {
            owner->input_error = e.what();
        }

    } // working loop

    CloseHandle(ov.hEvent);
}

