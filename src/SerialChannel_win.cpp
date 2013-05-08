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

static const size_t BUFFER_SIZE = 256;

class InternalStruct {
public:
    typedef SerialChannel::byte_t   byte_t;

    typedef circular_buffer<byte_t> output_buffer_t;
    typedef circular_buffer<byte_t> input_buffer_t;
    typedef vector<byte_t>          read_buffer_t;
    typedef vector<byte_t>          write_buffer_t;

    InternalStruct(SerialChannel *owner_) 
      : owner(owner_), 
        output_buffer(BUFFER_SIZE),
        input_buffer (BUFFER_SIZE),
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

    SerialChannel  *owner;
    HANDLE          hComm;
    OVERLAPPED      overlapped_in;
    OVERLAPPED      overlapped_out;
    output_buffer_t output_buffer;
    input_buffer_t  input_buffer;
    OutputSlave     output_slave;
    InputSlave      input_slave;
    thread          output_thread;
    thread          input_thread;
    mutex           input_mutex;
    mutex           output_mutex;
    bool            disconnected;
    bool            terminating;
    string          input_error;
    string          output_error;

    friend class SerialChannel;
};

//--- MAIN CLASS IMPLEMENTATION -----------------------------------------------

SerialChannel::SerialChannel(const char *def_)
    :def(def_)
{
    _intern = new InternalStruct(this);
}

void
SerialChannel::open()
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    DCB dcb = { 0 };
    COMMTIMEOUTS commTimeouts = {0};

    intern->hComm = CreateFile( def.c_str(),  
        GENERIC_READ | GENERIC_WRITE, 
        0, 
        0, 
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED |FILE_FLAG_NO_BUFFERING,
        0);
    if (intern->hComm == INVALID_HANDLE_VALUE) throw winapi_error("CreateFile()");

    // Set the com port parameters
    dcb.DCBlength = sizeof(DCB);
    string spec = def.find(':') != string::npos ? def : string("9600,n,8,1");
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

    memset(&intern->overlapped_in , 0, sizeof(OVERLAPPED)); 
    memset(&intern->overlapped_out, 0, sizeof(OVERLAPPED)); 

    /*
    DWORD count;
    if (ReadFile(intern->hComm, &intern->read_buffer[0], intern->read_buffer.size(), &count, &intern->overlapped_in) == 0) 
        throw winapi_error("ReadFile() on COM port");
    */

    intern->output_thread = thread( boost::ref(intern->output_slave) );

}

void
SerialChannel::close()
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    intern->input_thread .join();
    intern->output_thread.join();

    if (CloseHandle(intern->hComm) == 0) throw winapi_error("CloseHandle()");
}

void
SerialChannel::send(const byte_t *buffer, size_t size)
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    mutex::scoped_lock(output_mutex);

    if (intern->output_buffer.size() + size >= intern->output_buffer.capacity())
        throw runtime_error("Output buffer capacity exceeded");
    
    for (unsigned int i = 0; i < size; i ++) intern->output_buffer.push_front(buffer[i]);
}

const vector<BYTE>
SerialChannel::retrieve(size_t max_bytes)
{
    auto *intern = static_cast<InternalStruct*>(_intern);

    mutex::scoped_lock(input_mutex);

    size_t size = (max_bytes > 0) ? min(max_bytes, intern->input_buffer.size()) : intern->input_buffer.size();
    vector<BYTE> buffer(size);

    for (unsigned int i = 0; i < size; i++) {
        buffer[i] = intern->input_buffer.back();
        intern->input_buffer.pop_back();
    }
    
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
	while (! owner->terminating)
	{
        try {
            // Check for pending outgoing bytes
            vector<byte_t> write_buffer;
            {
                // Lock the output buffer to check for pending output bytes
                lock_guard<mutex> guard(owner->output_mutex);
                if (owner->output_buffer.size() > 0) {
                    // If output is pending, transfer to the write buffer
                    write_buffer.reserve(owner->output_buffer.size());
                    while (!owner->output_buffer.empty()) {
                        write_buffer.push_back( owner->output_buffer.back() );
                        owner->output_buffer.pop_back();
                    }
                }
            }

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
    }
}

void
InternalStruct::InputSlave::operator() ()
{
    OVERLAPPED ov = {0};
    ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    try {

        while (!owner->terminating) {

            // Start read operation - synchronous or asynchronous
            DWORD bytesRead = 0;
            byte_t buffer[BUFFER_SIZE];
            if (ReadFile(owner->hComm, buffer, BUFFER_SIZE, &bytesRead, &ov) == 0) {
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
                lock_guard<mutex> guard(owner->output_mutex);
                for (unsigned int i = 0; i < bytesRead; i ++) owner->input_buffer.push_front(buffer[i]);
            }

	    }

    }
    catch (const std::exception &e) {
        owner->input_error = e.what();
    }

    CloseHandle(ov.hEvent);
}

