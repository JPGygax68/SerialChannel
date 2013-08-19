#ifndef __WINAPI_ERROR_HPP
#define __WINAPI_ERROR_HPP

#include <Windows.h>
#include <string>

class winapi_error : public std::exception {
   public:
      winapi_error(const char * header, UINT error = 0) : m_msg(header) {
         char buffer[1024];
         if (error == 0) error = GetLastError();
         FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, NULL, error, 
            0,	//MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            buffer, 1024, NULL);
         m_msg = m_msg + ": " + buffer;
      }
      const char * what() const { return m_msg.c_str(); }
   private:
		 std::string m_msg;
};

#endif // __WINAPI_ERROR_HPP
