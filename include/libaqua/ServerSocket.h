#pragma once


#include <stdint.h>
#include "SerialDevice.h"
#include "Socket.h"

#ifdef WIN32

#else // WIN32
//#include <sys/socket.h>
//#include <unistd.h>
//#include <stdio.h>
#endif

namespace ssr {

  class TimeoutException : public std::exception {
  public:
    TimeoutException() {}
    virtual ~TimeoutException() throw() {}

  public:
    const char* what() const throw() {
      return "Timeout Exception";

    }
  };

  class ServerSocket {

  private:
#ifdef WIN32
	  SOCKET m_ServerSocket;

	  struct sockaddr_in m_SockAddr;
#else // WIN32
    int m_ServerSocket;

    struct sockaddr_in m_SockAddr;//,caddr;
    //    char buf[BUFSIZ];

#endif

    
  public:
    ServerSocket() {
#ifdef WIN32
		if ((m_ServerSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			throw SocketException("socket failed.");
		}

#else
      if( (m_ServerSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) { 
	throw SocketException("socket failed.");
      }
#endif
    }

    ~ServerSocket() {
    }


  public:

    void Close() {
#ifdef WIN32
      closesocket(m_ServerSocket);
#else

      close(m_ServerSocket);
#endif
    }

    void Bind(const unsigned int port) {
#ifdef WIN32
		m_SockAddr.sin_family = AF_INET;
		m_SockAddr.sin_port = htons(port);
		m_SockAddr.sin_addr.S_un.S_addr = INADDR_ANY;

		if (bind(m_ServerSocket, (struct sockaddr *)&m_SockAddr, sizeof(m_SockAddr)) < 0) {
			throw SocketException("bind failed.");
		}

#else
	  memset((char*)&m_SockAddr, 0, sizeof(m_SockAddr));
      m_SockAddr.sin_family      = AF_INET;
      m_SockAddr.sin_addr.s_addr = INADDR_ANY;
      m_SockAddr.sin_port        = htons(port);

      if(bind(m_ServerSocket, (struct sockaddr*)&m_SockAddr, sizeof(m_SockAddr))<0) {
	throw SocketException("Bind Failed.");
      }
#endif
    }

    void Listen(const unsigned int backlog = 5) {
#ifdef WIN32
		if (listen(m_ServerSocket, backlog) < 0) {
			throw SocketException("listen failed.");
		}
#else
      if (listen(m_ServerSocket, backlog) < 0) {
	throw SocketException("Listen Failed.");
      }
#endif
    }

    Socket Accept(int timeoutUsec) {
#ifdef WIN32
		return Accept();
#else
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(m_ServerSocket, &fds);
      
      struct timeval timeout;
      timeout.tv_sec = timeoutUsec / 100000;
      timeout.tv_usec = timeoutUsec % 100000;
      int result = select(m_ServerSocket+1, &fds, NULL, NULL, &timeout);
      if (result < 0) {
	throw SocketException("select failed.");
      }
	  
      if (!FD_ISSET(m_ServerSocket, &fds)) {
	throw TimeoutException();
      }

      struct sockaddr_in sockaddr_;
      int client_sock;
      socklen_t len;
      len = sizeof(sockaddr_);
      if ((client_sock = accept(m_ServerSocket, (struct sockaddr*)&sockaddr_, &len)) < 0) {
	throw SocketException("Accept Failed.");
      }

      return Socket(client_sock, sockaddr_);

#endif
    }

    Socket Accept() {
#ifdef WIN32
		struct sockaddr_in sockaddr_ = m_SockAddr;
		SOCKET client_sock;
		int len;
		len = sizeof(sockaddr_);
		if ((client_sock = accept(m_ServerSocket, (struct sockaddr*)&sockaddr_, &len)) < 0) {
			throw SocketException("Accept Failed.");
		}
		return Socket(client_sock, sockaddr_);
#else
      struct sockaddr_in sockaddr_;
      int client_sock;
      socklen_t len;
      len = sizeof(sockaddr_);
      if ((client_sock = accept(m_ServerSocket, (struct sockaddr*)&sockaddr_, &len)) < 0) {
	throw SocketException("Accept Failed.");
      }

      return Socket(client_sock, sockaddr_);
#endif
    }

    
  };

};
