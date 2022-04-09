#include "moveable_tcp_socket.h"

#include <argos3/core/utility/string_utilities.h>

#include <arpa/inet.h>
#include <cstring>
#include <netdb.h>

namespace argos {

    /****************************************/
    /****************************************/

    CMTCPSocket::CMTCPSocket(SInt32 n_stream) :
        m_nStream(n_stream) {
    }


    CMTCPSocket::CMTCPSocket(CMTCPSocket&& original) noexcept{
        swap(*this, original);
    }

    /****************************************/
    /****************************************/

    CMTCPSocket::~CMTCPSocket() {
        Disconnect();
    }

    /****************************************/
    /****************************************/

    void CMTCPSocket::Connect(const std::string& str_hostname,
                             SInt32 n_port) {
        /* Used to store the return value of the network function calls */
        int nRetVal;
        /* Get information on the available interfaces */
        ::addrinfo tHints, *ptInterfaceInfo;
        ::memset(&tHints, 0, sizeof(tHints));
        tHints.ai_family = AF_INET;       /* Only IPv4 is accepted */
        tHints.ai_socktype = SOCK_STREAM; /* TCP socket */
        nRetVal = ::getaddrinfo(str_hostname.c_str(),
                                ToString(n_port).c_str(),
                                &tHints,
                                &ptInterfaceInfo);
        if(nRetVal != 0) {
            THROW_ARGOSEXCEPTION("Error getting address information: " << ::gai_strerror(nRetVal));
        }
        /* Bind on the first interface available */
        m_nStream = -1;
        ::addrinfo* ptInterface = nullptr;
        for(ptInterface = ptInterfaceInfo;
            (ptInterface != nullptr) && (m_nStream == -1);
            ptInterface = ptInterface->ai_next) {
            m_nStream = ::socket(ptInterface->ai_family,
                                 ptInterface->ai_socktype,
                                 ptInterface->ai_protocol);
            if(m_nStream > 0) {
                if(::connect(m_nStream,
                             ptInterface->ai_addr,
                             ptInterface->ai_addrlen) == -1) {
                    m_nStream = -1;
                    THROW_ARGOSEXCEPTION("Can't connect to host: " << ::strerror(errno));
                }
            }
        }

        m_strAddress = ::inet_ntoa(reinterpret_cast< ::sockaddr_in* >(&ptInterface)->sin_addr);
        ::freeaddrinfo(ptInterfaceInfo);
    }

    /****************************************/
    /****************************************/
    void CMTCPSocket::Listen(int32_t n_port, int32_t n_queue_length) {
        /* Used to store the return value of the network function calls */
        int nRetVal;
        /* Get information on the available interfaces */
        ::addrinfo tHints, *ptInterfaceInfo;
        ::memset(&tHints, 0, sizeof(tHints));
        tHints.ai_family = AF_INET;       /* Only IPv4 is accepted */
        tHints.ai_socktype = SOCK_STREAM; /* TCP socket */
        tHints.ai_flags = AI_PASSIVE;     /* Necessary for bind() later on */
        nRetVal = ::getaddrinfo(nullptr,
                                std::to_string(n_port).c_str(),
                                &tHints,
                                &ptInterfaceInfo);
        if (nRetVal != 0) {
            THROW_ARGOSEXCEPTION("Error getting local address information: " << ::gai_strerror(nRetVal));
        }
        /* Bind on the first interface available */
        m_nStream = -1;
        ::addrinfo *ptInterface = nullptr;
        for (ptInterface = ptInterfaceInfo;
             (ptInterface != nullptr) && (m_nStream == -1);
             ptInterface = ptInterface->ai_next) {
            m_nStream = ::socket(ptInterface->ai_family,
                                 ptInterface->ai_socktype,
                                 ptInterface->ai_protocol);
            if (m_nStream > 0) {
                int nTrue = 1;
                if ((::setsockopt(m_nStream,
                                  SOL_SOCKET,
                                  SO_REUSEADDR,
                                  &nTrue,
                                  sizeof(nTrue)) != -1)
                    &&
                    (::bind(m_nStream,
                            ptInterface->ai_addr,
                            ptInterface->ai_addrlen) == -1)) {
                    Disconnect();
                }
            }
        }

        m_strAddress = ::inet_ntoa(reinterpret_cast< ::sockaddr_in * >(&ptInterface)->sin_addr);
        ::freeaddrinfo(ptInterfaceInfo);
        if (m_nStream == -1) {
            THROW_ARGOSEXCEPTION("Can't bind socket to any interface");
        }
        if (::listen(m_nStream, n_queue_length) == -1) {
            Disconnect();
            THROW_ARGOSEXCEPTION("Can't listen on the socket" << ::strerror(errno));
        }
    }

   /****************************************/
   /****************************************/

   void CMTCPSocket::Accept(CMTCPSocket& c_socket) {
      /* Accept connections */
      ::sockaddr tAddress;
      ::socklen_t tAddressLen = sizeof(tAddress);
      int nNewStream = ::accept(m_nStream, &tAddress, &tAddressLen);
      if(nNewStream == -1) {
         Disconnect();
         THROW_ARGOSEXCEPTION("Error accepting connection: " << ::strerror(errno));
      }
      c_socket.m_nStream = nNewStream;
      c_socket.m_strAddress = ::inet_ntoa(reinterpret_cast< ::sockaddr_in* >(&tAddress)->sin_addr);
   }

   /****************************************/
   /****************************************/

   void CMTCPSocket::Disconnect() {
      if(m_nStream != -1)
        std::cout << "Disconnect socket: " << m_nStream << std::endl;
      if(m_nStream >-1){
        ::close(m_nStream);
        m_nStream = -1;
        m_strAddress = "";
      }
   }

   /****************************************/
   /****************************************/

   void CMTCPSocket::SendBuffer(const UInt8* pun_buffer,
                               size_t un_size) {
      ssize_t nSent;
      while(un_size > 0) {
         nSent = ::send(m_nStream, pun_buffer, un_size, 0);
         if(nSent < 0) {
            Disconnect();
            THROW_ARGOSEXCEPTION("Error sending data (" << pun_buffer
                                  << "): " << ::strerror(errno));
         }
         un_size -= nSent;
         pun_buffer += nSent;
      }
   }

   /****************************************/
   /****************************************/

   bool CMTCPSocket::ReceiveBuffer(UInt8* pun_buffer,
                                  size_t un_size) {
      ssize_t nReceived;
      while(un_size > 0) {
         nReceived = ::recv(m_nStream, pun_buffer, un_size, 0);
         if(nReceived < 0){
            Disconnect();
             THROW_ARGOSEXCEPTION("Error receiving data: " << ::strerror(errno));
         }
         if(nReceived == 0) return false;
         un_size -= nReceived;
         pun_buffer += nReceived;
      }
      return true;
   }

   /****************************************/
   /****************************************/

   void CMTCPSocket::SendByteArray(const CByteArray& c_byte_array) {
      /* Send the length of the byte array */
      UInt32 unSizeNBO = htonl(c_byte_array.Size());
      SendBuffer(reinterpret_cast<UInt8*>(&unSizeNBO), sizeof(unSizeNBO));
      /* Send the actual data */
      SendBuffer(c_byte_array.ToCArray(), c_byte_array.Size());
   }

   /****************************************/
   /****************************************/

   bool CMTCPSocket::ReceiveByteArray(CByteArray& c_byte_array) {
      /* Receive the length of the byte array */
      UInt32 unSizeNBO;
      if(ReceiveBuffer(reinterpret_cast<UInt8*>(&unSizeNBO), sizeof(unSizeNBO))) {
         /* Receive the actual data */
         c_byte_array.Resize(ntohl(unSizeNBO));
         if(ReceiveBuffer(c_byte_array.ToCArray(), c_byte_array.Size())) {
            return true;
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/

    bool CMTCPSocket::MessageAvalible(){
        fd_set cReadSet, cEmptySet;
        struct timeval timeout;
        UInt8 unRet;

        FD_ZERO(&cReadSet);
        FD_ZERO(&cEmptySet);
        FD_SET(m_nStream,&cReadSet);

        timeout.tv_sec = 0;
        timeout.tv_usec = 10;

        unRet = select(m_nStream+1, &cReadSet, &cEmptySet, &cEmptySet, &timeout);
        return unRet && FD_ISSET(m_nStream, &cReadSet);
    }

   /****************************************/
   /****************************************/

}