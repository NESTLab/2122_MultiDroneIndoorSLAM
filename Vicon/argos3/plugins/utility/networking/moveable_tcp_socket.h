#ifndef MOVEABLE_TCP_SOCKET_H
#define MOVEABLE_TCP_SOCKET_H

namespace argos {
   class CMTCPSocket;
}

#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {

    class CMTCPSocket {

    public:

        explicit CMTCPSocket(SInt32 n_stream = -1);

        CMTCPSocket(CMTCPSocket&& original) noexcept;

        ~CMTCPSocket();

        /**
         * Swaps the contens of the CMTCSockets.
         * @param first  First socket.
         * @param second Second socket.
         */
        friend void swap(CMTCPSocket& first, CMTCPSocket& second){

            std::swap(first.m_nStream, second.m_nStream);
            std::swap(first.m_strAddress, second.m_strAddress);
        }


        CMTCPSocket& operator=(CMTCPSocket&& that) noexcept
        {
            swap(*this, that);
            return *this;
        }

        /**
        * Returns <tt>true</tt> if the socket is connected.
        * @return <tt>true</tt> if the socket is connected.
        */
        inline bool IsConnected() const {
            return m_nStream != -1;
        }

        /**
        * Returns a string containing the IPv4 address in dot notation.
        * @return A string containing the IPv4 address in dot notation.
        */
        inline const std::string& GetAddress() const {
            return m_strAddress;
        }

        /**
        * Connects this socket to the specified hostname and port.
        * Internally, the connection is forced to be only IPv4.
        * @param str_hostname The wanted hostname
        * @param n_port The wanted port
        * @throws CARGoSException in case of error
        * @see Accept
        */
        void Connect(const std::string& str_hostname,
                   SInt32 n_port);

        /**
         * Listens for connections on the specified local port.
         * Internally, the connection is forced to be only IPv4.
         * To actually accept connections, you must call Accept() after calling this function.
         * @param n_port The wanted port
         * @param n_queue_length The maximum length of the queue of pending connections (also called the backlog)
         * @throws CARGoSException in case of error
         *  @see Accept
         */
        void Listen(int32_t n_port, int32_t n_queue_length = 10);

        /**
        * Accept a connection from a client.
        * Internally, the connection is forced to be only IPv4.
        * Before calling this function, you must first call Listen() to setup connection
        * listening.
        * @param c_socket The socket on which the connection has been created
        * @throws CARGoSException in case of error
        * @see Listen
        * @see Connect
        */
        void Accept(CMTCPSocket& c_socket);

        /**
        * Close the socket.
        * @throws CARGoSException in case of error
        */
        void Disconnect();

        /**
        * Sends the passed buffer through the socket.
        * @param pun_buffer The wanted buffer
        * @param un_size The size of the buffer
        * @throws CARGoSException in case of error
        */
        void SendBuffer(const UInt8* pun_buffer,
                        size_t un_size);

        /**
        * Fills the passed buffer with the data received through the socket.
        * @param pun_buffer The buffer to fill
        * @param un_size The size of the buffer
        * @return <tt>true</tt> if the buffer was filled correctly; <tt>false</tt> if the connection was closed by the other peer
        * @throws CARGoSException in case of error
        */
        bool ReceiveBuffer(UInt8* pun_buffer,
                           size_t un_size);

        /**
        * Sends the passed byte array through the socket.
        * Internally, this function first sends the size of the
        * byte array as a long int, and then sends the content of
        * the byte array.
        * It is meant to the be used in conjunction with
        * ReceiveByteArray().
        * @param c_byte_array The byte array
        * @throws CARGoSException in case of error
        * @see CByteArray
        * @see SendBuffer
        * @see ReceiveByteArray
        */
        void SendByteArray(const CByteArray& c_byte_array);

        /**
        * Receives the passed byte array through the socket.
        * Internally, this function first receives the size of the
        * byte array as a long int, and then receives the content of
        * the byte array.
        * It is meant to the be used in conjunction with
        * SendByteArray().
        * @param c_byte_array The byte array
        * @return <tt>true</tt> if the buffer was filled correctly; <tt>false</tt> if the connection was closed by the other peer
        * @throws CARGoSException in case of error
        * @see CByteArray
        * @see ReceiveBuffer
        * @see SendByteArray
        */
        bool ReceiveByteArray(CByteArray& c_byte_array);

        /**
         * Determines whether or not there is a message to be read.
         * Internally, uses select with a 1 milisecon timout.
         * Does not check if the message to be read is a full message.
         * @return <tt>true</tt> if the is a message to be read; <tt>false</tt> otherwise
         */
        bool MessageAvalible();

    private:
        
        /** The socket stream */
        int m_nStream = -1;
        /** Address data */
        std::string m_strAddress;
    };
}

#endif
