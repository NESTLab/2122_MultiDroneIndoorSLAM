#ifndef RESERVABLE_BUFFER_H
#define RESERVABLE_BUFFER_H

#include <map>
#include <argos3/core/utility/datatypes/byte_array.h>

using namespace argos;

class CReservableBuffer{

public:
    //what to when already reserved, error?
    void ReserveSpace(std::string str_name, UInt16 un_size);

    inline bool IsReserved(const std::string& str_name){
        return m_tMetaData.find(str_name) != m_tMetaData.end();
    }

    //returns an empty byte array when str_name is not in buffer
    CByteArray* Read(std::string str_name);

    void Write(std::string str_name, const CByteArray&);

private:
    typedef struct SRBufferMDEntry{
        UInt16 m_unIndex = 0;
        UInt16 m_unSize = 0;

        inline UInt16 Start(){
            return m_unIndex;
        }

        inline UInt16 End(){
            return m_unIndex + m_unSize;
        }
    } TRBufferMDEntry;

    typedef std::map<std::string, TRBufferMDEntry> TMetaData;

    UInt16 m_unNextIndex = 0;
    TMetaData m_tMetaData;
    CByteArray m_cBufferData;
};

#endif