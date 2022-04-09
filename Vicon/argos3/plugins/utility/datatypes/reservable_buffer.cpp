#include "reservable_buffer.h"

/****************************************/
/****************************************/

void CReservableBuffer::ReserveSpace(std::string str_name, UInt16 un_size){
    if(!IsReserved(str_name)){
        TRBufferMDEntry tEntry;
        tEntry.m_unIndex = m_unNextIndex;
        tEntry.m_unSize = un_size;

        m_tMetaData[str_name] = tEntry;
        m_unNextIndex += un_size;
        m_cBufferData.Resize(m_unNextIndex);
    }
}

/****************************************/
/****************************************/

//smart pointers?
CByteArray* CReservableBuffer::Read(std::string str_name){
    if(!IsReserved(str_name)){
        return new CByteArray();
    }

    TRBufferMDEntry tEntry = m_tMetaData[str_name];
    return m_cBufferData(tEntry.Start(), tEntry.End());
}

/****************************************/
/****************************************/

void CReservableBuffer::Write(std::string str_name, const CByteArray& c_data){
    if(IsReserved(str_name)){
        TRBufferMDEntry tEntry = m_tMetaData[str_name];
        if(c_data.Size() <= tEntry.m_unSize){
            for(int i = 0; i < c_data.Size(); i++)
                m_cBufferData[tEntry.m_unIndex+i] = c_data[i];
        }else{
            //invalid size
        }
    }else{
        //invalid name
    }
}