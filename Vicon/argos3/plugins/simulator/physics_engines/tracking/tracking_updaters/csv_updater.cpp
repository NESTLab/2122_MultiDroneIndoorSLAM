#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/csv_updater.h>
#include <argos3/core/utility/logging/argos_log.h>

void CCSVUpdater::Init(TConfigurationNode& t_tree){
    std::string strFileName;
    GetNodeAttribute(t_tree, "file", strFileName);
    m_unRequest = 0;
    
    std::ifstream cFile(strFileName);

    m_tRobots = ParseCSV(cFile);
    int i = 2;
    while(cFile){
        TRow tRow = ParseCSV(cFile);
        if(tRow.size() != m_tRobots.size()){
            if(tRow.empty())
                THROW_ARGOSEXCEPTION("CSV file \"" << strFileName << " is improperly formated, "
                                     << "all lines need to have the same length.\n"
                                     << "Line " << i-1 << " has " << tRow.size()
                                     << " entries while the rest have " << m_tRobots.size()
                                     << " entries.");
        }
        else
            m_tEntries.push_back(tRow);
        i++;
    }
}

/****************************************/
/****************************************/

TRobotInfoList CCSVUpdater::Update(){
    TRobotInfoList tRobotInfoList;
    if(m_unRequest < m_tEntries.size()){
        for(UInt16 i = 0; i < m_tRobots.size(); i++){
            std::string strType, strName, strOptions;
            CVector3 cTrans;
            CQuaternion cQuat;

            GetRobotTypeFromName(m_tRobots[i], strType, strName, strOptions);
            GetDataFromEntry(m_tEntries[m_unRequest][i], cTrans, cQuat);

            // LOG << cQuat << "->(" << cQuat.GetW() << "," << cQuat.GetX()
            //     << "," << cQuat.GetY() << "," << cQuat.GetZ() << ")" << std::endl;

            TRobotInfo tRobotInfo(strType, strName, cTrans, cQuat, strOptions);
            tRobotInfoList.push_back(tRobotInfo);
        }
        m_unRequest += 1;
    }

    return tRobotInfoList;
}

/****************************************/
/****************************************/

void CCSVUpdater::GetRobotTypeFromName(const std::string&  str_original_name,
                                           std::string& str_type,
                                           std::string& str_name,
                                           std::string& str_options)const{
    if(str_original_name.empty())
        return;
    std::stringstream cStringStream, cNameStringStream;

    cStringStream.str(str_original_name);
    std::getline(cStringStream, str_name, '\'');
    std::getline(cStringStream, str_options);
    if(str_name.empty())
        return;
    cNameStringStream.str(str_name);
    std::getline(cNameStringStream, str_type, '_');
    
}

/****************************************/
/****************************************/

bool CCSVUpdater::GetDataFromEntry(const std::string& entry, 
                                   CVector3& c_position, 
                                   CQuaternion& c_orientation){

    std::stringstream lineStream(entry);
    std::string       cell;

    std::getline(lineStream,cell, ' ');
    c_position.SetX(std::stof(cell));
    std::getline(lineStream,cell, ' ');
    c_position.SetY(std::stof(cell));
    std::getline(lineStream,cell, ' ');
    c_position.SetZ(std::stof(cell));

    std::getline(lineStream,cell, ' ');
    c_orientation.SetW(std::stof(cell));
    std::getline(lineStream,cell, ' ');
    c_orientation.SetX(std::stof(cell));
    std::getline(lineStream,cell, ' ');
    c_orientation.SetY(std::stof(cell));
    std::getline(lineStream,cell, ' ');
    c_orientation.SetZ(std::stof(cell));

    return true;
}

/****************************************/
/****************************************/

CCSVUpdater::TRow CCSVUpdater::ParseCSV(std::istream& str){
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell, ','))
    {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
        // If there was a trailing comma then add an empty element.
        result.emplace_back("");
    }
    return result;
}

/****************************************/
/****************************************/

void CCSVUpdater::Destroy(){

} 

REGISTER_TRACKING_UPDATER(CCSVUpdater, "csv_updater");
