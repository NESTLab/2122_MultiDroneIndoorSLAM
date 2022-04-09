#ifndef SPAM_UPDATER_H
#define SPAM_UPDATER_H

#include <vector>
#include <argos3/plugins/simulator/physics_engines/tracking/tracking_updaters/tracking_updater.h>

using namespace argos;

class CCSVUpdater : public CTrackingUpdater{
public:
    typedef std::vector<std::string> TRow;
public: 
    void Init(TConfigurationNode& t_tree) override;

    TRobotInfoList Update() override;

    void Reset() override {}

    void Destroy() override;

    void GetRobotTypeFromName(const std::string&  str_original_name,
                              std::string& str_type,
                              std::string& str_name,
                              std::string& str_options)const;

    bool GetDataFromEntry(const std::string& entry, CVector3& c_position, CQuaternion& c_orientation);


    CCSVUpdater::TRow ParseCSV(std::istream& str);
private:

    TRow m_tRobots;
    std::vector<TRow> m_tEntries;
    UInt16 m_unRequest;

};

#endif