#ifndef VICON_H_
#define VICON_H_


namespace VD = vicon_driver;
std::string formatTranslation(const double* translation, bool single_line);
void subjectCallback(const VD::ViconDriver::Subject& subject);
void markerCallback(const VD::ViconDriver::Markers& markers);
std::string formatRotation(const double* rotation, bool single_line);

#endif // VICON_H_