#pragma once
// ── Regex patterns for line-edit validators ──
// Separated from data_type.h to keep that header MOC-safe
// (QRegularExpression/QValidator pull in heavy Qt internals).
// Include this only from .cpp files.

#include <QRegularExpression>
#include <QValidator>

inline const QRegularExpression& Regex_All_Int()          { static QRegularExpression re("^-?\\d{1,15}$"); return re; }
inline const QRegularExpression& Regex_Nonnegative_Int()  { static QRegularExpression re("^\\d{1,15}$"); return re; }
inline const QRegularExpression& Regex_Positive_Int()     { static QRegularExpression re("^[1-9]\\d{0,7}$"); return re; }
inline const QRegularExpression& Regex_All_Double()       { static QRegularExpression re("^-?(?=.{1,15}$)(?=\\d|\\.\\d)\\d*(\\.\\d+)?$"); return re; }
inline const QRegularExpression& Regex_Nonnegative_Double(){ static QRegularExpression re("^(?=.{1,15}$)(?=\\d|\\.\\d)\\d*(\\.\\d+)?$"); return re; }
inline const QRegularExpression& Regex_Pos_Double()       { static QRegularExpression re("^-?\\d{1,5}(\\.\\d{1,3})?$"); return re; }
inline const QRegularExpression& Regex_Digital_Index()    { static QRegularExpression re("^-?N?\\d\\.\\d(\\d)?$"); return re; }
inline const QRegularExpression& Regex_Digital_Out()      { static QRegularExpression re("^[01]$"); return re; }
inline const QRegularExpression& Regex_Digital_IndexOut() { static QRegularExpression re("^-?N?\\d\\.\\d(\\d)?=[01]$"); return re; }
inline const QRegularExpression& Regex_Analog_Index()     { static QRegularExpression re("^N?\\d(\\d)?$"); return re; }
inline const QRegularExpression& Regex_Analog_Out()       { static QRegularExpression re("^(?=.{1,15}$)(?=\\d|\\.\\d)\\d*(\\.\\d+)?$"); return re; }
inline const QRegularExpression& Regex_Analog_IndexOut()  { static QRegularExpression re("^N?\\d(\\d)?=(?=.{1,15}$)(?=\\d|\\.\\d)\\d*(\\.\\d+)?$"); return re; }
inline const QRegularExpression& Regex_Internet_IP()      { static QRegularExpression re("^N?\\d\\.\\d(\\d)?$"); return re; }
inline const QRegularExpression& Regex_Internet_Port()    { static QRegularExpression re("^N?\\d(\\d)?$"); return re; }
inline const QRegularExpression& Regex_Normal_String()    { static QRegularExpression re("^[a-zA-Z0-9_\\-\\.]?$"); return re; }
