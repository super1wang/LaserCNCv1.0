#pragma once

#include <QMap>
#include <QString>

#include <gp_Pnt.hxx>

enum class MachineRenderQuality {
    High = 0,
    Medium = 1,
    Low = 2,
};

class CamConfig
{
public:
    static CamConfig& instance();

    void load();
    void save();

    QString machineModelPath() const;
    void setMachineModelPath(const QString& path);

    QString machinePreset() const;
    void setMachinePreset(const QString& preset);

    MachineRenderQuality machineRenderQuality() const;
    void setMachineRenderQuality(MachineRenderQuality quality);

    double leadInLength() const;
    void setLeadInLength(double mm);

    double normalAngle() const;
    void setNormalAngle(double deg);

    double deflection() const;
    void setDeflection(double mm);

    double smoothAngle() const;
    void setSmoothAngle(double deg);

    bool useFaceClassification() const;
    void setUseFaceClassification(bool enabled);

    bool showNormals() const;
    void setShowNormals(bool enabled);

    double normalSampleStep() const;
    void setNormalSampleStep(double mm);

    bool axisOriginForMachine(const QString& machinePath,
                              const QString& axisName,
                              gp_Pnt* outOrigin) const;
    void setAxisOriginForMachine(const QString& machinePath,
                                 const QString& axisName,
                                 const gp_Pnt& origin);

    bool cutterHeadModelPositionForMachine(const QString& machinePath,
                                           gp_Pnt* outPosition) const;
    void setCutterHeadModelPositionForMachine(const QString& machinePath,
                                              const gp_Pnt& position);

    bool cutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                              gp_Pnt* outPosition) const;
    void setCutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                                 const gp_Pnt& position);

    bool workpieceInstallPositionForMachine(const QString& machinePath,
                                            gp_Pnt* outPosition) const;
    void setWorkpieceInstallPositionForMachine(const QString& machinePath,
                                               const gp_Pnt& position);

private:
    struct MachineProfile {
        QMap<QString, gp_Pnt> axisOrigins;
        bool hasCutterHeadModel{false};
        gp_Pnt cutterHeadModelPosition;
        bool hasCutterHeadPhysical{false};
        gp_Pnt cutterHeadPhysicalPosition;
        bool hasWorkpieceInstallPosition{false};
        gp_Pnt workpieceInstallPosition;
    };

    CamConfig() = default;

    void ensureLoaded() const;
    QString configDirectoryPath() const;
    QString configFilePath() const;
    QString machineKey(const QString& machinePath) const;

    MachineProfile* mutableProfileForMachine(const QString& machinePath);
    const MachineProfile* profileForMachine(const QString& machinePath) const;

    mutable bool m_loaded{false};

    QString m_machineModelPath;
    QString m_machinePreset;
    MachineRenderQuality m_machineRenderQuality{MachineRenderQuality::Medium};
    double m_leadInLength{5.0};
    double m_normalAngle{0.0};
    double m_deflection{0.1};
    double m_smoothAngle{5.0};
    bool m_useFaceClassification{true};
    bool m_showNormals{false};
    double m_normalSampleStep{2.0};

    QMap<QString, MachineProfile> m_machineProfiles;
};