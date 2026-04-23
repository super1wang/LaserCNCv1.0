#pragma once

#include "core/command/commands_api.h"
#include <QString>

// ─────────────────────────────────────────────────────────────────────────────
// File commands
// ─────────────────────────────────────────────────────────────────────────────

class CmdNewDocument : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.new";
    explicit CmdNewDocument(IAppContext* ctx);
    void execute() override;
};

class CmdOpenDocument : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.open";
    explicit CmdOpenDocument(IAppContext* ctx);
    void execute() override;
};

class CmdSaveDocument : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.save";
    explicit CmdSaveDocument(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSaveDocumentAs : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.saveAs";
    explicit CmdSaveDocumentAs(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdImportStep : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.importStep";
    explicit CmdImportStep(IAppContext* ctx);
    void execute() override;
};

class CmdImportStl : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.importStl";
    explicit CmdImportStl(IAppContext* ctx);
    void execute() override;
};

class CmdExportStep : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.exportStep";
    explicit CmdExportStep(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdCloseDocument : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "file.close";
    explicit CmdCloseDocument(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};
