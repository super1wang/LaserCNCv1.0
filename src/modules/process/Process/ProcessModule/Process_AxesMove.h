#pragma once
#include "treeitem.h"
#include "ui_Process_AxesMove.h"

class ProcessAxesMove :
    public TreeItem
{
public:
    explicit                            ProcessAxesMove(TreeItem *parent = 0);
    explicit                            ProcessAxesMove(const QString &text, TreeItem *parent = 0);
    explicit                            ProcessAxesMove(const QVector<QVariant> &data, TreeItem *parent = 0);
    ~ProcessAxesMove(void);

public:
    virtual TreeItem*                   clone() const;
    virtual void                        Edit();
    virtual void                        UpdateInfo();
    virtual void                        SwitchState(ItemState state = ItemState::StateSave);
    virtual void                        SetState(ItemState state)                   { m_state   = state; };
    virtual void                        SetMaps(map<QString, QString> maps)         { m_maps    = maps;  };
    virtual ItemType                    GetType()                                   { return    m_type;  };
    virtual ItemState                   GetState()                                  { return    m_state; };
    virtual map<QString, QString>       GetMaps()                                   { return    m_maps;  };

public:
    void                                SetAxesMoveNote     (QString& QstrValue)    { m_maps["Note"]    = QstrValue; };
    void                                SetAxesMoveMode     (int iValue)            { m_maps["Mode"]    = QString::number(iValue); };
    void                                SetAxesMoveSpeed    (int iValue)            { m_maps["Speed"]   = QString::number(iValue); };
    void                                SetAxesMoveX        (bool bValue)           { m_maps["X"]       = QString::number(bValue); };
    void                                SetAxesMoveA        (bool bValue)           { m_maps["A"]       = QString::number(bValue); };
    void                                SetAxesMoveY        (bool bValue)           { m_maps["Y"]       = QString::number(bValue); };
	void                                SetAxesMoveZ        (bool bValue)           { m_maps["Z"]       = QString::number(bValue); };
	void                                SetAxesMoveZIdle    (bool bValue)           { m_maps["ZIdle"]   = QString::number(bValue); };
    void                                SetAxesMoveX1       (bool bValue)           { m_maps["X1"]      = QString::number(bValue); };
    void                                SetAxesMoveA1       (bool bValue)           { m_maps["A1"]      = QString::number(bValue); };
    void                                SetAxesMoveY1       (bool bValue)           { m_maps["Y1"]      = QString::number(bValue); };
    void                                SetAxesMoveZ1       (bool bValue)           { m_maps["Z1"]      = QString::number(bValue); };
	void                                SetAxesMoveZ1Idle   (bool bValue)           { m_maps["Z1Idle"]  = QString::number(bValue); };
    
    void                                SetAxesMoveXPos     (QString& QstrValue)    { m_maps["XPos"]        = QstrValue; };
    void                                SetAxesMoveAPos     (QString& QstrValue)    { m_maps["APos"]        = QstrValue; };
    void                                SetAxesMoveYPos     (QString& QstrValue)    { m_maps["YPos"]        = QstrValue; };
	void                                SetAxesMoveZPos     (QString& QstrValue)    { m_maps["ZPos"]        = QstrValue; };
	void                                SetAxesMoveZIdlePos (QString& QstrValue)    { m_maps["ZIdlePos"]    = QstrValue; };
    void                                SetAxesMoveX1Pos    (QString& QstrValue)    { m_maps["X1Pos"]       = QstrValue; };
    void                                SetAxesMoveA1Pos    (QString& QstrValue)    { m_maps["A1Pos"]       = QstrValue; };
    void                                SetAxesMoveY1Pos    (QString& QstrValue)    { m_maps["Y1Pos"]       = QstrValue; };
	void                                SetAxesMoveZ1Pos    (QString& QstrValue)    { m_maps["Z1Pos"]       = QstrValue; };
	void                                SetAxesMoveZ1IdlePos(QString& QstrValue)    { m_maps["Z1IdlePos"]   = QstrValue; };

    QString                             GetAxesMoveNote()   	{ return    m_maps["Note"];                     };
    int                                 GetAxesMoveMode()   	{ return    m_maps["Mode"].toInt();             };
    int                                 GetAxesMoveSpeed()  	{ return    m_maps["Speed"].toInt();            };
    bool                                GetAxesMoveX()      	{ return    QVariant(m_maps["X"]).toBool();     };
    bool                                GetAxesMoveA()      	{ return    QVariant(m_maps["A"]).toBool();     };
    bool                                GetAxesMoveY()      	{ return    QVariant(m_maps["Y"]).toBool();     };
	bool                                GetAxesMoveZ()      	{ return    QVariant(m_maps["Z"]).toBool();     };
	bool                                GetAxesMoveZIdle()  	{ return    QVariant(m_maps["ZIdle"]).toBool(); };
    bool                                GetAxesMoveX1()     	{ return    QVariant(m_maps["X1"]).toBool();    };
    bool                                GetAxesMoveA1()     	{ return    QVariant(m_maps["A1"]).toBool();    };
    bool                                GetAxesMoveY1()     	{ return    QVariant(m_maps["Y1"]).toBool();    };
	bool                                GetAxesMoveZ1()     	{ return    QVariant(m_maps["Z1"]).toBool();    };
	bool                                GetAxesMoveZ1Idle() 	{ return    QVariant(m_maps["Z1Idle"]).toBool();};
    QString                             GetAxesMoveXPos()   	{ return    m_maps["XPos"];  	};
    QString                             GetAxesMoveAPos()   	{ return    m_maps["APos"];  	};
    QString                             GetAxesMoveYPos()   	{ return    m_maps["YPos"];  	};
	QString                             GetAxesMoveZPos()   	{ return    m_maps["ZPos"]; 	};
	QString                             GetAxesMoveZIdlePos() 	{ return    m_maps["ZIdlePos"]; };
    QString                             GetAxesMoveX1Pos()  	{ return    m_maps["X1Pos"]; 	};
    QString                             GetAxesMoveA1Pos()  	{ return    m_maps["A1Pos"]; 	};
    QString                             GetAxesMoveY1Pos()  	{ return    m_maps["Y1Pos"]; 	};
	QString                             GetAxesMoveZ1Pos() 		{ return    m_maps["Z1Pos"]; 	};
	QString                             GetAxesMoveZ1IdlePos() 	{ return    m_maps["Z1IdlePos"];};
};

class Dialog_ProcessSetting_AxesMove :
    public QDialog
{
    Q_OBJECT

public:
    explicit                            Dialog_ProcessSetting_AxesMove(QWidget* parent = 0);
    ~Dialog_ProcessSetting_AxesMove();

private:
    Ui::Dialog_ProcessSetting_AxesMove* Dialog_AxesMove;

public slots:
    void                                ViewSetting();
    void                                UpdatePage();
    void                                ButtonOK();
    void                                ButtonCancel();
    void                                reject();

public:
    ProcessAxesMove*                    m_pProcessAxesMove;
};
