#include "Process_AxesMove.h"
#include <qDebug>


ProcessAxesMove::ProcessAxesMove(TreeItem *parent)
{
    m_parentItem    = parent;
    m_type          = ItemType::AxesMove;
    m_state         = ItemState::Unavailable;
    m_stateSave     = m_state;
    m_maps          = { {"Note",""}, {"Mode","2"}, {"Speed","1"}, {"X","0"}, {"A","0"}, {"Y","0"}, {"Z","0"}, {"ZIdle","0"}, {"X1","0"}, {"A1","0"}, {"Y1","0"}, {"Z1","0"}, {"Z1Idle","0"},
                        {"XPos","0"}, {"APos","0"}, {"YPos","0"}, {"ZPos","0"}, {"ZIdlePos","0"}, {"X1Pos","0"}, {"A1Pos","0"}, {"Y1Pos","0"}, { "Z1Pos","0" }, {"ZIdlePos","0"}, };
    m_itemData.resize(2);
}

ProcessAxesMove::ProcessAxesMove(const QString& text, TreeItem* parent)
{
    m_parentItem    = parent;
    m_type          = ItemType::AxesMove;
    m_state         = ItemState::Unavailable;
    m_stateSave     = m_state;
	m_maps          = { {"Note",""}, {"Mode","2"}, {"Speed","1"}, {"X","0"}, {"A","0"}, {"Y","0"}, {"Z","0"}, {"ZIdle","0"}, {"X1","0"}, {"A1","0"}, {"Y1","0"}, {"Z1","0"}, {"Z1Idle","0"},
	                    {"XPos","0"}, {"APos","0"}, {"YPos","0"}, {"ZPos","0"}, {"ZIdlePos","0"}, {"X1Pos","0"}, {"A1Pos","0"}, {"Y1Pos","0"}, { "Z1Pos","0" }, {"ZIdlePos","0"}, };
    m_itemData.resize(2);
    UpdateInfo();
}

ProcessAxesMove::ProcessAxesMove(const QVector<QVariant>& data, TreeItem* parent)
{
    m_parentItem    = parent;
    m_type          = ItemType::AxesMove;
    m_state         = ItemState::Unavailable;
    m_stateSave     = m_state;
	m_maps          = { {"Note",""}, {"Mode","2"}, {"Speed","1"}, {"X","0"}, {"A","0"}, {"Y","0"}, {"Z","0"}, {"ZIdle","0"}, {"X1","0"}, {"A1","0"}, {"Y1","0"}, {"Z1","0"}, {"Z1Idle","0"},
	                    {"XPos","0"}, {"APos","0"}, {"YPos","0"}, {"ZPos","0"}, {"ZIdlePos","0"}, {"X1Pos","0"}, {"A1Pos","0"}, {"Y1Pos","0"}, { "Z1Pos","0" }, {"ZIdlePos","0"}, };
    m_itemData.resize(2);
    m_itemData      = data;
}

ProcessAxesMove::~ProcessAxesMove(void)
{
    qDeleteAll(m_childItems);
    m_childItems.clear();
    qDeleteAll(m_childItems);
}

TreeItem* ProcessAxesMove::clone() const
{
    TreeItem* newItem = nullptr;

    if (!m_itemData.isEmpty()) {
        newItem = new ProcessAxesMove(QStringLiteral("AxesMove"));
    }
    else {
        newItem = new TreeItem(static_cast<TreeItem*>(nullptr));
    }

    newItem->m_type = m_type;
    newItem->m_state = m_state;
    newItem->m_stateSave = m_stateSave;
    newItem->m_maps = m_maps;

    for (TreeItem* child : m_childItems)
    {
        TreeItem* clonedChild = child->clone();
        newItem->appendChild(clonedChild);
    }

    return newItem;
}

void ProcessAxesMove::Edit()
{
    if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
    {
        Dialog_ProcessSetting_AxesMove* Dialog = new Dialog_ProcessSetting_AxesMove();
        Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
        Dialog->m_pProcessAxesMove = this;
        Dialog->ViewSetting();
        Dialog->show();
    }
}

void ProcessAxesMove::UpdateInfo()
{
    if (TreeItem::IsChinese())
        setData(0, "\xE5\xA4\x9A\xE8\xBD\xB4\xE8\xBF\x90\xE5\x8A\xA8"); // 多轴运动
    else
        setData(0, "AxesMove");
    
    setData(1, m_maps["Note"]);
}

void ProcessAxesMove::SwitchState(ItemState state)
{
    if (state == ItemState::StateSave)
    {
		if (m_state == ItemState::Unrun || m_state == ItemState::Run ||
			m_state == ItemState::Pause || m_state == ItemState::Stop)
			SetState(ItemState::Enable);
		else if (m_state == ItemState::Unuse)
			SetState(ItemState::Disable);
		else
			SetState(m_stateSave);
    }
    else
    {
        m_stateSave = m_state;
        SetState(state);
    }
}


Dialog_ProcessSetting_AxesMove::Dialog_ProcessSetting_AxesMove(QWidget* parent) :
    QDialog(parent),
    m_pProcessAxesMove(nullptr),
    Dialog_AxesMove(new Ui::Dialog_ProcessSetting_AxesMove)
{
    Dialog_AxesMove->setupUi(this);
    setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

    connect(Dialog_AxesMove->Button_AxesMove_OK,     SIGNAL(clicked()),                 this, SLOT(ButtonOK()));
    connect(Dialog_AxesMove->Button_AxesMove_Cancel, SIGNAL(clicked()),                 this, SLOT(ButtonCancel()));
    connect(Dialog_AxesMove->comboBox_AxesMove_Mode, SIGNAL(currentIndexChanged(int)),  this, SLOT(UpdatePage()));
}

Dialog_ProcessSetting_AxesMove::~Dialog_ProcessSetting_AxesMove()
{
    delete Dialog_AxesMove;
}

void Dialog_ProcessSetting_AxesMove::ViewSetting()
{
    //修改标签状态，避免重复打开标签设置
    m_pProcessAxesMove->SwitchState(ItemState::Editing);

	//限制输入值
	QList<QLineEdit*> lineEdits = this->findChildren<QLineEdit*>();
    for (QLineEdit* lineEdit : lineEdits)
    {
        lineEdit->setValidator(new QRegExpValidator(Regex_Pos_Double));
    }

	if (!DT::IsAxisUse(Axis::X)) {
		Dialog_AxesMove->checkBox_AxesMove_X    ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_XPos ->setHidden(true);  }

	if (!DT::IsAxisUse(Axis::X1))   {
		Dialog_AxesMove->checkBox_AxesMove_X1   ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_X1Pos->setHidden(true);  }

    if (!DT::IsAxisUse(Axis::A))    {
        Dialog_AxesMove->checkBox_AxesMove_A    ->setHidden(true);
        Dialog_AxesMove->lineEdit_AxesMove_APos ->setHidden(true);  }

	if (!DT::IsAxisUse(Axis::A1))   {
		Dialog_AxesMove->checkBox_AxesMove_A1   ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_A1Pos->setHidden(true);  }

	if (!DT::IsAxisUse(Axis::Y))    {
		Dialog_AxesMove->checkBox_AxesMove_Y    ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_YPos ->setHidden(true);  }

	if (!DT::IsAxisUse(Axis::Y1))   {
		Dialog_AxesMove->checkBox_AxesMove_Y1   ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_Y1Pos->setHidden(true);  }

	if (!DT::IsAxisUse(Axis::Z))    {
		Dialog_AxesMove->checkBox_AxesMove_Z        ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_ZPos     ->setHidden(true);
		Dialog_AxesMove->checkBox_AxesMove_ZIdle    ->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_ZIdlePos ->setHidden(true);   }
	
    if (!DT::IsAxisUse(Axis::Z1))   {
		Dialog_AxesMove->checkBox_AxesMove_Z1->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_Z1Pos->setHidden(true);
		Dialog_AxesMove->checkBox_AxesMove_Z1Idle->setHidden(true);
		Dialog_AxesMove->lineEdit_AxesMove_Z1IdlePos->setHidden(true);   }

    //读取参数
    QString QstrNote    	= m_pProcessAxesMove->GetAxesMoveNote();
    int     iMode       	= m_pProcessAxesMove->GetAxesMoveMode();
    int     iSpeed      	= m_pProcessAxesMove->GetAxesMoveSpeed();
    bool    bX          	= m_pProcessAxesMove->GetAxesMoveX();
    bool    bA          	= m_pProcessAxesMove->GetAxesMoveA();
    bool    bY          	= m_pProcessAxesMove->GetAxesMoveY();
	bool    bZ          	= m_pProcessAxesMove->GetAxesMoveZ();
	bool    bZIdle      	= m_pProcessAxesMove->GetAxesMoveZIdle();
    bool    bX1         	= m_pProcessAxesMove->GetAxesMoveX1();
    bool    bA1         	= m_pProcessAxesMove->GetAxesMoveA1();
    bool    bY1         	= m_pProcessAxesMove->GetAxesMoveY1();
	bool    bZ1         	= m_pProcessAxesMove->GetAxesMoveZ1();
	bool    bZ1Idle     	= m_pProcessAxesMove->GetAxesMoveZ1Idle();
    QString QstrXPos    	= m_pProcessAxesMove->GetAxesMoveXPos();
    QString QstrAPos    	= m_pProcessAxesMove->GetAxesMoveAPos();
    QString QstrYPos    	= m_pProcessAxesMove->GetAxesMoveYPos();
	QString QstrZPos    	= m_pProcessAxesMove->GetAxesMoveZPos();
	QString QstrZIdlePos 	= m_pProcessAxesMove->GetAxesMoveZIdlePos();
    QString QstrX1Pos   	= m_pProcessAxesMove->GetAxesMoveX1Pos();
    QString QstrA1Pos   	= m_pProcessAxesMove->GetAxesMoveA1Pos();
    QString QstrY1Pos   	= m_pProcessAxesMove->GetAxesMoveY1Pos();
	QString QstrZ1Pos   	= m_pProcessAxesMove->GetAxesMoveZ1Pos();
	QString QstrZ1IdlePos 	= m_pProcessAxesMove->GetAxesMoveZ1IdlePos();

    //显示参数
    Dialog_AxesMove->lineEdit_AxesMove_Note 	->setText(QstrNote);
    Dialog_AxesMove->comboBox_AxesMove_Mode 	->setCurrentIndex(iMode);
    Dialog_AxesMove->comboBox_AxesMove_Speed	->setCurrentIndex(iSpeed);
    Dialog_AxesMove->checkBox_AxesMove_X    	->setChecked(bX);
    Dialog_AxesMove->checkBox_AxesMove_A    	->setChecked(bA);
    Dialog_AxesMove->checkBox_AxesMove_Y    	->setChecked(bY);
	Dialog_AxesMove->checkBox_AxesMove_Z    	->setChecked(bZ);
	Dialog_AxesMove->checkBox_AxesMove_ZIdle	->setChecked(bZIdle);
    Dialog_AxesMove->checkBox_AxesMove_X1   	->setChecked(bX1);
    Dialog_AxesMove->checkBox_AxesMove_A1   	->setChecked(bA1);
    Dialog_AxesMove->checkBox_AxesMove_Y1   	->setChecked(bY1);
	Dialog_AxesMove->checkBox_AxesMove_Z1    	->setChecked(bZ1);
	Dialog_AxesMove->checkBox_AxesMove_Z1Idle	->setChecked(bZ1Idle);
    Dialog_AxesMove->lineEdit_AxesMove_XPos 	->setText(QstrXPos);
    Dialog_AxesMove->lineEdit_AxesMove_APos 	->setText(QstrAPos);
    Dialog_AxesMove->lineEdit_AxesMove_YPos 	->setText(QstrYPos);
	Dialog_AxesMove->lineEdit_AxesMove_ZPos		->setText(QstrZPos);
	Dialog_AxesMove->lineEdit_AxesMove_ZIdlePos	->setText(QstrZIdlePos);
    Dialog_AxesMove->lineEdit_AxesMove_X1Pos	->setText(QstrX1Pos);
    Dialog_AxesMove->lineEdit_AxesMove_A1Pos	->setText(QstrA1Pos);
    Dialog_AxesMove->lineEdit_AxesMove_Y1Pos	->setText(QstrY1Pos);
	Dialog_AxesMove->lineEdit_AxesMove_Z1Pos	->setText(QstrZ1Pos);
	Dialog_AxesMove->lineEdit_AxesMove_Z1IdlePos->setText(QstrZ1IdlePos);
}

void Dialog_ProcessSetting_AxesMove::UpdatePage()
{
    if (Dialog_AxesMove->comboBox_AxesMove_Mode->currentIndex() == 2)
        Dialog_AxesMove->groupBox_AxesMove->setEnabled(true);
    else
        Dialog_AxesMove->groupBox_AxesMove->setEnabled(false);
}

void Dialog_ProcessSetting_AxesMove::ButtonOK()
{
    //获取参数
    QString QstrNote    	= Dialog_AxesMove->lineEdit_AxesMove_Note   	->text();
    int     iMode       	= Dialog_AxesMove->comboBox_AxesMove_Mode   	->currentIndex();
    int     iSpeed      	= Dialog_AxesMove->comboBox_AxesMove_Speed  	->currentIndex();
    bool    bX          	= Dialog_AxesMove->checkBox_AxesMove_X      	->isChecked();
    bool    bA          	= Dialog_AxesMove->checkBox_AxesMove_A      	->isChecked();
    bool    bY          	= Dialog_AxesMove->checkBox_AxesMove_Y      	->isChecked();
	bool    bZ          	= Dialog_AxesMove->checkBox_AxesMove_Z      	->isChecked();
	bool    bZIdle      	= Dialog_AxesMove->checkBox_AxesMove_ZIdle  	->isChecked();
    bool    bX1         	= Dialog_AxesMove->checkBox_AxesMove_X1     	->isChecked();
    bool    bA1         	= Dialog_AxesMove->checkBox_AxesMove_A1     	->isChecked();
    bool    bY1         	= Dialog_AxesMove->checkBox_AxesMove_Y1     	->isChecked();
	bool    bZ1         	= Dialog_AxesMove->checkBox_AxesMove_Z1     	->isChecked();
	bool    bZ1Idle     	= Dialog_AxesMove->checkBox_AxesMove_Z1Idle 	->isChecked();
    QString QstrXPos    	= Dialog_AxesMove->lineEdit_AxesMove_XPos   	->text();
    QString QstrAPos    	= Dialog_AxesMove->lineEdit_AxesMove_APos   	->text();
    QString QstrYPos    	= Dialog_AxesMove->lineEdit_AxesMove_YPos   	->text();
	QString QstrZPos    	= Dialog_AxesMove->lineEdit_AxesMove_ZPos		->text();
	QString QstrZIdlePos 	= Dialog_AxesMove->lineEdit_AxesMove_ZIdlePos	->text();
    QString QstrX1Pos   	= Dialog_AxesMove->lineEdit_AxesMove_X1Pos  	->text();
    QString QstrA1Pos   	= Dialog_AxesMove->lineEdit_AxesMove_A1Pos  	->text();
    QString QstrY1Pos   	= Dialog_AxesMove->lineEdit_AxesMove_Y1Pos  	->text();
	QString QstrZ1Pos   	= Dialog_AxesMove->lineEdit_AxesMove_Z1Pos		->text();
	QString QstrZ1IdlePos 	= Dialog_AxesMove->lineEdit_AxesMove_Z1IdlePos	->text();
    //设置参数
    m_pProcessAxesMove->SetAxesMoveNote(QstrNote);
    m_pProcessAxesMove->SetAxesMoveMode(iMode);
    m_pProcessAxesMove->SetAxesMoveSpeed(iSpeed);
    m_pProcessAxesMove->SetAxesMoveX(bX);
    m_pProcessAxesMove->SetAxesMoveA(bA);
    m_pProcessAxesMove->SetAxesMoveY(bY);
	m_pProcessAxesMove->SetAxesMoveZ(bZ);
	m_pProcessAxesMove->SetAxesMoveZIdle(bZIdle);
    m_pProcessAxesMove->SetAxesMoveX1(bX1);
    m_pProcessAxesMove->SetAxesMoveA1(bA1);
    m_pProcessAxesMove->SetAxesMoveY1(bY1);
	m_pProcessAxesMove->SetAxesMoveZ1(bZ1);
	m_pProcessAxesMove->SetAxesMoveZ1Idle(bZ1Idle);
    m_pProcessAxesMove->SetAxesMoveXPos(QstrXPos);
    m_pProcessAxesMove->SetAxesMoveAPos(QstrAPos);
    m_pProcessAxesMove->SetAxesMoveYPos(QstrYPos);
    m_pProcessAxesMove->SetAxesMoveZPos(QstrZPos);
    m_pProcessAxesMove->SetAxesMoveZIdlePos(QstrZIdlePos);
    m_pProcessAxesMove->SetAxesMoveX1Pos(QstrX1Pos);
    m_pProcessAxesMove->SetAxesMoveA1Pos(QstrA1Pos);
    m_pProcessAxesMove->SetAxesMoveY1Pos(QstrY1Pos);
	m_pProcessAxesMove->SetAxesMoveZ1Pos(QstrZ1Pos);
	m_pProcessAxesMove->SetAxesMoveZ1IdlePos(QstrZ1IdlePos);

    m_pProcessAxesMove->SetState(ItemState::Enable);
    m_pProcessAxesMove->SwitchState(ItemState::Enable);
    m_pProcessAxesMove->UpdateInfo();
    QDialog::accept();
}

void Dialog_ProcessSetting_AxesMove::ButtonCancel()
{
    //点击取消/ESC退出均不保存参数，还原节点状态
    reject();
}

void Dialog_ProcessSetting_AxesMove::reject()
{
    m_pProcessAxesMove->SwitchState();
    m_pProcessAxesMove->UpdateInfo();
    QDialog::reject();
}
