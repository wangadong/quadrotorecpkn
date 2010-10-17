// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

// for all others, include the necessary headers
#ifndef WX_PRECOMP
#include "wx/app.h"
#include "wx/log.h"
#include "wx/frame.h"
#include "wx/statusbr.h"
#include "wx/timer.h"
#include "wx/checkbox.h"
#include "wx/statbmp.h"
#include "wx/menu.h"
#include "wx/msgdlg.h"
#include "wx/textdlg.h"
#include "wx/sizer.h"
#include "wx/stattext.h"
#include "wx/bmpbuttn.h"
#include "wx/dcmemory.h"
#include <iostream>
#endif

#include "wx/bookctrl.h"
#include "wx/datetime.h"
#include "wx/numdlg.h"
#include "wx/statbox.h"
#include "wx/textctrl.h"
#include "wx/grid.h"
#include "wx/choice.h"
#include "wx/file.h"
#include "wx/colour.h"
#include "wx/gauge.h"

#include "ctb/timer.h"
#include "ctb/getopt.h"
#include "ctb/iobase.h"
#include "ctb/serport.h"

#include "App.h"
#include "daemon.h"

// Define a new application type, each program should derive a class from wxApp
class MyApp: public wxApp {

public:
	MyApp();
	virtual ~MyApp() {
	}
	;
	virtual bool OnInit();
};
BEGIN_EVENT_TABLE(GuiFrame, wxFrame)
EVT_BUTTON(ID_BT_START, GuiFrame::OnBtStart)
EVT_BUTTON(ID_BT_STOP, GuiFrame::OnBtStop)
//	EVT_TIMER(ID_TIMER, MyFrame::OnTimer)
END_EVENT_TABLE()

IMPLEMENT_APP( MyApp )

MyApp::MyApp()
{

}

bool MyApp::OnInit() {
	// create the main application window
	GuiFrame *frame = new GuiFrame();
	// and show it (the frames, unlike simple controls, are not shown when
	// created initially)
	frame->Show(true);
	return true;
}

GuiFrame::GuiFrame() :
	wxFrame((wxWindow *) NULL, wxID_ANY, _T("System V0 Test"), wxPoint(50, 50),
			wxSize(800, 800), wxDEFAULT_FRAME_STYLE | wxMAXIMIZE), m_timer
( this , ID_TIMER) {
	FrameLayout();
	InitParams();
}
void GuiFrame::FrameLayout() {
	m_panel = new wxPanel(this);
	wxFont stFont(12, wxFONTFAMILY_ROMAN, wxNORMAL, wxBOLD, false);
	wxFont font(12, wxFONTFAMILY_ROMAN, wxNORMAL, wxNORMAL, false);

	btStart = new wxButton(m_panel, ID_BT_START, wxT("开始"), wxDefaultPosition,
			wxDefaultSize);
	btStop = new wxButton(m_panel, ID_BT_STOP, wxT("结束"), wxDefaultPosition,
			wxDefaultSize);

	/* 定义wxBoxSizer，保证测试结果区域内容随窗口变化而变化 */
	wxBoxSizer* box = new wxBoxSizer(wxVERTICAL);

	wxStaticText * stCom = new wxStaticText(m_panel, wxID_ANY, wxT("串口选择"),
			wxDefaultPosition, wxDefaultSize);
	stCom->SetFont(stFont);

	wxStaticText * stNodeNum = new wxStaticText(m_panel, wxID_ANY, wxT(
			"Node数量："), wxDefaultPosition, wxDefaultSize);
	stNodeNum->SetFont(stFont);

	wxStaticText * stRelayNum = new wxStaticText(m_panel, wxID_ANY, wxT(
			"Relay数量："), wxDefaultPosition, wxDefaultSize);
	stRelayNum->SetFont(stFont);

	nodeNum = new wxTextCtrl(m_panel, ID_TxtCtl_NODEB, wxEmptyString,
			wxDefaultPosition, wxSize(80, 20));
	relayNum = new wxTextCtrl(m_panel, wxID_ANY, wxEmptyString,
			wxDefaultPosition, wxSize(80, 20));

	wxBoxSizer *btBoxer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer *bsNode = new wxBoxSizer(wxHORIZONTAL);

	btBoxer->Add(stCom, 0, wxALL | wxEXPAND, 5);
	wxString comID;
	for (int comNum = 0; comNum < 4; comNum++) {
		comID.Printf(wxT("COM%2d"), comNum + 1);
		comcheck[comNum] = new wxCheckBox(m_panel, wxID_ANY, comID,
				wxDefaultPosition, wxSize(80, 30));
		comcheck[comNum]->SetFont(font);
		btBoxer->Add(comcheck[comNum], 0, wxLEFT, 5);
	}
	btBoxer->Add(btStart, 0, wxLEFT, 5);
	btBoxer->Add(btStop, 0, wxLEFT, 5);

	bsNode->Add(stNodeNum, 0, wxLEFT, 5);
	bsNode->Add(nodeNum, 0, wxLEFT, 5);
	bsNode->Add(stRelayNum, 0, wxLEFT, 5);
	bsNode->Add(relayNum, 0, wxLEFT, 5);

	wxBoxSizer *infoSizer = new wxStaticBoxSizer(new wxStaticBox(m_panel,
			wxID_ANY, _T("统计信息")), wxVERTICAL);
	m_txt_log = new wxTextCtrl(m_panel, ID_TEXTLOG, _T(""), wxDefaultPosition,
			wxDefaultSize, wxTE_MULTILINE | wxTE_READONLY);

	/* 表格 */
	gridinfo = new wxGrid(m_panel, wxID_ANY, wxDefaultPosition,
			wxSize(600, 300), wxWANTS_CHARS, wxT(""));
	gridinfo->CreateGrid(0, 0);
	gridinfo->AppendCols(7);
	/* 行高 */
	gridinfo->SetDefaultRowSize(20, true);
	/* 列宽 */
	gridinfo->SetDefaultColSize(100, true);

	/* 标题 */
	gridinfo->SetColLabelValue(0, wxT("Address"));
	gridinfo->SetColLabelValue(1, wxT("NumRecv"));
	gridinfo->SetColLabelValue(2, wxT("MissPktNum"));
	gridinfo->SetColLabelValue(3, wxT("AvgWriteTimes"));
	gridinfo->SetColLabelValue(4, wxT("AvgSendTimes"));
	gridinfo->SetColLabelValue(5, wxT("MaxWriteTimes"));
	gridinfo->SetColLabelValue(6, wxT("MaxSendTimes"));
	/* 自动设置大小 */
	gridinfo->AutoSizeColumns();
	infoSizer->Add(m_txt_log, 1, wxALL | wxEXPAND, 0);
	infoSizer->Add(gridinfo, 1, wxEXPAND, 0);
	/* 加入两个内容到box中，这样这两个部分可以随窗口变化而自动缩放 */
	box->Add(btBoxer, 0, wxALL | wxEXPAND, 2);
	/* 加入两个内容到box中，这样这两个部分可以随窗口变化而自动缩放 */
	box->Add(bsNode, 0, wxALL | wxEXPAND, 2);
	box->Add(infoSizer, 0, wxALL | wxEXPAND, 2);
	m_statusbar = this->CreateStatusBar();
	/* 定义box到Panel中 */
	m_panel->SetSizer(box);
	box->Fit(this);
}

void GuiFrame::InitParams(void) {
	recvNum = 0;
	btStop->Enable(false);
}

void GuiFrame::SetAddress() {
	int i;
	unsigned char addrLow;
	unsigned char addrHigh;
	addrHigh = 0xA1;
	addrLow = 0x01;
	for (i = 0; i < nodeAddressNumber; i++) {
		nodeAddress[i].Printf(wxT("%02X%02X"), addrHigh, addrLow);
		addrHigh++;
	}
}
wxDb* GuiFrame::OpenDB() {
	/*Database connection */
	wxDb *DB;

	wxString message;
	/* ODBC MySQL*/
	wxString table = wxT("nwk_1");
	DBConnectInf = new wxDbConnectInf(NULL, wxT("hyena"), wxT("root"), wxT(
			"hyenae"));
	// Error checking....
	if (!DBConnectInf || !DBConnectInf->GetHenv()) {
		//return HandleError(wxT("DB ENV ERROR: Cannot allocate ODBC env handle"));
		wxMessageBox(wxT("DB ENV ERROR: Cannot allocate ODBC env handle"));
		message.Printf(wxT("DB ENV ERROR: Cannot allocate ODBC env handle"));
	}
	DB = wxDbGetConnection(DBConnectInf);
	if (!DB) {
		//return HandleError(wxT("CONNECTION ERROR - Cannot get DB connection"));
		wxMessageBox(wxT("CONNECTION ERROR - Cannot get DB connection"));
		message.Printf(wxT("CONNECTION ERROR - Cannot get DB connection"));
	}
	if (!DB->TableExists(table.c_str())) {
		SQLCommand.Printf(
				wxT("CREATE TABLE nwk_1 (ID integer PRIMARY KEY AUTO_INCREMENT UNIQUE NOT NULL,RawMessage text,MsgID int,MsgIDB int,WriteTimes int,SendTimes int,Address char(4),Time datetime)"));
		if (!DB->ExecSql(SQLCommand.c_str())) {
		}
	}
	return DB;
}

void GuiFrame::CloseDB(wxDb* DB) {
	/* ODBC MySQL */
	/**
	 * Free the cached connection
	 * (meaning release it back in to the cache of datasource
	 * connections) for the next time a call to wxDbGetConnection()
	 * is made.
	 */
	wxDbFreeConnection(DB);
	DB = NULL;
	/**
	 * CLEANUP BEFORE EXITING APP
	 * The program is now ending, so we need to close
	 * any cached connections that are still being
	 * maintained.
	 */
	wxDbCloseConnections();
	/**
	 * Release the environment handle that was created
	 * for use with the ODBC datasource connections
	 */
	//	wxDELETE(DBConnectInf);
}

void GuiFrame::OnBtStart(wxCommandEvent& WXUNUSED(event)) {

	btStart->Enable(false);
	btStop->Enable(true);
	m_txt_log->Clear();
	if (gridinfo->GetRows() != 0) {
		gridinfo->DeleteRows(0, gridinfo->GetRows());
	}
	if (nodeNum->GetValue().IsNumber()) {
		nodeAddressNumber = wxAtoi(nodeNum->GetValue());
		gridinfo->AppendRows(nodeAddressNumber);
	} else {
		wxBell();
		nodeNum->SetValue(wxT(""));
	}
	if (relayNum->GetValue().IsNumber()) {
		relayAddressNumber = wxAtoi(relayNum->GetValue());
	} else {
		wxBell();
		relayNum->SetValue(wxT(""));
	}
	SetAddress();
	wxDb* DB = OpenDB();
	/* 清空表 */
	SQLCommand.Printf(wxT("DELETE FROM nwk_1"));
	DB->ExecSql(SQLCommand.c_str());
	/* 清空ID递增 */
	SQLCommand.Printf(wxT("ALTER TABLE nwk_1 AUTO_INCREMENT = 1;"));
	DB->ExecSql(SQLCommand.c_str());
	CloseDB(DB);
	wxDELETE(DBConnectInf);
	for (int i = 0; i < 4; i++) {
		if (comcheck[i]->IsChecked()) {
			m_thread = new UartThread(i + 1, this);
			if (m_thread->Create() != wxTHREAD_NO_ERROR) {
				wxLogError(wxT("Can't create thread!"));
			}
			if (m_thread->Run() != wxTHREAD_NO_ERROR) {
				wxMessageBox(wxT("Couldn't run thread!"));
			}
		}
	}
}
int GuiFrame::HandleError(wxString errmsg, wxDb *pDb = NULL) {
	// Retrieve all the error message for the errors that occurred
	wxString allErrors;
	if (!pDb == NULL)
		// Get the database errors and append them to the error message
		allErrors = wxDbLogExtendedErrorMsg(errmsg.c_str(), pDb, 0, 0);
	else
		allErrors = errmsg;

	// Do whatever you wish with the error message here
	// wxLogDebug() is called inside wxDbLogExtendedErrorMsg() so this
	// console program will show the errors in the console window,
	// but these lines will show the errors in RELEASE builds also
	wxFprintf(stderr, wxT("\n%s\n"), allErrors.c_str());
	wxMessageBox(allErrors);
	fflush(stderr);

	return 1;
}

void GuiFrame::OnBtStop(wxCommandEvent& WXUNUSED(event)) {
	/* does the thread still exist? */
	if (m_thread) {
		m_thread->Delete();
	}
	wxDb* DB = OpenDB();
	AnalyseData(DB);
	CloseDB(DB);
	wxDELETE(DBConnectInf);
	SetStatusBar(wxT("设备已关闭"));
	btStart->Enable(true);
	btStop->Enable(false);
}
/* 没有启用Timer，暂时保留 */
void GuiFrame::OnTimer(wxTimerEvent& WXUNUSED(event)) {
}

void GuiFrame::AnalyseData(wxDb* DB) {
	LONG mID = 0;
	LONG recordTotal = 0;
	LONG record = 0;
	SDWORD cbReturned = 0;
	LONG writeTimes = 0;
	double writeTimesSum = 0;
	LONG writeTimesMax = 0;
	wxString text;

	SQLCommand.Printf(wxT("SELECT COUNT(*) AS Total FROM nwk_1"));
	DB->ExecSql(SQLCommand.c_str());
	if (DB->GetNext()) {
		DB->GetData(1, SQL_C_LONG, &recordTotal, 0, &cbReturned);
	} else {
		recordTotal = 0;
		return;
	}
	SQLCommand.Printf(
			wxT("SELECT COUNT(*) AS Total FROM (SELECT DISTINCT substr(RawMessage,1,5) AS MID FROM nwk_1) AS A"));
	DB->ExecSql(SQLCommand.c_str());
	if (DB->GetNext()) {
		DB->GetData(1, SQL_C_LONG, &record, 0, &cbReturned);
	} else {
		record = 0;
		return;
	}
	SQLCommand.Printf(
			wxT("SELECT MAX(MID) FROM (SELECT DISTINCT CONCAT(substr(RawMessage,1,2),substr(RawMessage,4,2)) AS MID FROM nwk_1 ORDER BY Time) AS A"));
	DB->ExecSql(SQLCommand.c_str());
	if (DB->GetNext()) {
		DB->GetData(1, SQL_C_LONG, &mID, 5, &cbReturned);
	} else {
		mID = 0;
		return;
	}
	SQLCommand.Printf(
			wxT("SELECT CONCAT(substr(RawMessage,1,2),substr(RawMessage,4,2)) AS MID,substr(RawMessage,7,2) AS TI FROM nwk_1 GROUP by MID ORDER BY Time "));
	DB->ExecSql(SQLCommand.c_str());
	while (DB->GetNext()) {
		DB->GetData(2, SQL_C_LONG, &writeTimes, 5, &cbReturned);
		writeTimesSum += writeTimes;
		if (writeTimesMax < writeTimes) {
			writeTimesMax = writeTimes;
		}
	}

	text.Printf(
			wxT(""
					"<table border=\"1\" bordercolor=\"#000000\" cellpadding=\"3\" cellspacing=\"0\" id=\"ve32\" width=\"100%\">"
					"<tbody>"
					"<tr>"
					"<td width=\"16.666666666666668%\"><br>"
					"</td>"
					"<td width=\"16.666666666666668%\">平均Write次数</td>"
					"<td width=\"16.666666666666668%\">最大Write次数</td>"
					"<td width=\"16.666666666666668%\">丢包率</td>"
					"<td width=\"16.666666666666668%\">总包数</td>"
					"<td width=\"16.666666666666668%\">重复率</td>"
					"</tr>"
					"<tr>"
					"<td width=\"16.666666666666668%\">A(向B发送)</td>"
					"<td width=\"16.666666666666668%\">%0.2f<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">%d<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">--</td>"
					"<td width=\"16.666666666666668%\">%d<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">--</td>"
					"</tr>"
					"<tr>"
					"<td width=\"16.666666666666668%\">B(向C发送)<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">%0.2f<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">%d<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">--</td>"
					"<td width=\"16.666666666666668%\">%d<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">--</td>"
					"</tr>"
					"<tr>"
					"<td width=\"16.666666666666668%\">C(接收)</td>"
					"<td width=\"16.666666666666668%\">--</td>"
					"<td width=\"16.666666666666668%\">--</td>"
					"<td width=\"16.666666666666668%\">%0.2f<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">%d<br>"
					"</td>"
					"<td width=\"16.666666666666668%\">%0.2f<br>"
					"</td>"
					"</tr>"
					"</tbody>"
					"</table>"
			), 0, 0, writeTimesSum / mID, writeTimesMax,
			(double) ((double) (mID - record) / (double) mID), mID,
			(double) ((double) (recordTotal - record) / (double) mID));
	SetTextCtrl(text);

}
GuiFrame::~GuiFrame() {

}

void GuiFrame::SetTextCtrl(wxString message) {
	m_txt_log->AppendText(message);
}
void GuiFrame::SetStatusBar(wxString message) {
	m_statusbar->SetStatusText(message);
}
UartThread::UartThread(int ComID, GuiFrame* Frame) :
	wxThread() {
	m_int = ComID;
	m_frame = Frame;
}

wxDb* UartThread::OpenDB() {
	/*Database connection */
	wxDb *DB;

	wxString message;
	/* ODBC MySQL*/
	wxString table = wxT("nwk_1");
	DBConnectInf = new wxDbConnectInf(NULL, wxT("hyena"), wxT("root"), wxT(
			"hyenae"));
	// Error checking....
	if (!DBConnectInf || !DBConnectInf->GetHenv()) {
		//return HandleError(wxT("DB ENV ERROR: Cannot allocate ODBC env handle"));
		wxMessageBox(wxT("DB ENV ERROR: Cannot allocate ODBC env handle"));
		message.Printf(wxT("DB ENV ERROR: Cannot allocate ODBC env handle"));
		wxMutexGuiEnter();
		m_frame->SetStatusBar(message);
		wxMutexGuiLeave();
	}
	DB = wxDbGetConnection(DBConnectInf);
	if (!DB) {
		//return HandleError(wxT("CONNECTION ERROR - Cannot get DB connection"));
		wxMessageBox(wxT("CONNECTION ERROR - Cannot get DB connection"));
		message.Printf(wxT("CONNECTION ERROR - Cannot get DB connection"));
		wxMutexGuiEnter();
		m_frame->SetStatusBar(message);
		wxMutexGuiLeave();
	}
	return DB;
}

/* 关闭数据库 */
void UartThread::CloseDB(wxDb* DB) {
	/* ODBC MySQL */
	/**
	 * Free the cached connection
	 * (meaning release it back in to the cache of datasource
	 * connections) for the next time a call to wxDbGetConnection()
	 * is made.
	 */
	if (DB) {
		wxDbFreeConnection(DB);
		DB = NULL;
		/**
		 * CLEANUP BEFORE EXITING APP
		 * The program is now ending, so we need to close
		 * any cached connections that are still being
		 * maintained.
		 */
		wxDbCloseConnections();
		/**
		 * Release the environment handle that was created
		 * for use with the ODBC datasource connections
		 */
		//		wxDELETE( DBConnectInf);
	}
}
/* 关闭串口 */
void UartThread::CloseUART() {
	if (device->IsOpen()) {
		device->Close();
	}
	delete device;
}

void *UartThread::Entry() {
	char deviceCharName[4];
	int comID;
	size_t readedBytes = 0;
	wxString message;
	wxString deviceName;
	wxString receiveStr;
	wxString nodeAddress;
	unsigned short mID;
	unsigned short i;
	wxString msgID;
	double power;
	wxDateTime now;
	device = new wxSerialPort();
	comID = m_int;
	if (!device->IsOpen()) {
		sprintf(deviceCharName, "com%d", comID);
		deviceName.Printf(wxT("COM%d"), comID);
		if (device->Open(deviceCharName) < 0) {
			message.Printf(wxT("打开%s错误\n"), deviceName.c_str());
			wxMutexGuiEnter();
			m_frame->SetStatusBar(message);
			wxMutexGuiLeave();
			/* 关闭串口 */
			delete device;
			/* 关闭线程指针  */
			wxMutexGuiEnter();
			m_frame->m_thread = NULL;
			wxMutexGuiLeave();
			return NULL;
		}
		message.Printf(wxT("打开%s成功\n"), deviceName.c_str());
		wxMutexGuiEnter();
		m_frame->SetStatusBar(message);
		wxMutexGuiLeave();
		/* 设置串口 比特率 */
		((wxSerialPort*) device)->SetBaudRate(wxBAUD_9600);
		message.Printf(wxT("设备%s已连接"), deviceName.c_str());
		wxMutexGuiEnter();
		m_frame->SetStatusBar(message);
		wxMutexGuiLeave();
	}

	wxDb* DB = OpenDB();
	int SQLCnt = 100;
	while (1) {
		SQLCnt--;
		if (!SQLCnt) {
			CloseDB(DB);
			DB = OpenDB();
			SQLCnt = 100;
		}
		/* 线程注销 */
		if (TestDestroy()) {
			/* 关闭串口 */
			CloseUART();
			/* 关闭数据库 */
			CloseDB(DB);
			wxDELETE(DBConnectInf);
			/* 关闭线程指针 */
			wxMutexGuiEnter();
			m_frame->m_thread = NULL;
			wxMutexGuiLeave();
			/* 跳出循环，退出线程 */
			break;
		}
		readedBytes = device->Readv(recvBuf, MESSAGE_LENGTH, READV_TIMEOUT);
		if (readedBytes == MESSAGE_LENGTH) {
			receiveStr.Clear();
			/* 插入数据库 */
			for (i = 0; i < MESSAGE_LENGTH; i++) {
				message.Printf(wxT("%02X "), (unsigned char) recvBuf[i]);
				receiveStr += message;
			}
			wxMutexGuiEnter();
			m_frame->SetStatusBar(receiveStr);
			wxMutexGuiLeave();
			mID = (unsigned char) recvBuf[9];
			mID = (mID << 8) + (unsigned char) recvBuf[10];
			msgID.Printf(wxT("%04X"), (unsigned short) mID);
			power = (((unsigned char) (recvBuf[5] - 0x80)) * 5) / 128.0;
			nodeAddress.Printf(wxT("%02X%02X"), (unsigned char) recvBuf[2],
					(unsigned char) recvBuf[2 + 1]);
			now = wxDateTime::GetTimeNow();
			SQLCommand.Printf(
					wxT("INSERT INTO nwk_1 (Time,RawMessage,MsgID,MsgIDB,Address) VALUES ('%s','%s','%d','%2.2f','%s')"),
					now.Format(wxT("%Y-%m-%d %H:%M:%S")).c_str(),
					receiveStr.c_str(), mID, power,
					nodeAddress.c_str());
			DB->ExecSql(SQLCommand);
			DB->CommitTrans();
		} else {
			for (i = 0; i < readedBytes; i++) {
				device->PutBack(recvBuf[i]);
			}
		}
	}
	return NULL;
}
