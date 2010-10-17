// ----------------------------------------------------------------------------
// private classes
// ----------------------------------------------------------------------------
// Define a new frame type: this is going to be our main frame
class GuiFrame: public wxFrame {
public:
	GuiFrame();
	virtual ~GuiFrame();
	void FrameLayout();
	void InitParams(void);
	void OnTimer(wxTimerEvent& WXUNUSED(event));
	void OnChComSelect(wxCommandEvent& WXUNUSED(event));
	void OnBtStart(wxCommandEvent& WXUNUSED(event));
	void OnBtStop(wxCommandEvent& WXUNUSED(event));
	void SetTextCtrl(wxString message);
	void SetStatusBar(wxString message);
	wxDb* OpenDB();
	void CloseDB(wxDb* DB);
	void AnalyseData(wxDb* DB);
	void SetAddress();
	int HandleError(wxString errmsg, wxDb *pDb);
public:
	wxThread *m_thread;
private:
	wxPanel *m_panel;
	wxNotebook* m_notebook;
	wxTimer m_timer;
	wxTextCtrl *m_txt_log;
	wxStatusBar* m_statusbar;
	wxGrid *gridinfo;
	wxCheckBox *comcheck[4];
	wxTextCtrl *nodeNum;
	wxTextCtrl *relayNum;
	wxButton *btStart;
	wxButton *btStop;
	int nodeAddressNumber;
	int relayAddressNumber;
	int recvNum;

	wxFile file;
	wxString SQLCommand;
	/* DB connection information */
	wxDbConnectInf *DBConnectInf;

	unsigned int MsgID;
	wxString nodeAddress[ADDRESS_NUMBER];
	// any class wishing to process wxWidgets events must use this macro
DECLARE_EVENT_TABLE()
};

class UartThread: public wxThread {
public:
	UartThread(int ComID, GuiFrame* Frame);
	virtual void *Entry();
	wxDb* OpenDB();
	void CloseDB(wxDb* DB);
	void CloseUART();
private:
	GuiFrame *m_frame;
	int m_int;
	wxIOBase* device;
	char recvBuf[COM_BUF_SIZE];
	wxString SQLCommand;
	/* DB connection information */
	wxDbConnectInf *DBConnectInf;

};
