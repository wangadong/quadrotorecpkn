/*
 * example.cpp
 *
 *  Created on: 2010-10-16
 *      Author: WANGADONG
 */

#include "wx/wx.h"
//define the applicatoin class
class MyApp: public wxApp {
public:
	virtual bool OnInit();
};
//define the main frame
class MyFrame: public wxFrame {
public:
	MyFrame(const wxString& title);
	void OnAbout(wxCommandEvent& event);
	void OnQuit(wxCommandEvent& event);
private:
	//declare the events table
DECLARE_EVENT_TABLE()
	;
};
DECLARE_APP(MyApp)
;
IMPLEMENT_APP(MyApp)
;
bool MyApp::OnInit() {
	MyFrame *frame = new MyFrame(wxT("my frame example"));
	frame->Show(true);
	return true;
}
BEGIN_EVENT_TABLE(MyFrame,wxFrame)
EVT_MENU(wxID_ABOUT,MyFrame::OnAbout) EVT_MENU(wxID_EXIT,MyFrame::OnQuit)
END_EVENT_TABLE()
void MyFrame::OnAbout(wxCommandEvent& event) {
	wxString msg;
	msg.Printf(wxT("Oh la la!C'est %s"), wxVERSION_STRING);
	wxMessageBox(msg, wxT("About Dialog"), wxOK | wxICON_INFORMATION, this);
}
void MyFrame::OnQuit(wxCommandEvent& event) {
	Close();
}
MyFrame::MyFrame(const wxString& title) :
	wxFrame(NULL, wxID_ANY, title) {
	wxMenu *fileMenu = new wxMenu;
	wxMenu *helpMenu = new wxMenu;
	helpMenu->Append(wxID_ABOUT, wxT("&About\tF1"), wxT("Show about dialog"));
	fileMenu->Append(wxID_EXIT, wxT("E&xit\tF4"), wxT("quit the app"));
wxMenuBar *menuBar=new wxMenuBar;
menuBar->Append(fileMenu,wxT("File"));
menuBar->Append(helpMenu,wxT("Help"));
SetMenuBar(menuBar);
CreateStatusBar(2);
SetStatusText(wxT("welcome to my first wxWidgets application"));

}

