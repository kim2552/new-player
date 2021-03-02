#include "cApp.h"

// cApp becomes the main function.
wxIMPLEMENT_APP(cApp);

cApp::cApp()
{
}

cApp::~cApp()
{
}

// The main function, instantiates the main frame.
bool cApp::OnInit()
{
	m_frame = new cFrame("NewPlayer",wxPoint(30,30),wxSize(600,600));
	m_frame->Show();

	return true;
}