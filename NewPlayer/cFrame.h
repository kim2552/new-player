#pragma once

/*
* Description:
*	The main wxFrame.
*	A frame is a window whose size and position can (usually) be changed by the user.
* 
* Author: Joohoon Kim
* E-mail: joohoon.kim@outlook.com
*/

#include "globals.h"
#include "cPanel.h"
#include "cVideoProcess.h"

#include "wx/wx.h"

class cFrame : public wxFrame
{
public:
	//Frame title, position relative to window, size of frame
	cFrame(wxString title, wxPoint point, wxSize size);
	~cFrame();

private:
	wxMenuBar* m_menubar;
	wxMenu* m_menuoptions;
	wxMenu* m_menufile;
	cPanel* m_panel;

// wx events and callbacks
private:
	cVideoProcess* vidproc;

	void LoadVideoCallback(wxCommandEvent&);
	void ToggleStepsCallback(wxCommandEvent&);

	// Required for mapping events to callbacks
	wxDECLARE_EVENT_TABLE();
};

