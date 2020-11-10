#pragma once

/*
* Description:
*	A panel is a window on which controls are placed.
*	Contains the media controller and media buttons.
*
* Author: Joohoon Kim
* E-mail: joohoon.kim@outlook.com
*/

#include "globals.h"

// wxWidget libraries
#include "wx/wx.h"
#include "wx/mediactrl.h"

class cPanel : public wxPanel
{
public:
	cPanel(wxFrame* parent);
	~cPanel();

public:
	wxMediaCtrl* m_mediactrl;
	wxButton* m_playbutton;

private:
	void PlayButtonCallback(wxCommandEvent&);

	wxDECLARE_EVENT_TABLE();
};
