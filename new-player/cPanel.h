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
#include "wx/notebook.h"

class cPanel : public wxPanel
{
public:
	cPanel(wxFrame* parent);
	~cPanel();

public:
	wxMediaCtrl* new_video_mediactrl;
	wxMediaCtrl* landmark_video_mediactrl;
	wxMediaCtrl* convex_video_mediactrl;
	wxMediaCtrl* delaunay_video_mediactrl;
	wxMediaCtrl* original_video_mediactrl;

	wxString image_file_path;

	wxButton* m_playbutton;
	wxNotebook* m_notebook;

private:
	void PlayButtonCallback(wxCommandEvent&);
	wxWindow* CreateVisualPage(wxWindow* parent, wxMediaCtrl* m_mediactrl);

	wxDECLARE_EVENT_TABLE();
};

