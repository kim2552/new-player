#include "cFrame.h"

// Bind controls to callback functions
wxBEGIN_EVENT_TABLE(cFrame, wxFrame)
	EVT_MENU(ID_FILE, cFrame::LoadVideoCallback)
	EVT_MENU(ID_IMG, cFrame::LoadImageCallback)
	EVT_MENU(ID_STEPS, cFrame::ToggleStepsCallback)
wxEND_EVENT_TABLE()

cFrame::cFrame(wxString title, wxPoint point, wxSize size) : 
	wxFrame(nullptr, wxID_ANY, title, point, size)
{
	// File menu
	m_menufile = new wxMenu;
	m_menufile->Append(ID_FILE, "Load Video");
	m_menufile->Append(ID_IMG, "Load Image");

	// Options menu
	m_menuoptions = new wxMenu;
	m_menuoptions->Append(ID_STEPS, "Toggle Steps");

	// Menu bar
	m_menubar = new wxMenuBar;
	m_menubar->Append(m_menufile, "File");
	m_menubar->Append(m_menuoptions, "Options");
	SetMenuBar(m_menubar);
	
	//Load media into controller
	m_panel = new cPanel(this);

	vidproc = new cVideoProcess();
}

cFrame::~cFrame()
{

}

void cFrame::LoadImageCallback(wxCommandEvent& event)
{
	wxString image_path = wxFileSelector(_T("Select file"), _T(""), _T(""), _T(""), _T("All files (*.*)|*.*"));
	m_panel->image_file_path = image_path;
}

void cFrame::LoadVideoCallback(wxCommandEvent& event)
{
	const wxString& new_video = NEW_VIDEO_PATH;
	const wxString& landmark_video = LANDMARK_VIDEO_PATH;
	const wxString& convex_video = CONVEX_VIDEO_PATH;
	const wxString& delaunay_video = DELAUNAY_VIDEO_PATH;

	wxString filename = wxFileSelector(_T("Select file"), _T(""), _T(""), _T(""), _T("All files (*.*)|*.*"));
	if (!filename.empty() && !m_panel->image_file_path.empty())
	{
		//TODO:: load video file here.
		vidproc->ProcessVideo(std::string(filename.mb_str()),std::string(m_panel->image_file_path.mb_str()));

		bool bOK = m_panel->new_video_mediactrl->Load(new_video);
		wxASSERT_MSG(bOK, "Could not load NEW_VIDEO_PATH media file!");

		bOK = m_panel->landmark_video_mediactrl->Load(landmark_video);
		wxASSERT_MSG(bOK, "Could not load LANDMARK_VIDEO_PATH media file!");

		bOK = m_panel->convex_video_mediactrl->Load(convex_video);
		wxASSERT_MSG(bOK, "Could not load CONVEX_VIDEO_PATH media file!");

		bOK = m_panel->delaunay_video_mediactrl->Load(delaunay_video);
		wxASSERT_MSG(bOK, "Could not load DELAUNAY_VIDEO_PATH media file!");

		bOK = m_panel->original_video_mediactrl->Load(filename);
		wxASSERT_MSG(bOK, "Could not load ORIG_VIDEO_PATH media file!");

		wxUnusedVar(bOK);
	}
}

void cFrame::ToggleStepsCallback(wxCommandEvent& event)
{
	vidproc->ToggleIntermediateSteps();
}