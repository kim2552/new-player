#include "cPanel.h"

// Bind controls to callback functions
wxBEGIN_EVENT_TABLE(cPanel, wxPanel)
	EVT_BUTTON(ID_BUTTONPLAY, cPanel::PlayButtonCallback)
wxEND_EVENT_TABLE()

cPanel::cPanel(wxFrame* parent) : wxPanel(parent, wxID_ANY)
{
	//Sizer lays out objects in the panel
	wxSizer* sizer = new wxBoxSizer(wxVERTICAL);

	new_video_mediactrl = new wxMediaCtrl();
	landmark_video_mediactrl = new wxMediaCtrl();
	convex_video_mediactrl = new wxMediaCtrl();
	delaunay_video_mediactrl = new wxMediaCtrl();

	//Notebook pages
	m_notebook = new wxNotebook(this, wxID_ANY);
	m_notebook->AddPage(CreateVisualPage(m_notebook,new_video_mediactrl), "Face Swapped");
	m_notebook->AddPage(CreateVisualPage(m_notebook, delaunay_video_mediactrl), "Delaunay Triangulation");
	m_notebook->AddPage(CreateVisualPage(m_notebook, convex_video_mediactrl), "Convex Hull");
	m_notebook->AddPage(CreateVisualPage(m_notebook,landmark_video_mediactrl), "Facial Landmarks");

	sizer->Add(m_notebook, wxSizerFlags(0));

	this->SetSizerAndFit(sizer);
}

cPanel::~cPanel()
{
}

//Play button callback. Restarts the media controller
void cPanel::PlayButtonCallback(wxCommandEvent& event)
{
	new_video_mediactrl->Stop();
	landmark_video_mediactrl->Stop();
	convex_video_mediactrl->Stop();
	delaunay_video_mediactrl->Stop();

	new_video_mediactrl->Play();
	landmark_video_mediactrl->Play();
	convex_video_mediactrl->Play();
	delaunay_video_mediactrl->Play();

	if (new_video_mediactrl->GetState() == wxMEDIASTATE_PLAYING) {
		wxLogDebug("Media is playing.");
	}
	else {
		wxLogDebug("Media is not playing.");
	}
}

wxWindow* cPanel::CreateVisualPage(wxWindow* parent, wxMediaCtrl* m_mediactrl)
{
	wxPanel* page = new wxPanel(parent);
	wxSizer* sizer_page = new wxBoxSizer(wxVERTICAL);

	//Media display
	bool bOK = m_mediactrl->Create(page, ID_MEDIACTRL, wxEmptyString, wxPoint(0, 0), wxSize(600, 400));
	wxASSERT_MSG(bOK, "Could not create media control!");
	wxUnusedVar(bOK);

	//Play button
	m_playbutton = new wxButton();
	m_playbutton->Create(page, ID_BUTTONPLAY, "Play");
	m_playbutton->SetToolTip("Play Video");

	sizer_page->Add(m_mediactrl, wxSizerFlags(0));
	sizer_page->Add(m_playbutton, wxSizerFlags(0));

	page->SetSizer(sizer_page);

	return page;
}