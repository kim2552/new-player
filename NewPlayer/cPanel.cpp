#include "cPanel.h"

// Bind controls to callback functions
wxBEGIN_EVENT_TABLE(cPanel, wxPanel)
	EVT_BUTTON(ID_BUTTONPLAY, cPanel::PlayButtonCallback)
wxEND_EVENT_TABLE()

cPanel::cPanel(wxFrame* parent) : wxPanel(parent, wxID_ANY)
{
	//Sizer lays out objects in the panel
	wxBoxSizer* sizer_v = new wxBoxSizer(wxVERTICAL);
	this->SetSizerAndFit(sizer_v);

	//Media display
	m_mediactrl = new wxMediaCtrl();
	bool bOK = m_mediactrl->Create(this, ID_MEDIACTRL, wxEmptyString, wxPoint(0,0),wxSize(600,400));
	wxASSERT_MSG(bOK, "Could not create media control!");
	wxUnusedVar(bOK);

	//Play button
	m_playbutton = new wxButton();
	m_playbutton->Create(this, ID_BUTTONPLAY, "Play");
	m_playbutton->SetToolTip("Play Video");

	sizer_v->Add(m_mediactrl, wxSizerFlags().Expand().Border());
	sizer_v->Add(m_playbutton, wxSizerFlags().Expand().Border());
}

cPanel::~cPanel()
{
}

//Play button callback. Restarts the media controller
void cPanel::PlayButtonCallback(wxCommandEvent& event)
{
	m_mediactrl->Stop();
	m_mediactrl->Play();
	if (m_mediactrl->GetState() == wxMEDIASTATE_PLAYING) {
		wxLogDebug("Media is playing.");
	}
	else {
		wxLogDebug("Media is not playing.");
	}
}