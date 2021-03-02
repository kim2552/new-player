#pragma once

/*
* Description:
*	The main wxApp class.
* 	Instantiates the main wxFrame.
* 
* Author: Joohoon Kim
* E-mail: joohoon.kim@outlook.com
*/

#include "globals.h"

#include "wx/wx.h"

#include "cFrame.h"

class cApp : public wxApp
{
public:
	cApp();
	~cApp();

public:
	virtual bool OnInit();

private:
	cFrame* m_frame;

};

