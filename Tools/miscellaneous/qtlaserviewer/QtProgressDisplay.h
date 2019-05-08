
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : November 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Different template classes for showing the progress of a lengthy operation.
*
*--------------------------------------------------------------------*/

#ifndef __QT__PROGRESS_DISPLAY__H__
#define __QT__PROGRESS_DISPLAY__H__

#include "ProgressDisplay.h"
#include <QProgressBar>
#include <QLabel>
#include <QWidget>
#include <QApplication>

template <class T>
class QtProgressDisplay:public ProgressDisplay<int>, public QWidget
{
public:	
	QtProgressDisplay(T _interval=100,T _total=0,string msg="Progress")
	:ProgressDisplay<T>(_interval,_total,msg),bar(0),label(0)
	{
		Initialize();
	}
	
	QtProgressDisplay(T _total,string msg = "Progress ",T _times=100)
	:ProgressDisplay<T>(_total,msg,_times),bar(0),label(0)
	{
		Initialize();
	}
	
	void Initialize()
	{
		if(!bar && !label)
		{
			bar = new QProgressBar();
			label = new QLabel("Progress...",this);
			
			QVBoxLayout *layout = new QVBoxLayout;
	    	layout->addWidget(bar);
		    layout->addWidget(label);
	        this->setLayout(layout);
	        
	        QDesktopWidget* desktop = QApplication::desktop();
	        QRect rect = desktop->screenGeometry(this);
	        this->move(rect.center());
		}
    
		bar->setTextVisible(true);
		bar->setMinimum(0);
		bar->setMaximum(100);	
	}
	
	virtual void Step(const char* msg,int count)
	{
		lastStep = count;
		//We have called end so there is no point in more displays.
		if(lastDisplay<0)
			return;
			
		else if(!lastDisplay || (count - lastDisplay)>interval || count == total)
		{
			if(!msg)
				msg = defaultMessage.c_str();
			else
				defaultMessage = msg;
				
			QString str = QString("%1   %2  ").arg(msg).arg(count);				

			if(total>0)
			{
				str += QString("/ %1  ").arg(total);
				label->setText(str);
				
				double percent = (double)count/(double)total*100.00; 
				
				bar->setValue(percent);
				this->show();
				this->raise();
				QApplication::processEvents();
				

			}
			lastDisplay = count;
		}
	}
	
	//Reset the object
	virtual void Reset(const char* newMsg=NULL)
	{
		if(newMsg)
			defaultMessage = newMsg;
		lastDisplay = 0;
		tickCounter = 0;
		lastStep = 0;
		Step("  ",0);
	}
		

	
	virtual void End(const char* msg=NULL)
	{
		if(!msg)
			msg = defaultMessage.c_str();
			
		Step(msg,total);
		this->hide();
	}
	
	//Destructor.
	//If end is not called call it.
	~QtProgressDisplay()
	{
		//cerr<<"~ProgressDisplay for "<<this<<"  called\n"<<std::flush;
	}
private:
	QProgressBar* bar;
	QLabel* label;
	
};	

#endif// __PROGRESS_DISPLAY__H__
