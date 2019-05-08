
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
*   File made : Feb 2006 
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Utility functions for Qt
*
*--------------------------------------------------------------------*/
#ifndef __QT_UTILITY___H__
#define __QT_UTILITY___H__

template<class T>
QLayout* HorizontalCenteredLayout(T* widget)
{
	QHBoxLayout* hl = new QHBoxLayout;
	hl->addStretch();
	hl->addWidget(widget);
	hl->addStretch();
	
	return hl;
}

template<class T>
QLayout* VerticalCenteredLayout(T* widget)
{
	QVBoxLayout* hl = new QVBoxLayout;
	hl->addStretch();
	hl->addWidget(widget);
	hl->addStretch();
	
	return hl;
}


#endif //__QT_UTILITY___H__