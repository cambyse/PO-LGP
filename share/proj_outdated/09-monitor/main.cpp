/*
    Copyright (C) 2007, 2008  Marc Toussaint.
    email: mtoussai@cs.tu-berlin.de

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    A copy of the GNU Lesser General Public License can usually be found
    at http://www.gnu.org/copyleft/lesser.html; if not, write to the Free
    Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
    02110-1301 USA
*/

#define MT_IMPLEMENTATION

#ifdef MT_QT
#  include"gui.h"
#endif


int main(int argn,char** argv){
  QApplication *app;
  Gui *gui;
  
  app = new QApplication(argn, argv);
  gui = new Gui;
  gui->exec();

  delete gui;
  delete app;
  
  floatA b;
  cout <<b;
  return 0;
}


