#include "oniviewdialog.h"
#include <QApplication>
//#include"LogicLevel/log.h"

//#include <mcheck.h>
//#include<assert.h>

int main(int argc, char *argv[])
{
  //LogInit(0);
  //assert(!setenv("MALLOC_TRACE","./malloc.log",1)) ;
  //mtrace();

  QApplication a(argc, argv);
  ONIViewDialog w;
  w.show();

  return a.exec();
}
