#include "gui/mainwindow.h"
#include <QApplication>
#include <iostream>

#include "Utils/Utils.h"
#include "Globals.h"
#include "Core/SimGlobals.h"
#include <QGraphicsTextItem>

//#define MEMORY_CHECK
/*
int main(int argc, char *argv[])
{
    Globals::data_folder_path = get_folder_path("configuration_data", 5);
//    Globals::drawDesiredPose=true;
    SimGlobals::water_level=0.25;
    SimGlobals::liquid_density=1000;
//    SimGlobals::desiredHeading=3.14;
//    SimGlobals::desiredHeading=1.570796;
//    Globals::drawCollisionPrimitives=true;
    std::cout<<"data_folder_path:"<<Globals::data_folder_path<<std::endl;
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
//*/

#ifdef MEMORY_CHECK
// Necessary includes and defines for memory leak detection:
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>

// Code to display the memory leak report
// We use a custom report hook to filter out Qt's own memory leaks
// Credit to Andreas Schmidts - http://www.schmidt-web-berlin.de/winfig/blog/?p=154

_CRT_REPORT_HOOK prevHook;

int customReportHook(int /* reportType */, char* message, int* /* returnValue */) {
  // This function is called several times for each memory leak.
  // Each time a part of the error message is supplied.
  // This holds number of subsequent detail messages after
  // a leak was reported
  const int numFollowupDebugMsgParts = 2;
  static bool ignoreMessage = false;
  static int debugMsgPartsCount = 0;

  // check if the memory leak reporting starts
  if ((strncmp(message,"Detected memory leaks!\n", 10) == 0)
    || ignoreMessage)
  {
    // check if the memory leak reporting ends
    if (strncmp(message,"Object dump complete.\n", 10) == 0)
    {
      _CrtSetReportHook(prevHook);
      ignoreMessage = false;
    } else
      ignoreMessage = true;

    // something from our own code?
    if(strstr(message, ".cpp") == NULL)
    {
      if(debugMsgPartsCount++ < numFollowupDebugMsgParts)
        // give it back to _CrtDbgReport() to be printed to the console
        return FALSE;
      else
        return TRUE;  // ignore it
    } else
    {
      debugMsgPartsCount = 0;
      // give it back to _CrtDbgReport() to be printed to the console
      return FALSE;
    }
  } else
    // give it back to _CrtDbgReport() to be printed to the console
    return FALSE;
}


#endif

int main(int argc, char *argv[]) {
    int return_val=0;

#ifdef MEMORY_CHECK
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
    prevHook = _CrtSetReportHook(customReportHook);
#endif
    // _CrtSetBreakAlloc(157); // Use this line to break at the nth memory allocation

    //*
    Globals::data_folder_path = get_folder_path("configuration_data", 5);
//    Globals::drawDesiredPose=true;
    SimGlobals::water_level=0.25;
    SimGlobals::liquid_density=1000;
//    SimGlobals::desiredHeading=3.14;
//    SimGlobals::desiredHeading=1.570796;
//    Globals::drawCollisionPrimitives=true;
    std::cout<<"data_folder_path:"<<Globals::data_folder_path<<std::endl;
    QApplication a(argc, argv);
    MainWindow w(argc, argv);
    w.show();
    if (argc>5){
        w.start_click();
    }

    return_val= a.exec();
    //*/
    // Once the app has finished running and has been deleted,
    // we run this command to view the memory leaks:
#ifdef MEMORY_CHECK
    _CrtDumpMemoryLeaks();
#endif
    //return appError;
    return return_val;
}


