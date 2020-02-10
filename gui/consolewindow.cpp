#include "consolewindow.h"
#include "ui_consolewindow.h"

#include <QTextEdit>
#include <QMutex>

void outcallback( const char* ptr, std::streamsize count, void* pTextEdit )
{
   static QMutex mutex;
  (void) count;
   mutex.lock();
  QTextEdit* p = static_cast< QTextEdit* >( pTextEdit );
  p->append( ptr );
  mutex.unlock();
}

ConsoleWindow::ConsoleWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ConsoleWindow)
{
    setVisible(false);
    move(50+9,700);

    ui->setupUi(this);

    link_signals();

    //now we link cout to the console
    debug_stream=new Q_DebugStream(std::cout, ui->txt_edit_console); //Redirect Console output to QTextEdit
    //I don't use qDebug but i can still activate this
    Q_DebugStream::registerQDebugMessageHandler(); //Redirect qDebug() output to QTextEdit

}

ConsoleWindow::~ConsoleWindow()
{
    delete ui;
    delete debug_stream;
}

void ConsoleWindow::link_signals()
{

}
