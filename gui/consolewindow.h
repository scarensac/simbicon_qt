#ifndef CONSOLEWINDOW_H
#define CONSOLEWINDOW_H

#include <QMainWindow>

#include "Utils/qdebugstream.h"
#include "Utils/std_redirector.h"

namespace Ui {
class ConsoleWindow;
}

class ConsoleWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ConsoleWindow(QWidget *parent = 0);
    ~ConsoleWindow();

protected:
    Q_DebugStream* debug_stream;
    StdRedirector<>* redir;

    void link_signals();

private:
    Ui::ConsoleWindow *ui;
};

#endif // CONSOLEWINDOW_H
