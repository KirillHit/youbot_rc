#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QShortcut>
#include <QDebug>
#include <QTcpSocket>
#include <QKeyEvent>
#include <QIntValidator>
#include <QMessageBox>

#include "net_protocol.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

//public slots:
    

protected:
    void keyPressEvent(QKeyEvent *keyEvent) override;
    void keyReleaseEvent(QKeyEvent *keyEvent) override;
    //void closeEvent (QCloseEvent *event) override;

    void pressBut(QPushButton* button);
    void releasBut(QPushButton* button);
    void buttonInit();
    void buttonHandle();

    void ui_validator();

    void requestNewConnection();
    void displayNetError(QAbstractSocket::SocketError socketError);

private:
    Ui::MainWindow *ui;
    QTcpSocket *tcpSocket = nullptr;

    Msg tx_msg;
};

#endif // MAINWINDOW_H
