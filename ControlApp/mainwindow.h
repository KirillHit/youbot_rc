#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QDebug>
#include <QTcpSocket>
#include <QKeyEvent>
#include <QIntValidator>
#include <QMessageBox>
#include <QTimer>
#include <QRandomGenerator>

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

protected:
    void keyPressEvent(QKeyEvent *keyEvent) override;
    void keyReleaseEvent(QKeyEvent *keyEvent) override;

    void pressBut(QPushButton* button);
    void releasBut(QPushButton* button);
    void buttonInit();
    void buttonHandle();
    void buttonRandHandle();

    void sliderInit();
    void sliderHandle(int value);
    bool sliderKeyHandle(QKeyEvent *keyEvent);

    void uiValidator();

    void requestNewConnection();
    void disconnect();
    void sendTcp();
    void displayNetError(QAbstractSocket::SocketError socketError);
    void disconnectedHandle();
    void sendTcpComand();

private:
    Ui::MainWindow *ui;
    QTcpSocket *tcpSocket = nullptr;
    QTimer *tcpResendTimer = nullptr;

    YoubotMsg txMsg;
    const int resendTime = 100; // ms
    const int sliderShortkeyStep = 2;
};

#endif // MAINWINDOW_H
