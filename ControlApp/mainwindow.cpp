#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , tcpSocket(new QTcpSocket(this))
{
    ui->setupUi(this);

    buttonInit();

    ui_validator();

    // connect(tcpSocket, &QIODevice::readyRead, this, &MainWindow::buttonHandle); // TODO
    // connect(tcpSocket, &QAbstractSocket::errorOccurred, this, &MainWindow::displayNetError);
}


void MainWindow::requestNewConnection()
{
    ui->butConnect->setEnabled(false);
    tcpSocket->abort();
    tcpSocket->connectToHost(ui->lineIp->text(), YOUBOT_PORT);
}


void MainWindow::displayNetError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        break;
    case QAbstractSocket::HostNotFoundError:
        QMessageBox::information(this, tr("Fortune Client"),
                                 tr("The host was not found. Please check the "
                                    "host name and port settings."));
        break;
    case QAbstractSocket::ConnectionRefusedError:
        QMessageBox::information(this, tr("Fortune Client"),
                                 tr("The connection was refused by the peer. "
                                    "Make sure the fortune server is running, "
                                    "and check that the host name and port "
                                    "settings are correct."));
        break;
    default:
        QMessageBox::information(this, tr("Fortune Client"),
                                 tr("The following error occurred: %1.")
                                 .arg(tcpSocket->errorString()));
    }

    ui->butConnect->setEnabled(true);
}


void MainWindow::buttonInit()
{
    connect(ui->butConnect, &QPushButton::pressed, this, &MainWindow::requestNewConnection);
    
    
    connect(ui->butLeftForward, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butLeftForward, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butLeft, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butLeft, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butLeftBack, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butLeftBack, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butForward, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butForward, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butStop, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butStop, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butBack, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butBack, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butRightForward, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butRightForward, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butRight, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butRight, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butRightBack, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butRightBack, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butRotLeft, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butRotLeft, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butRotRight, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butRotRight, &QPushButton::released, this, &MainWindow::buttonHandle);
}


void MainWindow::ui_validator()
{
    QString IpRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
    QRegularExpression IpRegex ("^" + IpRange
                               + "(\\." + IpRange + ")"
                               + "(\\." + IpRange + ")"
                               + "(\\." + IpRange + ")$");
    QRegularExpressionValidator *ipValidator = new QRegularExpressionValidator(IpRegex, this);
    ui->lineIp->setValidator(ipValidator);
}


void MainWindow::buttonHandle()
{
    double lin_vel = ui->sliderLinVel->value();
    double ang_vel = ui->sliderAngVel->value();

    tx_msg.x_vel = lin_vel * 
        (static_cast<int>(ui->butForward->isDown() || ui->butLeftForward->isDown() || ui->butRightForward->isDown())
        - static_cast<int>(ui->butBack->isDown()) || ui->butLeftBack->isDown() || ui->butRightBack->isDown());
    
    tx_msg.y_vel = ang_vel *
        (static_cast<int>(ui->butLeft->isDown() || ui->butLeftBack->isDown() || ui->butLeftForward->isDown())
        - static_cast<int>(ui->butRight->isDown()) || ui->butRightBack->isDown() || ui->butRightForward->isDown());

    tx_msg.ang_speed = ang_vel * (static_cast<int>(ui->butRotLeft->isDown()) - static_cast<int>(ui->butRotRight->isDown()));


}


void MainWindow::keyPressEvent(QKeyEvent *keyEvent)
{
    if(keyEvent->isAutoRepeat()) {
        keyEvent->accept();
        return;
    }

    Qt::Key key = (Qt::Key)keyEvent->key();

    switch (key)
    {
    case Qt::Key_Q:
        pressBut(ui->butLeftForward);
        break;
    case Qt::Key_W:
        pressBut(ui->butForward);
        break;
    case Qt::Key_E:
        pressBut(ui->butRightForward);
        break;
    case Qt::Key_A:
        pressBut(ui->butLeft);
        break;
    case Qt::Key_S:
        pressBut(ui->butStop);
        break;    
    qInfo() << ui->butLeftForward->isDown();
    case Qt::Key_D:
        pressBut(ui->butRight);
        break;
    case Qt::Key_Z:
        pressBut(ui->butLeftBack);
        break;
    case Qt::Key_X:
        pressBut(ui->butBack);
        break;
    case Qt::Key_C:
        pressBut(ui->butRightBack);
        break;
    case Qt::Key_Comma:
        pressBut(ui->butRotLeft);
        break;
    case Qt::Key_Period:
        pressBut(ui->butRotRight);
        break;
    case Qt::Key_P:
        pressBut(ui->butRandomMove);
        break;

    default:
        break;
    }

    keyEvent->accept();
}


void MainWindow::keyReleaseEvent(QKeyEvent *keyEvent)
{
    if(keyEvent->isAutoRepeat()) {
        keyEvent->accept();
        return;
    }

    Qt::Key key = (Qt::Key)keyEvent->key();

    switch (key)
    {
    case Qt::Key_Q:
        releasBut(ui->butLeftForward);
        break;
    case Qt::Key_W:
        releasBut(ui->butForward);
        break;
    case Qt::Key_E:
        releasBut(ui->butRightForward);
        break;
    case Qt::Key_A:
        releasBut(ui->butLeft);
        break;
    case Qt::Key_S:
        releasBut(ui->butStop);
        break;
    case Qt::Key_D:
        releasBut(ui->butRight);
        break;
    case Qt::Key_Z:
        releasBut(ui->butLeftBack);
        break;
    case Qt::Key_X:
        releasBut(ui->butBack);
        break;
    case Qt::Key_C:
        releasBut(ui->butRightBack);
        break;
    case Qt::Key_Comma:
        releasBut(ui->butRotLeft);
        break;
    case Qt::Key_Period:
        releasBut(ui->butRotRight );
        break;
    case Qt::Key_P:
        releasBut(ui->butRandomMove);
        break;

    default:
        break;
    }

    keyEvent->accept();
}


void MainWindow::pressBut(QPushButton* button)
{
    button->setDown(true);
    emit button->pressed();
}


void MainWindow::releasBut(QPushButton* button)
{
    button->setDown(false);
    emit button->released();
}


MainWindow::~MainWindow()
{
    delete ui;
}
