#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , tcpSocket(new QTcpSocket(this))
{
    ui->setupUi(this);

    buttonInit();
    sliderInit();
    uiValidator();

    tcpResendTimer = new QTimer(this);
    connect(tcpResendTimer, &QTimer::timeout, this, &MainWindow::sendTcp);
    tcpResendTimer->start(resendTime);

    connect(tcpSocket, &QAbstractSocket::errorOccurred, this, &MainWindow::displayNetError);
    connect(tcpSocket, &QAbstractSocket::disconnected, this, &MainWindow::disconnectedHandle);
}


void MainWindow::sliderInit()
{
    ui->labelLinVel->setText(QString::number(ui->sliderLinVel->value() / 100.0, 'f', 2));
    ui->lableAngVel->setText(QString::number(ui->sliderAngVel->value() / 100.0, 'f', 2));
    ui->labelAxis1->setText(QString::number(ui->sliderAxis1->value() / 100.0, 'f', 2));
    ui->labelAxis2->setText(QString::number(ui->sliderAxis2->value() / 100.0, 'f', 2));
    ui->labelAxis3->setText(QString::number(ui->sliderAxis3->value() / 100.0, 'f', 2));
    ui->labelAxis4->setText(QString::number(ui->sliderAxis4->value() / 100.0, 'f', 2));
    ui->labelAxis5->setText(QString::number(ui->sliderAxis5->value() / 100.0, 'f', 2));

    txMsg.axis1 = ui->sliderAxis1->value();
    txMsg.axis2 = ui->sliderAxis2->value();
    txMsg.axis3 = ui->sliderAxis3->value();
    txMsg.axis4 = ui->sliderAxis4->value();
    txMsg.axis5 = ui->sliderAxis5->value();

    connect(ui->sliderLinVel, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
    connect(ui->sliderAngVel, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
    connect(ui->sliderAxis1, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
    connect(ui->sliderAxis2, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
    connect(ui->sliderAxis3, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
    connect(ui->sliderAxis4, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
    connect(ui->sliderAxis5, &QAbstractSlider::valueChanged, this, &MainWindow::sliderHandle);
}


void MainWindow::sliderHandle(int value)
{
    QObject* pObject = sender();
    
    if (pObject == ui->sliderLinVel) {
        ui->labelLinVel->setText(QString::number(value / 100.0, 'f', 2));
    } else if (pObject == ui->sliderAngVel) {
        ui->lableAngVel->setText(QString::number(value / 100.0, 'f', 2));
    } else if (pObject == ui->sliderAxis1) {
        ui->labelAxis1->setText(QString::number(value / 100.0, 'f', 2));
        txMsg.axis1 = value;
    } else if (pObject == ui->sliderAxis2) {
        ui->labelAxis2->setText(QString::number(value / 100.0, 'f', 2));
        txMsg.axis2 = value;
    } else if (pObject == ui->sliderAxis3) {
        ui->labelAxis3->setText(QString::number(value / 100.0, 'f', 2));
        txMsg.axis3 = value;
    } else if (pObject == ui->sliderAxis4) {
        ui->labelAxis4->setText(QString::number(value / 100.0, 'f', 2));
        txMsg.axis4 = value;
    } else if (pObject == ui->sliderAxis5) {
        ui->labelAxis5->setText(QString::number(value / 100.0, 'f', 2));
        txMsg.axis5 = value;
    }

    sendTcp();
    tcpResendTimer->start(resendTime);
}


void MainWindow::requestNewConnection()
{
    ui->butConnect->setEnabled(false);
    tcpSocket->abort();
    tcpSocket->connectToHost(ui->lineIp->text(), YOUBOT_PORT);
}


void MainWindow::sendTcp()
{
    if (tcpSocket->state() != QAbstractSocket::ConnectedState) {
        return;
    }
    
    tcpSocket->write(reinterpret_cast<char*>(&txMsg), YOUBOT_MSG_SIZE);
}


void MainWindow::displayNetError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        break;
    case QAbstractSocket::HostNotFoundError:
        QMessageBox::information(this, tr("Tcp Client"),
                                 tr("The host was not found. Please check the "
                                    "host name and port settings."));
        break;
    case QAbstractSocket::ConnectionRefusedError:
        QMessageBox::information(this, tr("Tcp Client"),
                                 tr("The connection was refused by the peer. "
                                    "Make sure the server is running, "
                                    "and check that the ip address "
                                    "settings are correct."));
        break;
    default:
        QMessageBox::information(this, tr("Tcp Client"),
                                 tr("The following error occurred: %1.")
                                 .arg(tcpSocket->errorString()));
    }

    ui->butConnect->setEnabled(true);
}


void MainWindow::disconnectedHandle()
{
    QMessageBox::warning(this, tr("Tcp Client"),
                                tr("Сonnection was lost."));

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
    connect(ui->butCompress, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butCompress, &QPushButton::released, this, &MainWindow::buttonHandle);
    connect(ui->butOpen, &QPushButton::pressed, this, &MainWindow::buttonHandle);
    connect(ui->butOpen, &QPushButton::released, this, &MainWindow::buttonHandle);

    connect(ui->butRandomMove, &QPushButton::pressed, this, &MainWindow::buttonRandHandle);
    connect(ui->butRandomMove, &QPushButton::released, this, &MainWindow::buttonHandle);
}


void MainWindow::uiValidator()
{
    QString IpRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
    QRegularExpression IpRegex ("^" + IpRange
                               + "(\\." + IpRange + ")"
                               + "(\\." + IpRange + ")"
                               + "(\\." + IpRange + ")$");
    QRegularExpressionValidator *ipValidator = new QRegularExpressionValidator(IpRegex, this);
    ui->lineIp->setValidator(ipValidator);
}


void MainWindow::buttonRandHandle()
{
    // TODO
}


void MainWindow::buttonHandle()
{
    double lin_vel = ui->sliderLinVel->value();
    double ang_vel = ui->sliderAngVel->value();

    txMsg.x_vel = lin_vel * 
        (static_cast<int>(ui->butForward->isDown() || ui->butLeftForward->isDown() || ui->butRightForward->isDown())
        - static_cast<int>(ui->butBack->isDown() || ui->butLeftBack->isDown() || ui->butRightBack->isDown()));
    
    txMsg.y_vel = ang_vel *
        (static_cast<int>(ui->butLeft->isDown() || ui->butLeftBack->isDown() || ui->butLeftForward->isDown())
        - static_cast<int>(ui->butRight->isDown() || ui->butRightBack->isDown() || ui->butRightForward->isDown()));

    txMsg.ang_speed = ang_vel * (static_cast<int>(ui->butRotLeft->isDown()) - static_cast<int>(ui->butRotRight->isDown()));

    if (ui->butOpen->isDown()) {
        txMsg.grip_cmd = GripControl::OPEN;
    } else if (ui->butCompress->isDown()) {
        txMsg.grip_cmd = GripControl::COMPRESS;
    } else {
        txMsg.grip_cmd = GripControl::WAIT;
    }

    sendTcp();
    tcpResendTimer->start(resendTime);
}


bool MainWindow::sliderKeyHandle(QKeyEvent *keyEvent)
{
    Qt::Key key = (Qt::Key)keyEvent->key();

    int shiftModifier = (keyEvent->modifiers() == Qt::ShiftModifier) ? -1 : 1;

    switch (keyEvent->nativeVirtualKey())
    {
    case Qt::Key_1:
        ui->sliderAxis1-> setValue(ui->sliderAxis1->value() + sliderShortkeyStep * shiftModifier);
        break;
    case Qt::Key_2:
        ui->sliderAxis2-> setValue(ui->sliderAxis2->value() + sliderShortkeyStep * shiftModifier);
        break;
    case Qt::Key_3:
        ui->sliderAxis3-> setValue(ui->sliderAxis3->value() + sliderShortkeyStep * shiftModifier);
        break;
    case Qt::Key_4:
        ui->sliderAxis4-> setValue(ui->sliderAxis4->value() + sliderShortkeyStep * shiftModifier);
        break;
    case Qt::Key_5:
        ui->sliderAxis5-> setValue(ui->sliderAxis5->value() + sliderShortkeyStep * shiftModifier);
        break;
    default:
        return false;
        break;
    }

    return true;
}


void MainWindow::keyPressEvent(QKeyEvent *keyEvent)
{  
    if (sliderKeyHandle(keyEvent)) {
        keyEvent->accept();
        return;
    }
    
    if(keyEvent->isAutoRepeat()) {
        keyEvent->accept();
        return;
    }

    switch (keyEvent->nativeVirtualKey())
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
    case Qt::Key_R:
        if (!ui->butOpen->isDown()) {
            pressBut(ui->butCompress);
        }
        break;
    case Qt::Key_F:
        if (!ui->butCompress->isDown()) {
            pressBut(ui->butOpen);
        }
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

    switch (keyEvent->nativeVirtualKey())
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
    case Qt::Key_R:
        releasBut(ui->butCompress);
        break;
    case Qt::Key_F:
        releasBut(ui->butOpen);
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
