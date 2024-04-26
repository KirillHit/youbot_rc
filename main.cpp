#include "mainwindow.h"
#include <QApplication>
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QFormLayout>
#include <QScreen>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QWidget window;
    window.setWindowTitle("ЁБОТ");

    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();

    window.setGeometry(screenGeometry.width() * 0.25, screenGeometry.height() * 0.25, screenGeometry.width() * 0.5, screenGeometry.height() * 0.5);

    QPalette pal = window.palette();
    pal.setColor(QPalette::Window, Qt::white);
    window.setPalette(pal);

    QLabel *label_1 = new QLabel("Linear speed x");
    QLineEdit *lineEdit_1 = new QLineEdit();

    QLabel *label_2 = new QLabel("Linear speed y");
    QLineEdit *lineEdit_2 = new QLineEdit();

    QLabel *label_3 = new QLabel("Angle speed z");
    QLineEdit *lineEdit_3 = new QLineEdit();

    QPushButton *button_1 = new QPushButton("Squeeze claw", &window);
    QPushButton *button_2 = new QPushButton("Unclench claw", &window);

    QLabel *label_info = new QLabel("Управление рукой");

    QLabel *label_hand_1 = new QLabel("Axis 1");
    QLineEdit *lineEdit_hand_1 = new QLineEdit();
    QLabel *label_hand_2 = new QLabel("Axis 2");
    QLineEdit *lineEdit_hand_2 = new QLineEdit();
    QLabel *label_hand_3 = new QLabel("Axis 3");
    QLineEdit *lineEdit_hand_3 = new QLineEdit();
    QLabel *label_hand_4 = new QLabel("Axis 4");
    QLineEdit *lineEdit_hand_4 = new QLineEdit();
    QLabel *label_hand_5 = new QLabel("Axis 5");
    QLineEdit *lineEdit_hand_5 = new QLineEdit();


    QFormLayout *formLayout = new QFormLayout();
    formLayout -> addRow(label_1, lineEdit_1);
    formLayout -> addRow(label_2, lineEdit_2);
    formLayout -> addRow(label_3, lineEdit_3);
    formLayout -> addRow(button_1, button_2);
    formLayout -> addRow(label_info);
    formLayout -> addRow(label_hand_1, lineEdit_hand_1);
    formLayout -> addRow(label_hand_2, lineEdit_hand_2);
    formLayout -> addRow(label_hand_3, lineEdit_hand_3);
    formLayout -> addRow(label_hand_4, lineEdit_hand_4);
    formLayout -> addRow(label_hand_5, lineEdit_hand_5);


    window.setLayout(formLayout);

    window.show();

    return app.exec();
}
