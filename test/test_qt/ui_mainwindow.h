/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Sat May 16 12:53:26 2015
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLineEdit *ledt_ip;
    QPushButton *btn_ipok;
    QGroupBox *groupBox;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *lb_angle_xy;
    QLabel *lb_angle_yz;
    QLabel *lb_angle_zx;
    QPushButton *btn_get;
    QLabel *label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(502, 359);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        ledt_ip = new QLineEdit(centralWidget);
        ledt_ip->setObjectName(QString::fromUtf8("ledt_ip"));
        ledt_ip->setGeometry(QRect(20, 30, 171, 21));
        btn_ipok = new QPushButton(centralWidget);
        btn_ipok->setObjectName(QString::fromUtf8("btn_ipok"));
        btn_ipok->setGeometry(QRect(210, 30, 75, 23));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 100, 461, 141));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(30, 40, 54, 12));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(30, 70, 54, 12));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(30, 100, 54, 12));
        lb_angle_xy = new QLabel(groupBox);
        lb_angle_xy->setObjectName(QString::fromUtf8("lb_angle_xy"));
        lb_angle_xy->setGeometry(QRect(100, 40, 54, 12));
        lb_angle_xy->setAlignment(Qt::AlignCenter);
        lb_angle_yz = new QLabel(groupBox);
        lb_angle_yz->setObjectName(QString::fromUtf8("lb_angle_yz"));
        lb_angle_yz->setGeometry(QRect(100, 70, 54, 12));
        lb_angle_yz->setAlignment(Qt::AlignCenter);
        lb_angle_zx = new QLabel(groupBox);
        lb_angle_zx->setObjectName(QString::fromUtf8("lb_angle_zx"));
        lb_angle_zx->setGeometry(QRect(100, 100, 54, 12));
        lb_angle_zx->setAlignment(Qt::AlignCenter);
        btn_get = new QPushButton(groupBox);
        btn_get->setObjectName(QString::fromUtf8("btn_get"));
        btn_get->setEnabled(false);
        btn_get->setGeometry(QRect(340, 100, 75, 23));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 10, 54, 12));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 502, 23));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        ledt_ip->setText(QApplication::translate("MainWindow", "192.168.1.111", 0, QApplication::UnicodeUTF8));
        btn_ipok->setText(QApplication::translate("MainWindow", "\347\241\256\345\256\232", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "\345\247\277\346\200\201\350\247\222\345\272\246\357\274\232", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "xoy\345\271\263\351\235\242\357\274\232", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "yoz\345\271\263\351\235\242\357\274\232", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "zox\345\271\263\351\235\242\357\274\232", 0, QApplication::UnicodeUTF8));
        lb_angle_xy->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lb_angle_yz->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lb_angle_zx->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        btn_get->setText(QApplication::translate("MainWindow", "\350\216\267\345\217\226\350\247\222\345\272\246", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "PI\347\232\204ip\357\274\232", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
