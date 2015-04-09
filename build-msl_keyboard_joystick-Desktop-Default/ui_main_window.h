/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QVBoxLayout *verticalLayout_2;
    QLabel *robotIdLabel;
    QLabel *motionLabel;
    QLabel *kickPowerLabel;
    QLabel *directionLabel;
    QLabel *translationLabel;
    QLabel *shovelLabel;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout;
    QLineEdit *robotIdEdit;
    QLabel *motionValue;
    QLabel *kickPowerValue;
    QLabel *rotationValue;
    QLabel *translationValue;
    QLabel *shovelValue;
    QVBoxLayout *verticalLayout_5;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_8;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(369, 300);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        robotIdLabel = new QLabel(centralwidget);
        robotIdLabel->setObjectName(QString::fromUtf8("robotIdLabel"));

        verticalLayout_2->addWidget(robotIdLabel);

        motionLabel = new QLabel(centralwidget);
        motionLabel->setObjectName(QString::fromUtf8("motionLabel"));

        verticalLayout_2->addWidget(motionLabel);

        kickPowerLabel = new QLabel(centralwidget);
        kickPowerLabel->setObjectName(QString::fromUtf8("kickPowerLabel"));

        verticalLayout_2->addWidget(kickPowerLabel);

        directionLabel = new QLabel(centralwidget);
        directionLabel->setObjectName(QString::fromUtf8("directionLabel"));

        verticalLayout_2->addWidget(directionLabel);

        translationLabel = new QLabel(centralwidget);
        translationLabel->setObjectName(QString::fromUtf8("translationLabel"));

        verticalLayout_2->addWidget(translationLabel);

        shovelLabel = new QLabel(centralwidget);
        shovelLabel->setObjectName(QString::fromUtf8("shovelLabel"));

        verticalLayout_2->addWidget(shovelLabel);


        hboxLayout->addLayout(verticalLayout_2);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(-1, 10, -1, 0);
        robotIdEdit = new QLineEdit(centralwidget);
        robotIdEdit->setObjectName(QString::fromUtf8("robotIdEdit"));

        verticalLayout->addWidget(robotIdEdit);


        verticalLayout_4->addLayout(verticalLayout);

        motionValue = new QLabel(centralwidget);
        motionValue->setObjectName(QString::fromUtf8("motionValue"));

        verticalLayout_4->addWidget(motionValue);

        kickPowerValue = new QLabel(centralwidget);
        kickPowerValue->setObjectName(QString::fromUtf8("kickPowerValue"));

        verticalLayout_4->addWidget(kickPowerValue);

        rotationValue = new QLabel(centralwidget);
        rotationValue->setObjectName(QString::fromUtf8("rotationValue"));

        verticalLayout_4->addWidget(rotationValue);

        translationValue = new QLabel(centralwidget);
        translationValue->setObjectName(QString::fromUtf8("translationValue"));

        verticalLayout_4->addWidget(translationValue);

        shovelValue = new QLabel(centralwidget);
        shovelValue->setObjectName(QString::fromUtf8("shovelValue"));

        verticalLayout_4->addWidget(shovelValue);


        hboxLayout->addLayout(verticalLayout_4);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_5->addWidget(label);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_5->addWidget(label_2);

        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_5->addWidget(label_3);

        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_5->addWidget(label_4);

        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_5->addWidget(label_5);

        label_8 = new QLabel(centralwidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_5->addWidget(label_8);


        hboxLayout->addLayout(verticalLayout_5);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 369, 25));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        robotIdLabel->setText(QApplication::translate("MainWindowDesign", "RobotId:", 0, QApplication::UnicodeUTF8));
        motionLabel->setText(QApplication::translate("MainWindowDesign", "Motion:", 0, QApplication::UnicodeUTF8));
        kickPowerLabel->setText(QApplication::translate("MainWindowDesign", "KickPower:", 0, QApplication::UnicodeUTF8));
        directionLabel->setText(QApplication::translate("MainWindowDesign", "RotationSpeed:", 0, QApplication::UnicodeUTF8));
        translationLabel->setText(QApplication::translate("MainWindowDesign", "Translation:", 0, QApplication::UnicodeUTF8));
        shovelLabel->setText(QApplication::translate("MainWindowDesign", "Shovel:", 0, QApplication::UnicodeUTF8));
        robotIdEdit->setInputMask(QString());
        motionValue->setText(QString());
        kickPowerValue->setText(QString());
        rotationValue->setText(QString());
        translationValue->setText(QString());
        shovelValue->setText(QString());
        label->setText(QString());
        label_2->setText(QString());
        label_3->setText(QApplication::translate("MainWindowDesign", "W -> + | S -> -", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindowDesign", "Q -> + | A -> -", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindowDesign", "R -> + | F = -> -", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindowDesign", "E", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
