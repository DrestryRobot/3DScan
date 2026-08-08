/********************************************************************************
** Form generated from reading UI file 'mainwindow7.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW7_H
#define UI_MAINWINDOW7_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include "qvtkopenglnativewidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow7
{
public:
    QWidget *centralwidget;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QPushButton *pushButton_8;
    QPushButton *pushButton_11;
    QPushButton *pushButton_4;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_4;
    QLabel *label_3;
    QPushButton *pushButton_10;
    QPushButton *pushButton_5;
    QPushButton *pushButton;
    QPushButton *pushButton_9;
    QPushButton *pushButton_6;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_7;
    QWidget *colorBarWidget;
    QVTKOpenGLNativeWidget *vtkWidget;
    QLineEdit *lineEdit;

    void setupUi(QMainWindow *MainWindow7)
    {
        if (MainWindow7->objectName().isEmpty())
            MainWindow7->setObjectName("MainWindow7");
        MainWindow7->resize(803, 627);
        QSizePolicy sizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow7->sizePolicy().hasHeightForWidth());
        MainWindow7->setSizePolicy(sizePolicy);
        centralwidget = new QWidget(MainWindow7);
        centralwidget->setObjectName("centralwidget");
        sizePolicy.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy);
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName("layoutWidget");
        layoutWidget->setGeometry(QRect(0, 0, 801, 631));
        gridLayout_2 = new QGridLayout(layoutWidget);
        gridLayout_2->setObjectName("gridLayout_2");
        gridLayout_2->setSizeConstraint(QLayout::SizeConstraint::SetDefaultConstraint);
        gridLayout_2->setHorizontalSpacing(0);
        gridLayout_2->setVerticalSpacing(3);
        gridLayout_2->setContentsMargins(10, 10, 10, 10);
        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        pushButton_8 = new QPushButton(layoutWidget);
        pushButton_8->setObjectName("pushButton_8");
        sizePolicy.setHeightForWidth(pushButton_8->sizePolicy().hasHeightForWidth());
        pushButton_8->setSizePolicy(sizePolicy);
        QFont font;
        font.setPointSize(14);
        pushButton_8->setFont(font);

        gridLayout->addWidget(pushButton_8, 10, 0, 1, 2);

        pushButton_11 = new QPushButton(layoutWidget);
        pushButton_11->setObjectName("pushButton_11");
        sizePolicy.setHeightForWidth(pushButton_11->sizePolicy().hasHeightForWidth());
        pushButton_11->setSizePolicy(sizePolicy);
        pushButton_11->setFont(font);

        gridLayout->addWidget(pushButton_11, 14, 0, 1, 2);

        pushButton_4 = new QPushButton(layoutWidget);
        pushButton_4->setObjectName("pushButton_4");
        sizePolicy.setHeightForWidth(pushButton_4->sizePolicy().hasHeightForWidth());
        pushButton_4->setSizePolicy(sizePolicy);
        pushButton_4->setFont(font);

        gridLayout->addWidget(pushButton_4, 8, 0, 1, 2);

        label = new QLabel(layoutWidget);
        label->setObjectName("label");
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        QFont font1;
        font1.setPointSize(14);
        font1.setBold(true);
        label->setFont(font1);
        label->setStyleSheet(QString::fromUtf8("border-radius: 3px;\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(0, 138, 200);"));
        label->setAlignment(Qt::AlignmentFlag::AlignCenter);

        gridLayout->addWidget(label, 0, 0, 1, 2);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName("label_2");
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setFont(font1);
        label_2->setStyleSheet(QString::fromUtf8("border-radius: 3px;\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(0, 138, 200);"));
        label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);

        gridLayout->addWidget(label_2, 5, 0, 1, 2);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName("label_4");
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);
        label_4->setFont(font1);
        label_4->setStyleSheet(QString::fromUtf8("border-radius: 3px;\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(0, 138, 200);"));
        label_4->setAlignment(Qt::AlignmentFlag::AlignCenter);

        gridLayout->addWidget(label_4, 12, 0, 1, 2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName("label_3");
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setFont(font1);
        label_3->setStyleSheet(QString::fromUtf8("border-radius: 3px;\n"
"color: rgb(255, 255, 255);\n"
"background-color: rgb(0, 138, 200);"));
        label_3->setAlignment(Qt::AlignmentFlag::AlignCenter);

        gridLayout->addWidget(label_3, 9, 0, 1, 2);

        pushButton_10 = new QPushButton(layoutWidget);
        pushButton_10->setObjectName("pushButton_10");
        sizePolicy.setHeightForWidth(pushButton_10->sizePolicy().hasHeightForWidth());
        pushButton_10->setSizePolicy(sizePolicy);
        pushButton_10->setFont(font);

        gridLayout->addWidget(pushButton_10, 15, 0, 1, 2);

        pushButton_5 = new QPushButton(layoutWidget);
        pushButton_5->setObjectName("pushButton_5");
        sizePolicy.setHeightForWidth(pushButton_5->sizePolicy().hasHeightForWidth());
        pushButton_5->setSizePolicy(sizePolicy);
        pushButton_5->setFont(font);

        gridLayout->addWidget(pushButton_5, 6, 0, 1, 2);

        pushButton = new QPushButton(layoutWidget);
        pushButton->setObjectName("pushButton");
        sizePolicy.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
        pushButton->setSizePolicy(sizePolicy);
        pushButton->setFont(font);

        gridLayout->addWidget(pushButton, 2, 0, 1, 2);

        pushButton_9 = new QPushButton(layoutWidget);
        pushButton_9->setObjectName("pushButton_9");
        sizePolicy.setHeightForWidth(pushButton_9->sizePolicy().hasHeightForWidth());
        pushButton_9->setSizePolicy(sizePolicy);
        pushButton_9->setFont(font);

        gridLayout->addWidget(pushButton_9, 13, 0, 1, 2);

        pushButton_6 = new QPushButton(layoutWidget);
        pushButton_6->setObjectName("pushButton_6");
        sizePolicy.setHeightForWidth(pushButton_6->sizePolicy().hasHeightForWidth());
        pushButton_6->setSizePolicy(sizePolicy);
        pushButton_6->setFont(font);

        gridLayout->addWidget(pushButton_6, 7, 0, 1, 2);

        pushButton_2 = new QPushButton(layoutWidget);
        pushButton_2->setObjectName("pushButton_2");
        sizePolicy.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy);
        pushButton_2->setFont(font);

        gridLayout->addWidget(pushButton_2, 3, 0, 1, 2);

        pushButton_3 = new QPushButton(layoutWidget);
        pushButton_3->setObjectName("pushButton_3");
        sizePolicy.setHeightForWidth(pushButton_3->sizePolicy().hasHeightForWidth());
        pushButton_3->setSizePolicy(sizePolicy);
        pushButton_3->setFont(font);

        gridLayout->addWidget(pushButton_3, 4, 0, 1, 2);

        pushButton_7 = new QPushButton(layoutWidget);
        pushButton_7->setObjectName("pushButton_7");
        sizePolicy.setHeightForWidth(pushButton_7->sizePolicy().hasHeightForWidth());
        pushButton_7->setSizePolicy(sizePolicy);
        pushButton_7->setFont(font);

        gridLayout->addWidget(pushButton_7, 1, 0, 1, 2);


        gridLayout_2->addLayout(gridLayout, 0, 3, 2, 1);

        colorBarWidget = new QWidget(layoutWidget);
        colorBarWidget->setObjectName("colorBarWidget");

        gridLayout_2->addWidget(colorBarWidget, 0, 0, 1, 2);

        vtkWidget = new QVTKOpenGLNativeWidget(layoutWidget);
        vtkWidget->setObjectName("vtkWidget");
        sizePolicy.setHeightForWidth(vtkWidget->sizePolicy().hasHeightForWidth());
        vtkWidget->setSizePolicy(sizePolicy);
        vtkWidget->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));

        gridLayout_2->addWidget(vtkWidget, 0, 2, 1, 1);

        lineEdit = new QLineEdit(layoutWidget);
        lineEdit->setObjectName("lineEdit");
        sizePolicy.setHeightForWidth(lineEdit->sizePolicy().hasHeightForWidth());
        lineEdit->setSizePolicy(sizePolicy);
        lineEdit->setMinimumSize(QSize(0, 0));
        QFont font2;
        font2.setPointSize(12);
        lineEdit->setFont(font2);

        gridLayout_2->addWidget(lineEdit, 1, 0, 1, 3);

        gridLayout_2->setRowStretch(0, 26);
        gridLayout_2->setRowStretch(1, 2);
        gridLayout_2->setColumnStretch(1, 1);
        gridLayout_2->setColumnStretch(2, 30);
        gridLayout_2->setColumnStretch(3, 6);
        MainWindow7->setCentralWidget(centralwidget);

        retranslateUi(MainWindow7);

        QMetaObject::connectSlotsByName(MainWindow7);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow7)
    {
        MainWindow7->setWindowTitle(QCoreApplication::translate("MainWindow7", "MainWindow", nullptr));
        pushButton_8->setText(QCoreApplication::translate("MainWindow7", "AMP\346\250\241\345\274\217", nullptr));
        pushButton_11->setText(QCoreApplication::translate("MainWindow7", "\347\252\227\345\217\243\346\210\252\345\233\276", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow7", "\351\207\215\347\275\256\346\225\260\346\215\256", nullptr));
        label->setText(QCoreApplication::translate("MainWindow7", "\346\211\253\346\237\245\346\210\220\345\203\217", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow7", "\346\225\260\346\215\256\345\244\204\347\220\206", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow7", "\350\276\205\345\212\251\345\267\245\345\205\267", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow7", "\345\210\207\346\215\242\346\230\276\347\244\272", nullptr));
        pushButton_10->setText(QCoreApplication::translate("MainWindow7", "\346\225\260\346\215\256\346\265\213\351\207\217", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow7", "\344\277\235\345\255\230\346\225\260\346\215\256", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow7", "\345\274\200\345\247\213\347\273\230\345\210\266", nullptr));
        pushButton_9->setText(QCoreApplication::translate("MainWindow7", "\345\257\271\351\275\220\350\247\206\347\202\271", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow7", "\345\212\240\350\275\275\346\225\260\346\215\256", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow7", "\345\201\234\346\255\242\347\273\230\345\210\266", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow7", "\347\273\223\346\235\237\347\273\230\345\210\266", nullptr));
        pushButton_7->setText(QCoreApplication::translate("MainWindow7", "\345\274\200\345\247\213\346\211\253\346\217\217", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow7: public Ui_MainWindow7 {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW7_H
