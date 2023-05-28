//
// Created by 0-0 mashuo on 2023/5/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include <QtGui/qpainter.h>
#include "mainwindow.h"
#include "UI/ui_MainWindow.h"



MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

MainWindow::~MainWindow() {
    delete ui;
}

