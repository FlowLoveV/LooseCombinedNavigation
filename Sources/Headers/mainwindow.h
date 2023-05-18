//
// Created by 0-0 mashuo on 2023/5/9.
//

#ifndef COMBINEDNAVIGATION_MAINWINDOW_H
#define COMBINEDNAVIGATION_MAINWINDOW_H

#include <QMainWindow>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

private:
    Ui::MainWindow *ui;
};


#endif //COMBINEDNAVIGATION_MAINWINDOW_H
