#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include<QWorkspace>
#include<QDockWidget>
#include "Mycom\Mycom.h"

class QAction;
class QMenu;
class QToolBar;
class QTextEdit;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    //�����ڹ��죬����
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    //���촮���Ӵ���
    void fabriqueDockwin1();
    //����3d��ʾ�Ӵ���
    void fabriqueDockwin2();
//    void fabriqueDockwin3();
//    void fabriqueDockwin4();
    //����˵�
    void fabriqueMenus();
private:
    QWorkspace * workSpace;
    QDockWidget * Dockwin1;
    QDockWidget * Dockwin2;
    QMenu * Menu;
    QWidget * Drone3d;
    Mycom * mycom;

    QMenu *fileMenu;
    QMenu *toolMenu;
    QMenu *aboutMenu;
    QMenu *layoutMenu;

//    QAction *exitAction;
//    QAction *openAction;
//    QAction *aboutAction;
//    QAction *arrange;
//    QAction *tile;
//    QAction *cascade;

//    QAction *dviewerAction;
//    QAction *serialAction;
//    QAction *mapviewAction;
//    QAction *compassAction;
//    QAction *videoAction;
};

#endif // MAINWINDOW_H