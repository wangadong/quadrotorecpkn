#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include<QWorkspace>
#include<QObject>
#include<QDockWidget>
#include "Mycom\Mycom.h"
#include "Mycom\MachineThread.h"
#include"3D/Module3D.h"

class QAction;
class QMenu;
class QToolBar;
class QTextEdit;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    //�����ڹ��죬����
    MainWindow();
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
    Module3D * Module3d;
    Mycom * mycom;

    QMenu *fileMenu;
    QMenu *toolMenu;
    QMenu *aboutMenu;
    QMenu *layoutMenu;
};

#endif // MAINWINDOW_H
