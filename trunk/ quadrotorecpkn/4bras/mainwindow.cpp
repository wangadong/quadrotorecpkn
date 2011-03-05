#include "mainwindow.h"

#include <QMenubar>

MainWindow::MainWindow()
//    : QMainWindow(parent)
{
    setWindowTitle(tr("Console for quadrocopter"));
    workSpace = new QWorkspace(this);
    setCentralWidget(workSpace);

    fabriqueDockwin1();
    fabriqueDockwin2();
    fabriqueMenus();

    QObject::connect(mycom->thread,SIGNAL(readsignal()),Module3d,SLOT(upselondataSlot()));
    QObject::connect(mycom->thread,SIGNAL(setCRotation(float)),Module3d,SLOT(setCRotation(float)));
}

MainWindow::~MainWindow()
{

}

void MainWindow::fabriqueDockwin1()
{
    Dockwin1=new QDockWidget(tr("3d view"),this);
    Dockwin1->setFeatures(QDockWidget::AllDockWidgetFeatures);
    Module3d = new Module3D;
    Dockwin1->setWidget( Module3d );
    addDockWidget(Qt::LeftDockWidgetArea,Dockwin1);
}

void MainWindow::fabriqueDockwin2()
{
    Dockwin2=new QDockWidget(tr("Mycom"),this);
    Dockwin2->setFeatures(QDockWidget::AllDockWidgetFeatures);
    mycom=new Mycom();
    Dockwin2->setWidget(mycom);
    addDockWidget(Qt::RightDockWidgetArea,Dockwin2);
}

void MainWindow::fabriqueMenus()
{
    fileMenu = menuBar()->addMenu(tr("File"));
    toolMenu = menuBar()->addMenu(tr("Tools"));
    layoutMenu = menuBar()->addMenu(tr("WindowsArrange"));
    aboutMenu = menuBar()->addMenu(tr("About"));

//    fileMenu->addAction(openAction);
//    fileMenu->addAction(exitAction);

//    toolMenu->addAction(dviewerAction);
//    toolMenu->addAction(serialAction);
//    toolMenu->addAction(mapviewAction);
//    toolMenu->addAction(compassAction);
//    toolMenu->addAction(videoAction);

//    aboutMenu->addAction(aboutAction);

//    layoutMenu->addAction(arrange);
//    layoutMenu->addAction(tile);
//    layoutMenu->addAction(cascade);
}

//void MainWindow::slotserial()
//{
//    Dockwin1->show();
//}
