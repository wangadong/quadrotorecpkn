#ifndef Module3D_H
#define Module3D_H

#include<QtOpenGL>
#include<QObject>
#include<QGLWidget>
#include <QImage>
#include <QTimer>

class Module3D : public QGLWidget
{
   Q_OBJECT
public:
    Module3D(QWidget * parent=0);
    ~Module3D();
    QSize sizeHint() const;

public slots:
    void setCRotation(float angle);
    void setPRotation(float angle);
    void setRRotation(float angle);
    void upDateAttitude();

    void setCVitesse(float v);
    void setPVitesse(float v);
    void setRVitesse(float v);
    void upselondataSlot();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL( int width, int height );
    void dessinerCorp();
    void dessinerHeliceface();
    void dessinerHelicecote();
    void dessinerVitesse();
    void dessinerAxe();
    void normalizeAngle(float* angle);
    void timerEvent(QTimerEvent*);
protected:
    GLfloat cRot,pRot,rRot;
    GLfloat vCdirection,vPdirection,vRdirection;
    QTimer *updateTimer;
};

#endif // Module3D_H
