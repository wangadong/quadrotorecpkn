#include"Module3D.h"
#include <QtOpenGL>
#include <math.h>
Module3D::Module3D(QWidget *parent)
    :QGLWidget(parent)
{
    setFormat(QGL::DoubleBuffer|QGL::DepthBuffer);
    cRot=0;
    pRot=0;
    rRot=0;
    vCdirection=vPdirection=vRdirection=0;
    updateTimer=new QTimer(this);
    connect(updateTimer,SIGNAL(timeout()),this,
                SLOT(upDateAttitude()));
    connect(updateTimer,SIGNAL(timeout()),this,
                SLOT(indiquecVitesse()));
    updateTimer->start(10);
}
//----------------------------------------------------------------------------
Module3D::~Module3D()
{

}
//----------------------------------------------------------------------------
void Module3D::initializeGL()
{
    qglClearColor(Qt::black);
    glEnable(GL_CULL_FACE);;
    GLfloat mat_specular[4]={1.0,1.0,1.0,1.0};
    GLfloat mat_shininess[4]={50.0};
    GLfloat lightAmbient[4] = { 0.5, 0.5, 1.0, 1.0 };
    GLfloat lightDiffuse[4] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lightPosition[4] = { 1.0, 0.0, 0.0, 0.0 };
    GLfloat lmodel_ambient[4]={0.1,0.1,0.1,1.0};
    glShadeModel( GL_SMOOTH );
    glClearColor( 0.0, 0.0, 0.0, 0.5);
    glClearDepth( 1.0 );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glMaterialfv(GL_FRONT,GL_SPECULAR,mat_specular);
    glMaterialfv(GL_FRONT,GL_SHININESS,mat_shininess);
    glLightfv( GL_LIGHT1, GL_AMBIENT, lightAmbient );
    glLightfv( GL_LIGHT1, GL_DIFFUSE, lightDiffuse );
    glLightfv( GL_LIGHT1, GL_POSITION, lightPosition );
    glLightf(GL_LIGHT1,GL_CONSTANT_ATTENUATION,2.0);
    glLightf(GL_LIGHT1,GL_LINEAR_ATTENUATION,1.0);
    glLightf(GL_LIGHT1,GL_QUADRATIC_ATTENUATION,2.0);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lmodel_ambient);

//    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT1);
}
//------------------------------------------------------------------------------
void Module3D::resizeGL(int width, int height)
{
    glViewport( 0, 0, (GLint)width, (GLint)height ); //20100125
    glMatrixMode( GL_PROJECTION );//
    glLoadIdentity();//
    gluPerspective( 30.0, (GLfloat)width/(GLfloat)height, 0.1, 100.0 );//
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

//    glViewport(0,0,width,height);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    GLfloat x=GLfloat(width)/height;
//    glFrustum(-x,+x,-10.0,+10.0,4.0,15.0);//该函数用于设置投影视口。
//    glMatrixMode(GL_MODELVIEW);//设定哪一个矩阵是当前矩阵，GL_MODEVIEW表示对模型视镜矩阵堆栈应用随后矩阵操作
}
//-------------------------------------------------------------------------------
void Module3D::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();//重置当前指定的矩阵为单位矩阵
    glTranslatef(0.0,0.0,-65.0);
    glRotatef(45.0,1,1,0);
    glRotatef(rRot,0,0,1);
    glRotatef(pRot,1,0,0);
    glRotatef(cRot,0,0,1);
    dessinerAxe();
    dessinerCorp();//螺旋桨为主题的子节点，主体动螺旋桨必动，但螺旋桨动，主体不一定动
    dessinerVitesse();


}
//---------------------------------------------------------------------------------
void Module3D::timerEvent(QTimerEvent*)
{
  updateGL();
}
//----------------------------------------------------------------------------------
void Module3D::dessinerCorp()
{
    glBegin(GL_QUADS);
    //z轴方向的支撑杆
        glColor3f(0.7,0.7,0.85);
//        右面
        glVertex3f(0.5,0.5,-10.0);
        glVertex3f(0.5,0.5,10.0);
        glVertex3f(0.5,-0.5,10.0);
        glVertex3f(0.5,-0.5,-10.0);

//        下面
        glColor3f(0.85,0.2,0.85);
        glVertex3f(0.5,-0.5,-10.0);
        glVertex3f(0.5,-0.5,10.0);
        glVertex3f(-0.5,-0.5,10.0);
        glVertex3f(-0.5,-0.5,-10.0);
//        左面
        glColor3f(0.7,0.7,0.85);
        glVertex3f(-0.5,-0.5,-10.0);
        glVertex3f(-0.5,-0.5,10.0);
        glVertex3f(-0.5,0.5,10.0);
        glVertex3f(-0.5,0.5,-10.0);
//        上面
        glColor3f(0.85,0.2,0.85);
        glVertex3f(-0.5,0.5,-10.0);
        glVertex3f(-0.5,0.5,10.0);
        glVertex3f(0.5,0.5,10.0);
        glVertex3f(0.5,0.5,-10.0);
//        包装
        glColor3f(0.85,0.85,0.1);

        glVertex3f(0.5,0.5,10.0);
        glVertex3f(-0.5,0.5,10.0);
        glVertex3f(-0.5,-0.5,10.0);
        glVertex3f(0.5,-0.5,10.0);

        glVertex3f(-0.5,0.5,-10.0);
        glVertex3f(0.5,0.5,-10.0);
        glVertex3f(0.5,-0.5,-10.0);
        glVertex3f(-0.5,-0.5,-10.0);
    glEnd();
//x轴支撑杆的方向
    glBegin(GL_QUADS);
        glColor3f(0.7,0.7,0.85);
//        后面
        glVertex3f(-10.0,0.5,-0.5);
        glVertex3f(10.0,0.5,-0.5);
        glVertex3f(10.0,-0.5,-0.5);
        glVertex3f(-10.0,-0.5,-0.5);
//        下面
        glColor3f(0.85,0.2,0.85);
        glVertex3f(-10.0,-0.5,-0.5);
        glVertex3f(10.0,-0.5,-0.5);
        glVertex3f(10.0,-0.5,0.5);
        glVertex3f(-10.0,-0.5,0.5);
//        前面
        glColor3f(0.7,0.7,0.85);
        glVertex3f(-10.0,-0.5,0.5);
        glVertex3f(10.0,-0.5,0.5);
        glVertex3f(10.0,0.5,0.5);
        glVertex3f(-10.0,0.5,0.5);
//        上面
        glColor3f(0.85,0.2,0.85);
        glVertex3f(-10.0,0.5,0.5);
        glVertex3f(10.0,0.5,0.5);
        glVertex3f(10.0,0.5,-0.5);
        glVertex3f(-10.0,0.5,-0.5);
 //        包装
        glColor3f(0.85,0.85,0.1);

        glVertex3f(10.0,0.5,-0.5);
        glVertex3f(10.0,0.5,0.5);
        glVertex3f(10.0,-0.5,0.5);
        glVertex3f(10.0,-0.5,-0.5);

        glVertex3f(-10.0,0.5,0.5);
        glVertex3f(-10.0,0.5,-0.5);
        glVertex3f(-10.0,-0.5,-0.5);
        glVertex3f(-10.0,-0.5,0.5);
   glEnd();
//   glutSolidSphere(5.0,20,20);
//   添加旋翼
   glPushMatrix();
   glTranslatef(0.0,0.0,10.0);
   dessinerHeliceface();
   dessinerHelicecote();
   glPopMatrix();
   glPushMatrix();
   glTranslatef(0.0,0.0,-10.0);
   dessinerHeliceface();
   dessinerHelicecote();
   glPopMatrix();
   glPushMatrix();
   glTranslatef(10.0,0.0,0.0);
   dessinerHeliceface();
   dessinerHelicecote();
   glPopMatrix();
   glPushMatrix();
   glTranslatef(-10.0,0.0,0.0);
   dessinerHeliceface();
   dessinerHelicecote();
   glPopMatrix();
}
//-----------------------------------------------------------------------------------------
void Module3D::dessinerHeliceface()
{
//    下表面
    glBegin(GL_TRIANGLES);
         glColor3f(0.2,0.2,0.2);
         glVertex3f(0.0,0.5,0.0);
         glVertex3f(0.0,0.5,-2.0);
         glVertex3f(1.5,0.5,-2.0);
         glVertex3f(0.0,0.5,0.0);
         glVertex3f(2.0,0.5,0.0);
         glVertex3f(2.0,0.5,1.5);
         glVertex3f(0.0,0.5,0.0);
         glVertex3f(0.0,0.5,2.0);
         glVertex3f(-1.5,0.5,2.0);
         glVertex3f(0.0,0.5,0.0);
         glVertex3f(-2.0,0.5,0.0);
         glVertex3f(-2.0,0.5,-1.5);
    glEnd();
//    上表面
    glBegin(GL_TRIANGLES);
         glColor3f(1.0,1.0,1.0);
         glVertex3f(0.0,0.8,0.0);
         glVertex3f(1.5,0.8,-2.0);
         glVertex3f(0.0,0.8,-2.0);
         glVertex3f(0.0,0.8,0.0);
         glVertex3f(2.0,0.8,1.5);
         glVertex3f(2.0,0.8,0.0);
         glVertex3f(0.0,0.8,0.0);
         glVertex3f(-1.5,0.8,2.0);
         glVertex3f(0.0,0.8,2.0);
         glVertex3f(0.0,0.8,0.0);
         glVertex3f(-2.0,0.8,-1.5);
         glVertex3f(-2.0,0.8,-0.0);

    glEnd();
}
//-------------------------------------------------------------------------------------------
void Module3D::dessinerVitesse()
{
    glBegin(GL_LINES);
        glColor3f(0.5,0.9,0.5);
        GLfloat r=sqrt(vCdirection*vCdirection+vPdirection*vPdirection+vRdirection*vRdirection);
        glVertex3f(-5.0,5.0,5.0);
        glVertex3f(-5.0+vCdirection/r,5.0+vPdirection/r,5.0+vRdirection/r);
//        glVertex3f(-5.0+vCdirection/r,5.0+vPdirection/r,5.0+vRdirection/r);
//        glVertex3f(-5.0+vCdirection/r-1.5,5.0+vPdirection/r+1,5.0+vRdirection/r);
//        glVertex3f(-5.0+vCdirection/r,5.0+vPdirection/r,5.0+vRdirection/r);
//        glVertex3f(-5.0+vCdirection/r,5.0+vPdirection/r,5.0+vRdirection/r);
     glEnd();

     glBegin(GL_POINTS);
     glColor3f(1.0,0.0,0.0);
     glVertex3f(-5.0+vCdirection/r,5.0+vPdirection/r,5.0+vRdirection/r);
     glEnd();
}
//----------------------------------------------------------------------------------------------
void Module3D::dessinerHelicecote()
{
    glBegin(GL_QUAD_STRIP);
            glColor3f(0.2,0.8,0.8);
            glVertex3f(0.0,0.5,0.0);
            glVertex3f(0.0,0.8,0.0);
            glVertex3f(0.0,0.5,-2.0);
            glVertex3f(0.0,0.8,-2.0);
            glVertex3f(1.5,0.5,-2.0);
            glVertex3f(1.5,0.8,-2.0);
            glVertex3f(0.0,0.5,0.0);
            glVertex3f(0.0,0.8,0.0);
     glEnd();
     glBegin(GL_QUAD_STRIP);
             glColor3f(0.2,0.8,0.8);
             glVertex3f(0.0,0.5,0.0);
             glVertex3f(0.0,0.8,0.0);
             glVertex3f(2.0,0.5,0.0);
             glVertex3f(2.0,0.8,0.0);
             glVertex3f(2.0,0.5,1.5);
             glVertex3f(2.0,0.8,1.5);
             glVertex3f(0.0,0.5,0.0);
             glVertex3f(0.0,0.8,0.0);
      glEnd();
      glBegin(GL_QUAD_STRIP);
              glColor3f(0.2,0.8,0.8);
              glVertex3f(0.0,0.5,0.0);
              glVertex3f(0.0,0.8,0.0);
              glVertex3f(0.0,0.5,2.0);
              glVertex3f(0.0,0.8,2.0);
              glVertex3f(-1.5,0.5,2.0);
              glVertex3f(-1.5,0.8,2.0);
              glVertex3f(0.0,0.5,0.0);
              glVertex3f(0.0,0.8,0.0);
       glEnd();
       glBegin(GL_QUAD_STRIP);
               glColor3f(0.2,0.8,0.8);
               glVertex3f(0.0,0.5,0.0);
               glVertex3f(0.0,0.8,0.0);
               glVertex3f(-2.0,0.5,0.0);
               glVertex3f(-2.0,0.8,0.0);
               glVertex3f(-2.0,0.5,-1.5);
               glVertex3f(-2.0,0.8,-1.5);
               glVertex3f(0.0,0.5,0.0);
               glVertex3f(0.0,0.8,0.0);
        glEnd();
}
//------------------------------------------------------------------------------------------------
void Module3D::dessinerAxe()
{
    glLineStipple(1,0x3F07);
    glEnable(GL_LINE_STIPPLE);
//    x轴
    glBegin(GL_LINES);
        glColor3f(1.0,0.0,0.0);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(25.0,0.0,0.0);
    glEnd();
//    y轴
    glBegin(GL_LINES);
        glColor3f(0.0,1.0,0.0);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(0.0,25.0,0.0);
    glEnd();
//     z轴
    glBegin(GL_LINES);
        glColor3f(0.0,0.0,1.0);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(0.0,0.0,25.0);
    glEnd();
}
//---------------------------------------------------------------------------
void Module3D::setCRotation(float angle)
{
    normalizeAngle(&angle);
    if (angle != cRot) {
        cRot = angle;
        updateGL();
    }
}
//-----------------------------------------------------------------------------
void Module3D::setPRotation(float angle)
{
    normalizeAngle(&angle);
    if (angle != pRot) {
        pRot = angle;
        updateGL();
    }
}
//-------------------------------------------------------------------------------
 void Module3D::setRRotation(float angle)
    {
        normalizeAngle(&angle);
        if (angle != rRot) {
            rRot = angle;
            updateGL();
        }
    }
//--------------------------------------------------------------------------------
 void Module3D::setCVitesse(float v)
 {
     if (v != vCdirection) {
         vCdirection = v;
         updateGL();
     }
 }
//---------------------------------------------------------------------------------
 void Module3D::setPVitesse(float v)
 {
     if (v != vPdirection) {
         vPdirection = v;
         updateGL();
     }
 }
//----------------------------------------------------------------------------------
 void Module3D::setRVitesse(float v)
 {
     if (v != vRdirection) {
         vRdirection = v;
         updateGL();
     }
 }
//-----------------------------------------------------------------------------------
 void Module3D::normalizeAngle(float* angle)
 {
     while (*angle < 0)
         *angle += 360;
     while (*angle > 360)
         *angle -= 360;
 }
//-------------------------------------------------------------------------------------
 void Module3D::upDateAttitude()
 {
     updateGL();
 }
//---------------------------------------------------------------------------------------
 void Module3D::upselondataSlot()
 {
 updateTimer->stop();
 }
 //--------------------------------------------------------------------------------------
 QSize Module3D::sizeHint() const
 {
     QSize size = QWidget::sizeHint();
     size.rheight() = 800;
     size.rwidth() = 600;
     return size;
 }
