#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QVector>
#include <QPoint>
#include <QtMath>
#include <QDebug>

QVector<QPoint> *pointVector;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->lineEdit_1->setText("120");
    ui->lineEdit_2->setText("120");
    ui->lineEdit_3->setText("180");
    ui->lineEdit_4->setText("120");
    ui->lineEdit_5->setText("180");

    ui->lineEdit_11->setText("100");
    ui->lineEdit_12->setText("2.5");
    ui->lineEdit_13->setText("60");
    ui->lineEdit_14->setText("2.5");

    ui->pushButton->setStyleSheet("QPushButton{font: 20pt 'Times New Roman';}"
                                "QPushButton:pressed{background-color:rgb(125, 125, 125);}");

    workspacePainter = new PaintWidget(nullptr);
    update();

    pointVector = new QVector<QPoint>();
    timer = new QTimer(this);
    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(startCaculate()));
    connect(timer,SIGNAL(timeout()), this,SLOT(inverse()));
}

static float i = 0, j = 0;
static float max_X = 0;
static float max_Y = 0;
void MainWindow::startCaculate()
{
    // load user-setted parameters
    l_b = ui->lineEdit_1->text().toFloat(nullptr);
    l_h1 = ui->lineEdit_2->text().toFloat(nullptr);
    l_h2 = ui->lineEdit_3->text().toFloat(nullptr);
    l_l1 = ui->lineEdit_4->text().toFloat(nullptr);
    l_l2 = ui->lineEdit_5->text().toFloat(nullptr);

    h_h = ui->lineEdit_11->text().toFloat(nullptr);
    d_h = ui->lineEdit_12->text().toFloat(nullptr);
    h_l = ui->lineEdit_13->text().toFloat(nullptr);
    d_l = ui->lineEdit_14->text().toFloat(nullptr);

    max_X = ((l_h1+l_h2)<(l_l1+l_l2)?(l_h1+l_h2):(l_l1+l_l2));
    max_Y = l_b/2+((l_h1+l_h2)>(l_l1+l_l2)?(l_h1+l_h2):(l_l1+l_l2));

    i = -max_X;
    j = -max_Y;

    // update workspace in opengl-widget
    pointVector->clear();

    timer->start(1);
}

void MainWindow::inverse()
{
//    qDebug()<<i<<" "<<j<<endl;
    static float LL = 0, LH = 0;
    static float alpha_l = 0, alpha_h = 0, beta_l = 0, beta_h = 0;

    // get the inverse solution, if exist, then add to painterVector
    LL = sqrt(pow(i,2)+pow(j+l_b/2,2));
    LH = sqrt(pow(i,2)+pow(j-l_b/2,2));
    if(l_l1+l_l2>=LL && abs(l_l1-l_l2)<=LL &&
            l_h1+l_h2>=LH && abs(l_h1-l_h2)<=LH){
        // there exists a solution
        alpha_l = acos((LL*LL+l_l1*l_l1-l_l2*l_l2)/(2*LL*l_l1));
        alpha_h = acos((LH*LH+l_h1*l_h1-l_h2*l_h2)/(2*LH*l_h1));
        beta_l = atan(i/(j+l_b/2));
        beta_h = atan(i/(-j+l_b/2));

        theta_l = alpha_l+beta_l;
        theta_h = alpha_h+beta_h;
        pointVector->push_back(QPoint(i+383,-j+157));
        update();
    }

    // caculate the kinematic parameters of the new robot
    if(j++>max_Y){
        j = -max_Y;
        if(i++>max_X){
            timer->stop();
        }
    }
}


MainWindow::~MainWindow()
{
    delete ui;
    delete workspacePainter;
    delete pointVector;
    delete timer;
}

PaintWidget::PaintWidget(QWidget *parent) : QWidget(parent){}


void PaintWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    QPen pen;
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);

    // draw workspace points
    pen.setWidth(1);
    pen.setBrush(Qt::green);
    painter.setPen(pen);
    for(int i=0;i<pointVector->size();i++){
        painter.drawPoint(pointVector->at(i));
    }

    // draw the base axis-XY and unit
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(8);
    pen.setBrush(Qt::white);
    painter.setPen(pen);
    painter.drawRect(0,0,765,313);
    pen.setStyle(Qt::DashDotLine);
    pen.setWidth(2);
    pen.setBrush(Qt::black);
    painter.setPen(pen);
    painter.drawLine(0,157,765,157);
    painter.drawLine(383,0,383,313);
    painter.drawText(10,30,"unit:mm");
}

PaintWidget::~PaintWidget()
{

}
