#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QVector>
#include <QPoint>
#include <QLine>
#include <QtMath>
#include <QDebug>

QVector<QPoint> *pointVector;
QVector<QLine> *lineVector;
QVector<QPoint> *textPointVector;
QVector<QLine> *linkLineVector;

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

    textPointVector = new QVector<QPoint>();
    pointVector = new QVector<QPoint>();
    linkLineVector = new QVector<QLine>();
    lineVector = new QVector<QLine>();
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
    max_Y = -l_b/2+((l_h1+l_h2)>(l_l1+l_l2)?(l_h1+l_h2):(l_l1+l_l2));

    i = -max_X;
    j = -max_Y;

    // update workspace in opengl-widget
    pointVector->clear();
    lineVector->clear();
    textPointVector->clear();

    if(movieMode && !ui->checkBox_2->isChecked())
        timer->start(0);
    else
        timer->stop();

    while(!movieMode){
        static float LL = 0, LH = 0;
        static float alpha_l = 0, alpha_h = 0, beta_l = 0, beta_h = 0;

        // get the inverse solution, if exist, then add to painterVector
        LL = sqrt(pow(i,2)+pow(j+l_b/2,2));
        LH = sqrt(pow(i,2)+pow(j-l_b/2,2));
        if(l_l1+l_l2>=LL && abs(l_l1-l_l2)<=LL &&
                l_h1+l_h2>=LH && abs(l_h1-l_h2)<=LH){
            // there exists a solution
            if(max_X/383 < max_Y/157){
                pointVector->push_back(QPoint(i/max_Y*157+383,-j/max_Y*157+157));
            }else{
                pointVector->push_back(QPoint(i/max_X*383+383,-j/max_X*383+157));
            }
            update();
        }

        // caculate the kinematic parameters of the new robot
        if(j++>max_Y){
            j = -max_Y;
            if(i++>max_X){
                break;
            }
        }
    }
    if(max_X/383 < max_Y/157){
        lineVector->push_back(QLine(QPoint(0+383,-l_b/2/max_Y*157+157),
                                    QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157),
                                    QPoint((sqrt(2)*0.5*(l_h1+l_h2)-8)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2)+sqrt(3)*8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157),
                                    QPoint((sqrt(2)*0.5*(l_h1+l_h2)-sqrt(3)*8)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2)+8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157),
                                    QPoint((sqrt(2)*0.5*(l_h1+l_h2)+90)/max_Y*157+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157)));
        textPointVector->push_back(QPoint((sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+383,
                                          -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157));


        lineVector->push_back(QLine(QPoint(0+383,l_b/2/max_Y*157+157),
                                    QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157),
                                    QPoint((-sqrt(2)*0.5*(l_l1+l_l2)+8)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2)-sqrt(3)*8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157),
                                    QPoint((-sqrt(2)*0.5*(l_l1+l_l2)+sqrt(3)*8)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2)-8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157),
                                    QPoint((-sqrt(2)*0.5*(l_l1+l_l2)-90)/max_Y*157+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157)));
        textPointVector->push_back(QPoint((-sqrt(2)*0.5*(l_l1+l_l2)-90)/max_Y*157+383,
                                          -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157));


        lineVector->push_back(QLine(QPoint(0+383,-l_b/2/max_Y*157+157),
                                    QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157),
                                    QPoint((sqrt(2)*0.5*abs(l_h1-l_h2)-8)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2)-sqrt(3)*8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157),
                                    QPoint((sqrt(2)*0.5*abs(l_h1-l_h2)-sqrt(3)*8)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2)-8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157),
                                    QPoint((sqrt(2)*0.5*abs(l_h1-l_h2)+90)/max_Y*157+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157)));
        textPointVector->push_back(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_Y*157+383,
                                          -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157));


        lineVector->push_back(QLine(QPoint(0+383,l_b/2/max_Y*157+157),
                                    QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157),
                                    QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)+sqrt(3)*8)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2)+8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157),
                                    QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)+8)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2)+sqrt(3)*8)/max_Y*157+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157),
                                    QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)-90)/max_Y*157+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157)));
        textPointVector->push_back(QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)-90)/max_Y*157+383,
                                   -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157));

    }else{
        lineVector->push_back(QLine(QPoint(0+383,-l_b/2/max_X*383+157),
                                    QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_X*383+157),
                                    QPoint((sqrt(2)*0.5*(l_h1+l_h2)-8)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2)+sqrt(3)*8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_X*383+157),
                                    QPoint((sqrt(2)*0.5*(l_h1+l_h2)-sqrt(3)*8)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2)+8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*(l_h1+l_h2)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_X*383+157),
                                    QPoint((sqrt(2)*0.5*(l_h1+l_h2)+90)/max_X*383+383,
                                           -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_X*383+157)));
        textPointVector->push_back(QPoint((sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+383,
                                          -(l_b/2-sqrt(2)*0.5*(l_h1+l_h2))/max_Y*157+157));

        lineVector->push_back(QLine(QPoint(0+383,l_b/2/max_X*383+157),
                                    QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_X*383+157),
                                    QPoint((-sqrt(2)*0.5*(l_l1+l_l2)+8)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2)-sqrt(3)*8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_X*383+157),
                                    QPoint((-sqrt(2)*0.5*(l_l1+l_l2)+sqrt(3)*8)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2)-8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*(l_l1+l_l2)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_X*383+157),
                                    QPoint((-sqrt(2)*0.5*(l_l1+l_l2)-90)/max_X*383+383,
                                           -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_X*383+157)));
        textPointVector->push_back(QPoint((-sqrt(2)*0.5*(l_l1+l_l2)-90)/max_Y*157+383,
                                          -(-l_b/2+sqrt(2)*0.5*(l_l1+l_l2))/max_Y*157+157));

        lineVector->push_back(QLine(QPoint(0+383,-l_b/2/max_X*383+157),
                                    QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_X*383+157),
                                    QPoint((sqrt(2)*0.5*abs(l_h1-l_h2)-8)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2)-sqrt(3)*8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_X*383+157),
                                    QPoint((sqrt(2)*0.5*abs(l_h1-l_h2)-sqrt(3)*8)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2)-8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_X*383+157),
                                    QPoint((sqrt(2)*0.5*abs(l_h1-l_h2)+90)/max_X*383+383,
                                           -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_X*383+157)));
        textPointVector->push_back(QPoint(sqrt(2)*0.5*abs(l_h1-l_h2)/max_Y*157+383,
                                          -(l_b/2+sqrt(2)*0.5*abs(l_h1-l_h2))/max_Y*157+157));


        lineVector->push_back(QLine(QPoint(0+383,l_b/2/max_X*383+157),
                                    QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_X*383+157),
                                    QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)+sqrt(3)*8)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2)+8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_X*383+157),
                                    QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)+8)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2)+sqrt(3)*8)/max_X*383+157)));
        lineVector->push_back(QLine(QPoint(-sqrt(2)*0.5*abs(l_l1-l_l2)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_X*383+157),
                                    QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)-90)/max_X*383+383,
                                           -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_X*383+157)));
        textPointVector->push_back(QPoint((-sqrt(2)*0.5*abs(l_l1-l_l2)-90)/max_Y*157+383,
                                   -(-l_b/2-sqrt(2)*0.5*abs(l_l1-l_l2))/max_Y*157+157));
    }
}

void MainWindow::inverse()
{
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
        beta_l = atan(abs(i)/(j+l_b/2))>0?atan(abs(i)/(j+l_b/2)):atan(abs(i)/(j+l_b/2))+M_PI;
        beta_h = atan(abs(i)/(-j+l_b/2))>0?atan(abs(i)/(-j+l_b/2)):atan(abs(i)/(-j+l_b/2))+M_PI;
//        qDebug()<<"beta_l"<<beta_l<<"beta_h"<<beta_h<<endl;

        theta_l = alpha_l+beta_l;
        theta_h = alpha_h+beta_h;
        if(max_X/383 < max_Y/157){
            pointVector->push_back(QPoint(i/max_Y*157+383,-j/max_Y*157+157));

            if(i>0){
                linkLineVector->push_back(QLine(QPoint(0+383,-l_b/2/max_Y*157+157),
                                        QPoint((0+l_h1*sin(theta_h))/max_Y*157+383,-(l_b/2-l_h1*cos(theta_h))/max_Y*157+157)));
                linkLineVector->push_back(QLine(QPoint((0+l_h1*sin(theta_h))/max_Y*157+383,-(l_b/2-l_h1*cos(theta_h))/max_Y*157+157),
                                         QPoint(i/max_Y*157+383,-j/max_Y*157+157)));
                linkLineVector->push_back(QLine(QPoint(0+383,l_b/2/max_Y*157+157),
                                        QPoint((0+l_l1*sin(theta_l))/max_Y*157+383,-(-l_b/2+l_l1*cos(theta_l))/max_Y*157+157)));
                linkLineVector->push_back(QLine(QPoint((0+l_l1*sin(theta_l))/max_Y*157+383,-(-l_b/2+l_l1*cos(theta_l))/max_Y*157+157),
                                         QPoint(i/max_Y*157+383,-j/max_Y*157+157)));
            }else{
                linkLineVector->push_back(QLine(QPoint(0+383,-l_b/2/max_Y*157+157),
                                        QPoint((0-l_h1*sin(theta_h))/max_Y*157+383,-(l_b/2-l_h1*cos(theta_h))/max_Y*157+157)));
                linkLineVector->push_back(QLine(QPoint((0-l_h1*sin(theta_h))/max_Y*157+383,-(l_b/2-l_h1*cos(theta_h))/max_Y*157+157),
                                         QPoint(i/max_Y*157+383,-j/max_Y*157+157)));
                linkLineVector->push_back(QLine(QPoint(0+383,l_b/2/max_Y*157+157),
                                        QPoint((0-l_l1*sin(theta_l))/max_Y*157+383,-(-l_b/2+l_l1*cos(theta_l))/max_Y*157+157)));
                linkLineVector->push_back(QLine(QPoint((0-l_l1*sin(theta_l))/max_Y*157+383,-(-l_b/2+l_l1*cos(theta_l))/max_Y*157+157),
                                         QPoint(i/max_Y*157+383,-j/max_Y*157+157)));
            }
        }else{
            pointVector->push_back(QPoint(i/max_X*383+383,-j/max_X*383+157));
        }
        update();
    }

    // caculate the kinematic parameters of the new robot
    if(!moveState)
    {
        if(j++>max_Y){
            moveState = 1;
            j = +max_Y;
            if(i++>max_X){
                timer->stop();
            }
        }
    }else{
        if(j--<-max_Y){
            moveState = 0;
            j = -max_Y;
            if(i++>max_X){
                timer->stop();
            }
        }
    }
}


MainWindow::~MainWindow()
{
    delete ui;
    delete workspacePainter;
    delete pointVector;
    delete lineVector;
    delete textPointVector;
    delete linkLineVector;
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

    // draw the widget border
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(8);
    pen.setBrush(Qt::white);
    painter.setPen(pen);
    painter.drawRect(0,0,765,313);

    // draw workspace points
    pen.setWidth(1);
    pen.setBrush(Qt::green);
    painter.setPen(pen);
    for(int i=0;i<pointVector->size();i++){
        painter.drawPoint(pointVector->at(i));
    }
    pen.setWidth(2);
    pen.setBrush(Qt::blue);
    painter.setPen(pen);
    for(int i=0;i<lineVector->size();i++){
        painter.drawLine(lineVector->at(i));
    }

    // draw the robot-link state
    pen.setWidth(4);
    pen.setBrush(Qt::red);
    painter.setPen(pen);
    for(int i=0;i<linkLineVector->size();i++){
        painter.drawLine(linkLineVector->at(i));
    }
    linkLineVector->clear();

    // draw the base axis-XY and unit
    pen.setStyle(Qt::DashDotLine);
    pen.setWidth(2);
    pen.setBrush(Qt::black);
    painter.setPen(pen);
    painter.drawLine(0,157,765,157);
    painter.drawLine(765,157,755,152);
    painter.drawLine(765,157,755,162);
    painter.drawLine(383,0,383,313);
    painter.drawLine(383,0,378,10);
    painter.drawLine(383,0,388,10);

    painter.drawText(10,30,"unit:mm");

    if(!textPointVector->isEmpty()){
        painter.drawText(textPointVector->at(0).x()+5,textPointVector->at(0).y()-5,"R_h1");
        painter.drawText(textPointVector->at(1).x()+5,textPointVector->at(1).y()-5,"R_l1");
        painter.drawText(textPointVector->at(2).x()+5,textPointVector->at(2).y()-5,"R_h2");
        painter.drawText(textPointVector->at(3).x()+5,textPointVector->at(3).y()-5,"R_l2");
    }
}

PaintWidget::~PaintWidget()
{

}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    movieMode = (arg1==2);
}


void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    if(arg1){
        timer->stop();
    }else{
        timer->start(0);
    }
}
