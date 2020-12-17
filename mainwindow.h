#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class PaintWidget : public QWidget
{
    Q_OBJECT

public:
    PaintWidget(QWidget *parent = nullptr);
    ~PaintWidget();

private:
    void paintEvent(QPaintEvent *event);
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // the link length parameters
    float l_b = 0;
    float l_h1 = 0;
    float l_h2 = 0;
    float l_l1 = 0;
    float l_l2 = 0;

    // the vertical parameters
    float h_h = 0;
    float h_l = 0;
    float d_h = 0;
    float d_l = 0;

    // the solution
    float theta_l = 0;
    float theta_h = 0;

    PaintWidget* workspacePainter;
    QTimer* timer;

    bool movieMode = 0;
    bool moveState = 0;

private slots:
    void inverse();
    void startCaculate();

    void on_checkBox_stateChanged(int arg1);

    void on_checkBox_2_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
