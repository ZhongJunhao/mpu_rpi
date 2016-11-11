#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <stdio.h>
#include <sys/types.h>
#include <QHostAddress>

#define HOST_PORT  	6666


#define MPU_IOC_GET_ANGLE       ((0x2<<30)|(sizeof(int32_t)<<16)|('M'<<8)|1)


#define PROT_GET_ANGLE      1
#define PROT_RET_ANGLE      2

struct prot{
        uint32_t types;
        char data[0];
};

struct pos_stat{
        //x0y平面内G投影与x轴的夹角
        int32_t x_y;
        //y0z平面内G投影与y轴的夹角
        int32_t y_z;
        //x0z平面内G投影与z轴的夹角
        int32_t z_x;
};

#define BUFF_MAX   20
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;
    QTcpSocket * sock;
private slots:
    void on_btn_get_clicked();
    void on_btn_ipok_clicked();
    void sock_recv();
};

#endif // MAINWINDOW_H
