#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    sock = new QTcpSocket();
    connect(sock,SIGNAL(readyRead()),this,SLOT(sock_recv()));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete sock;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_btn_ipok_clicked()
{
    sock->connectToHost(QHostAddress(ui->ledt_ip->text()),HOST_PORT);
    ui->ledt_ip->setEnabled(false);
    ui->btn_ipok->setEnabled(false);
    ui->btn_get->setEnabled(true);
}

void MainWindow::on_btn_get_clicked()
{
    struct prot * buff = (struct prot*) malloc(sizeof(uint32_t));
    buff->types = PROT_GET_ANGLE ;
    sock->write((char *)buff,sizeof(uint32_t));
    ui->lb_angle_xy->setText(tr("..."));
    ui->lb_angle_yz->setText(tr("..."));
    ui->lb_angle_zx->setText(tr("..."));
}

void MainWindow::sock_recv()
{
    char buff[BUFF_MAX];
    struct prot *tprot = NULL;
    struct pos_stat *tpos = NULL;
    sock->read(buff,BUFF_MAX);
    tprot = (struct prot*)buff;
    tpos = (struct pos_stat*)tprot->data;
    ui->lb_angle_xy->setText(QString::number((double)tpos->x_y/(1<<15),'f',5));
    ui->lb_angle_yz->setText(QString::number((double)tpos->y_z/(1<<15),'f',5));
    ui->lb_angle_zx->setText(QString::number((double)tpos->z_x/(1<<15),'f',5));
}
