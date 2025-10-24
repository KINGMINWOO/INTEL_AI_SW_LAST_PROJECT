#ifndef SOCKETCLIENT_H
#define SOCKETCLIENT_H

#include <QWidget>
#include <QTcpSocket>
#include <QHostAddress>
#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>

#define BLOCK_SIZE 1024

class SocketClient : public QWidget
{
    Q_OBJECT
    QTcpSocket *pQTcpSocket;
    QString SERVERIP = "10.10.16.29";
    int SERVERPORT = 9999;
    QString LOGID = "USER01";
    QString LOGPW = "USER1234";

public:
    explicit SocketClient(QWidget *parent = nullptr);
    ~SocketClient();

signals:
    void socketRecvDataSig(QString strRecvData);

private slots:
    void socketReadDataSlot();
    void socketErrorSlot();
    void socketConnectServerSlot();
    void socketDisconnectedSlot();

public slots:
    void connectToServerSlot(bool &);
    void socketWriteDataSlot(QString);
private:
    bool m_startSent = false;

signals:
};

#endif // SOCKETCLIENT_H
