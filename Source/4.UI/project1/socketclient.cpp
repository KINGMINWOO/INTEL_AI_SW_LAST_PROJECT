#include "socketclient.h"

SocketClient::SocketClient(QWidget *parent)
    : QWidget{parent}
{
    pQTcpSocket = new QTcpSocket(this);
    connect(pQTcpSocket, SIGNAL(connected()), this, SLOT(socketConnectServerSlot()));
    connect(pQTcpSocket, SIGNAL(readyRead()), this, SLOT(socketReadDataSlot()));
    connect(pQTcpSocket, SIGNAL(disconnected()), this, SLOT(socketDisconnectedSlot()));
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    connect(pQTcpSocket, SIGNAL(errorOccurred(QAbstractSocket::SocketError)), this, SLOT(socketErrorSlot()));
#else
    connect(pQTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(socketErrorSlot()));
#endif
}

void SocketClient::connectToServerSlot(bool &bFlag)
{
    QAbstractSocket::SocketState st = pQTcpSocket->state();
    if (st == QAbstractSocket::ConnectedState ||
      st == QAbstractSocket::ConnectingState) {
      qDebug() << "[SocketClient] already" << (st==QAbstractSocket::ConnectedState ? "connected" : "connecting")
               << "— skip new connection attempt";
      bFlag = false;
      return;
    }
    if (!bFlag) {
            pQTcpSocket->connectToHost(SERVERIP, SERVERPORT);
            return;
        }
    QString strHostIp;
    strHostIp = QInputDialog::getText(this,"Host Ip", "Input Server IP",QLineEdit::Normal,SERVERIP, &bFlag);
    if(bFlag)
    {
        if(strHostIp.isEmpty())
            pQTcpSocket->connectToHost(SERVERIP, SERVERPORT);
        else
            pQTcpSocket->connectToHost(strHostIp, SERVERPORT);
    }

}

void SocketClient::socketReadDataSlot()
{
    QByteArray byteRecvData;
    QString strRecvData;
    if(pQTcpSocket->bytesAvailable() > BLOCK_SIZE)
        return;
    byteRecvData = pQTcpSocket->read(BLOCK_SIZE);
    strRecvData = QString::fromLocal8Bit(byteRecvData);
    qDebug() << strRecvData;
    emit socketRecvDataSig(strRecvData);
    if (!m_startSent && strRecvData.contains("AUTH_OK", Qt::CaseInsensitive)) {
            m_startSent = true;
        }

}
void SocketClient::socketErrorSlot()
{
    QString strError = pQTcpSocket->errorString();
    QMessageBox::information(this, "socket", "error : "+strError);
}
void SocketClient::socketConnectServerSlot()
{
    if (!m_startSent) { // AUTH_OK 받기 전까지만 로그인 보냄
        const QString line = QString("%1:%2\r\n").arg(LOGID, LOGPW);
        pQTcpSocket->write(line.toUtf8());
        pQTcpSocket->flush();
        qDebug() << "[SocketClient] login sent";
    }
    else {
        qDebug() << "[SocketClient] login suppressed (already authed)";
    }
}
void SocketClient::socketWriteDataSlot(QString strData)
{
    qDebug() << "[SocketClient] write req:" << strData;
    strData = strData + "\n";
    QByteArray byteData = strData.toLocal8Bit();
    pQTcpSocket->write(byteData);
}
void SocketClient::socketDisconnectedSlot()
{
    qDebug() << "[SocketClient] disconnected — reset auth flag";
    m_startSent = false;
}

SocketClient::~SocketClient()
{

}
