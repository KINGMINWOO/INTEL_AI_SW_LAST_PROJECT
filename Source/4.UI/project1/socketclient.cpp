#include "socketclient.h"

SocketClient::SocketClient(QWidget *parent)
    : QWidget{parent}
{
    pQTcpSocket = new QTcpSocket();
    connect(pQTcpSocket, SIGNAL(connected()), this, SLOT(socketConnectServerSlot()));
    connect(pQTcpSocket, SIGNAL(disconnected()), this, SLOT(socketClosedServerSlot()));
    connect(pQTcpSocket, SIGNAL(readyRead()), this, SLOT(socketReadDataSlot()));
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    connect(pQTcpSocket, SIGNAL(errorOccurred(QAbstractSocket::SocketError)), this, SLOT(socketErrorSlot()));
#else
    connect(pQTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(socketErrorSlot()));
#endif
}

void SocketClient::connectToServerSlot(bool &bFlag)
{
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
            socketWriteDataSlot("[CCTV01]START");
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
    const QString line = QString("%1:%2\r\n").arg(LOGID, LOGPW);
    pQTcpSocket->write(line.toUtf8());
    pQTcpSocket->flush();
}

void SocketClient::socketClosedServerSlot()
{
    pQTcpSocket->close();
}
void SocketClient::socketWriteDataSlot(QString strData)
{
    qDebug() << "[SocketClient] write req:" << strData;
    strData = strData + "\n";
    QByteArray byteData = strData.toLocal8Bit();
    pQTcpSocket->write(byteData);
}

SocketClient::~SocketClient()
{

}
