#include "mainwidget.h"

#include <QApplication>
#include <QLocalServer>
#include <QLocalSocket>

static const char* kIpcKey = "sf_cli";  // 프로젝트 고유 키(짧게)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QApplication::setQuitOnLastWindowClosed(false);

    // 1) 먼저 기존 인스턴스가 살아있는지 확인: 살아있으면 창만 올리고 종료
    {
        QLocalSocket probe;
        probe.connectToServer(kIpcKey, QIODevice::WriteOnly);
        if (probe.waitForConnected(150)) {
            probe.write("RAISE");
            probe.flush();
            probe.waitForBytesWritten(150);
            return 0;
        }
    }

    // 2) 살아있는 인스턴스가 없을 때만 서버 소켓 열기(이전에 남은 소켓 파일 정리)
    QLocalServer::removeServer(kIpcKey);

    QLocalServer server;
    if (!server.listen(kIpcKey)) {
        // 경로/권한 문제 시 여기서 종료됨
        return 1;
    }

    MainWidget w;
    w.setFixedSize(800, 480);
    w.show();

    // 3) 두 번째 실행이 접속하면 기존 창을 앞으로
    QObject::connect(&server, &QLocalServer::newConnection, [&](){
        if (auto s = server.nextPendingConnection()) {
            QObject::connect(s, &QLocalSocket::readyRead, [&](){
                const QByteArray cmd = s->readAll();
                if (cmd.contains("RAISE")) {
                    w.bringToFront();
                }
            });
            QObject::connect(s, &QLocalSocket::disconnected, s, &QLocalSocket::deleteLater);
        }
    });

    return a.exec();
}
