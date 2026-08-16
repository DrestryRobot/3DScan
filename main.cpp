#include "mainwindow3.h"
#include <QApplication>
#include <QTimer>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QMutex>
#include <QDateTime>

static QMutex g_logMutex;
static void logToFile(QtMsgType type, const QMessageLogContext&, const QString& msg)
{
    QMutexLocker locker(&g_logMutex);
    QFile f("C:/3dscan/debug.log");
    if (f.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream ts(&f);
        ts << QDateTime::currentDateTime().toString("HH:mm:ss.zzz") << " "
           << (type == QtDebugMsg ? "D" : type == QtWarningMsg ? "W" : "E")
           << " " << msg << "\n";
    }
    fprintf(stderr, "%s\n", msg.toUtf8().constData());
}

int main(int argc, char *argv[])
{
    qInstallMessageHandler(logToFile);

    QApplication a(argc, argv);
    MainWindow3 w;

    w.setMinimumSize(1080, 720);

    w.show();

    if (argc > 1) {
        const QString csv = QString::fromLocal8Bit(argv[1]);
        QTimer::singleShot(800, &w, [&w, csv]() {
            w.autoStartDebug(csv);
        });
    }

    return a.exec();
}
