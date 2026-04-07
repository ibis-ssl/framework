#pragma once

#include <QByteArray>
#include <QHostAddress>
#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QWaitCondition>

// Sends UDP packets asynchronously from a dedicated worker thread.
// The producer (main/rcv thread) calls enqueue(); the worker thread
// batches and sends them so that UDP send latency does not block callers.
class PacketSenderThread : public QThread {
    Q_OBJECT
public:
    struct Packet {
        QByteArray   data;
        QHostAddress addr;
        quint16      port;
    };

    explicit PacketSenderThread(QObject* parent = nullptr);
    ~PacketSenderThread() override;

    void enqueue(QByteArray data, const QHostAddress& addr, quint16 port);
    void stop();

protected:
    void run() override;

private:
    QQueue<Packet>  queue_;
    QMutex          mutex_;
    QWaitCondition  cond_;
    bool            running_ = true;
};
