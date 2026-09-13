#include "packet_sender_thread.h"

#include <QUdpSocket>
#include <QVariant>
#include <utility>

PacketSenderThread::PacketSenderThread(QObject* parent)
    : QThread(parent)
{
    start();
}

PacketSenderThread::~PacketSenderThread()
{
    stop();
    wait();
}

void PacketSenderThread::enqueue(QByteArray data, const QHostAddress& addr, quint16 port)
{
    QMutexLocker locker(&mutex_);
    queue_.enqueue({std::move(data), addr, port});
    cond_.wakeOne();
}

void PacketSenderThread::stop()
{
    QMutexLocker locker(&mutex_);
    running_ = false;
    cond_.wakeOne();
}

void PacketSenderThread::run()
{
    QUdpSocket socket;
    socket.setSocketOption(QAbstractSocket::MulticastTtlOption, QVariant(1));

    QList<Packet> batch;
    while (true) {
        batch.clear();
        {
            QMutexLocker locker(&mutex_);
            while (queue_.isEmpty() && running_) {
                cond_.wait(&mutex_);
            }
            if (!running_ && queue_.isEmpty()) {
                break;
            }
            while (!queue_.isEmpty()) {
                batch.append(queue_.dequeue());
            }
        }
        for (const Packet& pkt : batch) {
            socket.writeDatagram(pkt.data, pkt.addr, pkt.port);
        }
    }
}
