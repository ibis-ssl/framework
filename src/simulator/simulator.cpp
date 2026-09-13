/****************************************************************************
 *   Copyright 2021 Tobias Heineken                                        *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
#include <clocale>
#include <QCoreApplication>
#include <QUdpSocket>
#include <QThread>
#include <QNetworkDatagram>
#include <QCommandLineParser>
#include <QTime>
#include <QTimer>
#include <cmath>
#include <cstdio>
#include <cstdarg>

#include "core/protobufhelper.h"
#include "protobuf/ssl_sim/ssl_simulation_robot_control.pb.h"
#include "protobuf/ssl_sim/ssl_simulation_robot_feedback.pb.h"
#include "protobuf/ssl_sim/ssl_simulation_custom_erforce_robot_spec.pb.h"
#include "protobuf/sslsim.h"
#include "protobuf/status.h"
#include "protobuf/command.h"
#include "protobuf/geometry.h"
#include "protobuf/robot.h"
#include "simulator/simulator.h"

#include "core/timer.h"
#include "core/run_out_of_scope.h"
#include "core/configuration.h"
#include "core/coordinates.h"
#include "core/sslprotocols.h"

#include "ssl_robocup_server.h"
#include "ibis_protocol.h"
#include "packet_sender_thread.h"

#include "protobuf/ssl_vision/ssl_wrapper.pb.h"
#include "protobuf/ssl_gc/state/ssl_gc_referee_message.pb.h"
#include "protobuf/world.pb.h"

/**
 * Stand alone Erforce simulator
 *
 * Known issues:
 *  - [ ]: Currently, it is not possible to supply partial positions for teleportBall or teleportRobot
 *  - [ ]: Robots go into standby after 0.1 seconds without command (Safty)
 *  - [ ]: It is not possible to change specs or geometry without resetting the world
 *  - [ ]: It is not possible to setUp a team with no robots (You can still teleport them away)
 *  - [ ]: It is not possible to supply only partial specs. It is planned to fuse them with the last specs for this team if not supplied, but NYI.
 *  - [ ]: Dribbler will reset if a new command doesn't contain a new dribbling speed (contrary to the definition that states all not set values should stay as previously assumed)
 *  - [ ]: Commands that are recieved at t0 will not be in effect after the next tick of the simulator (around 5 ms), no interpolation.
 *  - [ ]: Tournament mode where commands origin are checked is not implemented
 */

// Check log format strings
#if defined(__GNUC__) || defined(__CLANG__)
#define CHECK_PRINTF __attribute__((format(printf, 2, 3)))
#else
#define CHECK_PRINTF
#endif

static void CHECK_PRINTF log(FILE* stream, const char* fmt, ...) {
    const std::string timeStr = QTime::currentTime().toString().toStdString();
    std::fputs(timeStr.c_str(), stream);
    std::fputc(' ', stream);

    va_list args;
    va_start(args, fmt);
    std::vfprintf(stream, fmt, args);
    va_end(args);
}

class SSLVisionServer: public QObject {
    Q_OBJECT
public:
    SSLVisionServer(int port, const string &net_address);
    void setPort(int port);


public slots:
    void sendVisionData(const QByteArray& data, qint64 time, QString sender);

private:
    RoboCupSSLServer m_server;
};

class SimulatorCommandAdaptor: public QObject {
    Q_OBJECT
public:
    SimulatorCommandAdaptor(Timer* timer, SSLVisionServer *vision);
private slots:
    void handleDatagrams();

public slots:
    void handleSimulatorError(const QList<SSLSimError> &error, camun::simulator::ErrorSource source);

signals:
    void sendCommand(const Command& c);

private:
    QUdpSocket m_server;
    QHostAddress m_senderAddress;
    int m_senderPort;
    Timer* m_timer; // unowned
    SSLVisionServer* m_visionServer; // unowned
};

SimulatorCommandAdaptor::SimulatorCommandAdaptor(Timer* timer, SSLVisionServer* vision):
    m_server(this),
    m_senderAddress(QHostAddress::Null),
    m_senderPort(-1),
    m_timer(timer),
    m_visionServer(vision)
{
    m_server.bind(QHostAddress::Any, SSL_SIMULATION_CONTROL_PORT);
    connect(&m_server, &QUdpSocket::readyRead, this, &SimulatorCommandAdaptor::handleDatagrams);
}

class RobotCommandAdaptor: public QObject{
    Q_OBJECT
public:
    RobotCommandAdaptor(bool blue, Timer* timer);

private:
    void sendRobotRespose(const sslsim::RobotControlResponse& rcr);

public slots:
    void handleRobotResponse(const QList<robot::RadioResponse>& responses);
    void handleSimulatorError(const QList<SSLSimError> &error, camun::simulator::ErrorSource source);

private slots:
    void handleDatagrams();

signals:
    void sendRadioCommands(const SSLSimRobotControl & commands, bool isBlue, qint64 processingDelay);


private:
    bool m_is_blue;
    QUdpSocket m_server;
    QHostAddress m_senderAddress;
    int m_senderPort;
    Timer* m_timer; // unowned
};

RobotCommandAdaptor::RobotCommandAdaptor(bool blue, Timer* timer): m_is_blue(blue),
    m_server(this),
    m_senderAddress(QHostAddress::Null),
    m_senderPort(-1),
    m_timer(timer)
{
    m_server.bind(QHostAddress::Any, (blue)? SSL_SIMULATION_CONTROL_BLUE_PORT : SSL_SIMULATION_CONTROL_YELLOW_PORT);
    connect(&m_server, &QUdpSocket::readyRead, this, &RobotCommandAdaptor::handleDatagrams);
}

enum class SimError {
    UNSUPPORTED_VELOCITY,
    UNSUPPORTED_ANGLE,
    UNREADABLE,
    MISSING_SPEC,
    INVALID_REALISM,
};

enum class SimErrorSource {
    CONTROLLER,
    BLUE_TEAM,
    YELLOW_TEAM,
};

static void setError(sslsim::SimulatorError* error, SimError code, SimErrorSource source, std::string appendix = "") {
    const char* codeStr = nullptr;
    std::string message;
    switch(code) {
        case SimError::UNREADABLE:
            codeStr = "UNREADABLE";
            message = "The received message was unreadable " + appendix;
            break;
        case SimError::UNSUPPORTED_VELOCITY:
            codeStr = "VELOCITY_TYPE";
            message = "The received message had a velocity type unsupported by this simulator " + appendix;
            break;
        case SimError::UNSUPPORTED_ANGLE:
            codeStr = "ANGLE_VALUE";
            message = "The received kick angle was not equal to either 0 or 45 " + appendix;
            break;
        case SimError::MISSING_SPEC:
            codeStr = "INVALID_SPEC";
            message = "The received spec is missing one of the required fields for this simulator " + appendix;
            break;
        case SimError::INVALID_REALISM:
            codeStr = "INVALID_REALISM";
            message = "The received realism is not conforming to the realism configuration for this simulator " + appendix;
            break;
        default:
            log(stderr, "Unmanaged SimError for message\n");
            break;
    }
    if (!codeStr || message.size() == 0) {
        return;
    }
    error->set_code(codeStr);
    error->set_message(message);

    const char* sourceStr = [source]() {
        switch (source) {
            case SimErrorSource::CONTROLLER:
                return "CONTROLLER";
            case SimErrorSource::BLUE_TEAM:
                return "BLUE";
            case SimErrorSource::YELLOW_TEAM:
                return "YELLOW";
            default:
                return "INVALID";
        }
    }();

    log(stderr, "[%-10s - %-15s] %s\n", sourceStr, codeStr, message.c_str());
}

static void sendUDP(const google::protobuf::Message& out, QUdpSocket& server, const QHostAddress& senderAddress, int senderPort) {
    QByteArray data = protobufhelper::bufferWithSpaceFor(out);
    bool sendingSuccessful = false;
    if (out.SerializeToArray(data.data(), data.size())) {
        sendingSuccessful = server.writeDatagram(data, senderAddress, senderPort) == data.size();
    }
    if (!sendingSuccessful) {
        log(stderr, "Sending reply failed:\n");
    }
}

#define SCALE_UP(OBJ, ATTR) do{if((OBJ).has_##ATTR()) (OBJ).set_##ATTR((OBJ).ATTR() * 1e3);} while(0)

static void warnLatency(qint64 delta) {
    if (delta > 1e6) {
        log(stdout, "Warning: Handled Datagram in %lldns, should be lower than 1e6\n", delta);
    }
}


//TODO: Always update the following constant if the robotSpecs did change,
// either disregard the new field and just increase the expected number if the new field is useless to our simulator,
// or convert it properly and update this number.
constexpr int expected_specs_fields = 10 + 3;
constexpr int functionToFixForSpecs = __LINE__;
template<class T>
static bool convertSpecsToErForce(T outGen, const sslsim::RobotSpecs& in) // @return false: Error occured
{
    if (!in.has_mass()) {
        return false;
    }
    if (!in.has_limits()) {
        return false;
    }
    if (!in.has_center_to_dribbler()) {
        return false;
    }
    sslsim::RobotSpecErForce rsef;
    bool rsefInitialized = false;
    for(const auto& cus : in.custom()) {
        if (cus.UnpackTo(&rsef)) {
            rsefInitialized = true;
            break;
        }
    }
    if (!rsefInitialized) {
        return false;
    }
    if (!rsef.has_shoot_radius()) {
        return false;
    }
    /*if (!rsef.has_dribbler_height()) {
        return false;
    }*/
    if (!rsef.has_dribbler_width()) {
        return false;
    }
    const sslsim::RobotLimits& lim = in.limits();
    if (!lim.has_acc_speedup_absolute_max()) {
        return false;
    }
    if (!lim.has_acc_speedup_angular_max()) {
        return false;
    }
    if (!lim.has_acc_brake_absolute_max()) {
        return false;
    }
    if (!lim.has_acc_brake_angular_max()) {
        return false;
    }
    if (!lim.has_vel_absolute_max()) {
        return false;
    }
    if (!lim.has_vel_angular_max()) {
        return false;
    }
    if (!in.id().has_id()) {
        return false;
    }
    if (!in.id().has_team()) {
        return false;
    }
    robot::Specs* out = outGen(in.id().team() == gameController::BLUE);
    out->set_year(1970);
    out->set_generation(0);
    out->set_id(in.id().id());
    out->set_type(robot::Specs_GenerationType_Regular);
    out->set_radius(in.radius());
    out->set_height(in.height());
    out->set_mass(in.mass());
    out->set_v_max(lim.vel_absolute_max());
    out->set_omega_max(lim.vel_angular_max());
    if (in.has_max_linear_kick_speed()) {
        out->set_shot_linear_max(in.max_linear_kick_speed());
    } else {
        out->set_shot_linear_max(100);
    }
    if (in.has_max_chip_kick_speed()) {
        out->set_shot_chip_max(coordinates::chipDistanceFromChipVel(in.max_chip_kick_speed()));
    } else {
        out->set_shot_chip_max(100);
    }

    out->set_dribbler_width(rsef.dribbler_width());
    auto* acc = out->mutable_strategy();

    acc->set_a_speedup_f_max(lim.acc_speedup_absolute_max());
    acc->set_a_speedup_s_max(lim.acc_speedup_absolute_max());
    acc->set_a_speedup_phi_max(lim.acc_speedup_angular_max());
    acc->set_a_brake_f_max(lim.acc_brake_absolute_max());
    acc->set_a_brake_s_max(lim.acc_brake_absolute_max());
    acc->set_a_brake_phi_max(lim.acc_brake_angular_max());

    out->set_shoot_radius(rsef.shoot_radius());
    out->set_dribbler_height(0.04/*rsef.dribbler_height()*/); //FIXME: We use only our specs, because we don't know if dribbling will be possible at all with any other values :/


    // cos(angle / 2 ) = center_to_dribbler / radius
    const float ratio = in.center_to_dribbler() / in.radius();
    out->set_angle(2 * acosf(ratio));
    return true;
            /*
// Movement limits for a robot
message RobotLimits {
    // Max absolute speed-up acceleration [m/s^2]
    optional float acc_speedup_absolute_max = 1;
    // Max angular speed-up acceleration [rad/s^2]
    optional float acc_speedup_angular_max = 2;
    // Max absolute brake acceleration [m/s^2]
    optional float acc_brake_absolute_max = 3;
    // Max angular brake acceleration [rad/s^2]
    optional float acc_brake_angular_max = 4;
    // Max absolute velocity [m/s]
    optional float vel_absolute_max = 5;
    // Max angular velocity [rad/s]
    optional float vel_angular_max = 6;
    */
}


void SimulatorCommandAdaptor::handleDatagrams() {
    while(m_server.hasPendingDatagrams()) {
        qint64 start = m_timer->currentTime();
        auto datagram = m_server.receiveDatagram();
        sslsim::SimulatorResponse sir;
        bool sendSir = false;
        m_senderAddress = datagram.senderAddress();
        m_senderPort = datagram.senderPort();
        auto data = datagram.data();

        RUN_WHEN_OUT_OF_SCOPE({
                if (sendSir) {
                    sendUDP(sir, m_server, m_senderAddress, m_senderPort);
                }
            });
        sslsim::SimulatorCommand simcom;
        if (!simcom.ParseFromArray(data.data(), data.size())) {
            sendSir = true;
            setError(sir.add_errors(), SimError::UNREADABLE, SimErrorSource::CONTROLLER);
            continue;
        }
        if (simcom.has_control()) {
            Command c{new amun::Command};
            auto* sslControl = c->mutable_simulator()->mutable_ssl_control();
            sslControl->CopyFrom(simcom.control());
            if (sslControl->has_teleport_ball()) {
                auto* teleportBall = sslControl->mutable_teleport_ball();
                SCALE_UP(*teleportBall, x);
                SCALE_UP(*teleportBall, y);
                SCALE_UP(*teleportBall, z);
                SCALE_UP(*teleportBall, vx);
                SCALE_UP(*teleportBall, vy);
                SCALE_UP(*teleportBall, vz);
            }
            for(sslsim::TeleportRobot& robot : *sslControl->mutable_teleport_robot()) {
                SCALE_UP(robot, x);
                SCALE_UP(robot, y);
                SCALE_UP(robot, v_x);
                SCALE_UP(robot, v_y);
            }
            emit sendCommand(c);
        }
        if (simcom.has_config()) {
            const auto& config{simcom.config()};

            if (config.has_geometry()) {
                Command c{new amun::Command};
                auto* setup = c->mutable_simulator()->mutable_simulator_setup();
                convertFromSSlGeometry(config.geometry().field(), *(setup->mutable_geometry()));
                setup->mutable_camera_setup()->CopyFrom(config.geometry().calib());
                emit sendCommand(c);
            }

            if (config.robot_specs_size() > 0) {
                Command c{new amun::Command};
                robot::Team* blueTeam = nullptr;
                robot::Team* yellowTeam = nullptr;
                auto newSz = config.robot_specs_size();
                for (const auto& spec : config.robot_specs()) {
                    bool success = convertSpecsToErForce([&blueTeam, &yellowTeam, &c](bool isBlue){
                            if (isBlue) {
                                if (blueTeam == nullptr) {
                                    blueTeam = c->mutable_set_team_blue();
                                }
                                return blueTeam->add_robot();
                            }
                            if (yellowTeam == nullptr) {
                                yellowTeam = c->mutable_set_team_yellow();
                            }
                            return yellowTeam->add_robot();
                            }
                            , spec);
                    if (!success) {
                        sendSir = true;
                        setError(sir.add_errors(), SimError::MISSING_SPEC, SimErrorSource::CONTROLLER, spec.DebugString());
                        newSz--;
                    }
                }
                log(stdout, "Updated to %d robots\n", newSz);
                emit sendCommand(c);
            }
            if (config.has_realism_config()) {
                for(const auto& c : config.realism_config().custom()) {
                RealismConfigErForce rcef;
                    if (c.UnpackTo(&rcef)) {
                        Command c{new amun::Command};
                        c->mutable_simulator()->mutable_realism_config()->CopyFrom(rcef);
                        emit sendCommand(c);
                    }
                }
            }
            if (config.has_vision_port()) {
                m_visionServer->setPort(config.vision_port());
            }
        }

        qint64 delta = m_timer->currentTime() - start;
        warnLatency(delta);
    }
}

void RobotCommandAdaptor::handleSimulatorError(const QList<SSLSimError> &error,camun::simulator::ErrorSource source)
{

    camun::simulator::ErrorSource expected = m_is_blue ? camun::simulator::ErrorSource::BLUE : camun::simulator::ErrorSource::YELLOW;
    if (source != expected) return;
    if (error.size() == 0) return;

    sslsim::RobotControlResponse rcr;

    for(const SSLSimError& err : error) {
        auto* sendError = rcr.add_errors();
        *sendError = *err;
    }

    sendRobotRespose(rcr);
}

void SimulatorCommandAdaptor::handleSimulatorError(const QList<SSLSimError> &error, camun::simulator::ErrorSource source) {
    if (source != camun::simulator::ErrorSource::CONFIG) return;
    if (error.size() == 0) return;
    sslsim::SimulatorResponse sir;
    for (const SSLSimError& err : error) {
        sir.add_errors()->CopyFrom(*err);
    }
    sendUDP(sir, m_server, m_senderAddress, m_senderPort);
}


void RobotCommandAdaptor::handleDatagrams()
{
    const SimErrorSource ERROR_SOURCE = m_is_blue
        ? SimErrorSource::BLUE_TEAM
        : SimErrorSource::YELLOW_TEAM;
    while(m_server.hasPendingDatagrams()) {
        qint64 start =m_timer->currentTime();
        sslsim::RobotControlResponse rcr;
        bool sendRcr = false;
        auto datagram = m_server.receiveDatagram();
        // TODO: do something with m_senderAddress and datagram.senderAddress
        m_senderAddress = datagram.senderAddress();
        m_senderPort = datagram.senderPort();
        auto data = datagram.data();

        RUN_WHEN_OUT_OF_SCOPE({
                if (sendRcr) {
                    sendRobotRespose(rcr);
                }
            });

        SSLSimRobotControl control{new sslsim::RobotControl};
        if (!control->ParseFromArray(data.data(), data.size())) {
            sendRcr = true;
            setError(rcr.add_errors(), SimError::UNREADABLE, ERROR_SOURCE);
            continue;
        }

        for (const auto& command : control->robot_commands()) {
            if (command.has_move_command()) {
                const auto& moveCmd = command.move_command();
                if (moveCmd.has_wheel_velocity() || moveCmd.has_global_velocity()) {
                    sendRcr = true;
                    const std::string robotStr = "(Robot :" + std::to_string(command.id()) + ")";
                    setError(rcr.add_errors(), SimError::UNSUPPORTED_VELOCITY, ERROR_SOURCE, robotStr);
                }
            }
        }
        emit sendRadioCommands(control, m_is_blue, m_timer->currentTime()); // This might be a bit late.
        // TODO: response!
        qint64 delta = m_timer->currentTime() - start;

        warnLatency(delta);
    }
}

void RobotCommandAdaptor::handleRobotResponse(const QList<robot::RadioResponse>& res) {
    if (m_senderAddress.isNull()) {
        return;
    }

    sslsim::RobotControlResponse out;
    bool send = false;

    for (const auto& response : res) {
        if (response.has_is_blue() && response.is_blue() == m_is_blue && response.has_ball_detected()) {
            auto* outFeedback = out.add_feedback();
            outFeedback->set_id(response.id());
            outFeedback->set_dribbler_ball_contact(response.ball_detected());
            send = true;
        }
    }

    if (send) {
        sendRobotRespose(out);
    }
}

void RobotCommandAdaptor::sendRobotRespose(const sslsim::RobotControlResponse& out) {
    sendUDP(out, m_server, m_senderAddress, m_senderPort);
}


SSLVisionServer::SSLVisionServer(int port, const string &net_address): m_server(this, port, net_address)
{
}

void SSLVisionServer::sendVisionData(const QByteArray& data, qint64, QString)
{
    m_server.send(data);
}

void SSLVisionServer::setPort(int port) {
    m_server.change_port(port);
}

using camun::simulator::Simulator;
using camun::simulator::ErrorSource;

class SimProxy: public QObject {
    Q_OBJECT
public:
    SimProxy(Timer* t): m_timer(t) {}
signals:
    void sendSSLSimError(const QList<SSLSimError>& errors, ErrorSource source); // out
    void sendRadioResponses(const QList<robot::RadioResponse> &responses); // out
    void gotPacket(const QByteArray &data, qint64 time, QString sender); // out
    void sendGroundTruth(const QByteArray& data); // out - world::SimulatorState at 125Hz
    void gotCommand(const Command &command); // internal
    void handleRadioCommands(const SSLSimRobotControl& control, bool isBlue, qint64 processingStart); // in
public slots:
    void handleCommand(const Command &command);

private:
    Timer* m_timer;
    Simulator* m_sim = nullptr;
    Command m_teamCommand{new amun::Command};
};

void SimProxy::handleCommand(const Command &command) {
    bool hasSimSetup = command->has_simulator() && command->simulator().has_simulator_setup();

    if (command->has_set_team_blue()) {
        m_teamCommand->mutable_set_team_blue()->CopyFrom(command->set_team_blue());
        if (hasSimSetup) {
            command->clear_set_team_blue();
        }
    }
    if (command->has_set_team_yellow()) {
        m_teamCommand->mutable_set_team_yellow()->CopyFrom(command->set_team_yellow());
        if (hasSimSetup) {
            command->clear_set_team_yellow();
        }
    }
    if (command->has_simulator() && command->simulator().has_realism_config()) {
        m_teamCommand->mutable_simulator()->mutable_realism_config()->CopyFrom(command->simulator().realism_config());
        if (hasSimSetup) {
            command->mutable_simulator()->clear_realism_config();
        }
    }
    if (hasSimSetup) {
        // replace m_sim
        if (m_sim != nullptr) {
            // replace old connectios
            m_sim->blockSignals(true);
            m_sim->deleteLater();
        }
        m_sim = new Simulator(m_timer, command->simulator().simulator_setup());
        connect(this, &SimProxy::gotCommand, m_sim, &Simulator::handleCommand);
        connect(m_sim, &Simulator::gotPacket, this, &SimProxy::gotPacket);
        connect(this, &SimProxy::handleRadioCommands, m_sim, &Simulator::handleRadioCommands);
        connect(m_sim, &Simulator::sendSSLSimError, this, &SimProxy::sendSSLSimError);
        connect(m_sim, &Simulator::sendRadioResponses, this, &SimProxy::sendRadioResponses);
        connect(m_sim, &Simulator::sendGroundTruth, this, &SimProxy::sendGroundTruth);
        auto* simCommand = m_teamCommand->mutable_simulator();
        simCommand->set_enable(true);
        auto* trCommand = m_teamCommand->mutable_transceiver();
        trCommand->set_charge(true);
        emit gotCommand(m_teamCommand);
    }
    emit gotCommand(command);
}

class IbisCommandAdaptor : public QObject {
    Q_OBJECT
public:
    IbisCommandAdaptor(int port, Timer* timer, double accSpeedup, double accBrake)
        : m_server(this)
        , m_timer(timer)
        , m_accSpeedup(accSpeedup)
        , m_accBrake(accBrake)
    {
        m_server.bind(QHostAddress::Any, static_cast<quint16>(port));
        connect(&m_server, &QUdpSocket::readyRead, this, &IbisCommandAdaptor::handleDatagrams);
    }

public slots:
    void handleVisionData(const QByteArray& data, qint64, QString) {
        SSL_WrapperPacket pkt;
        if (!pkt.ParseFromArray(data.data(), data.size()) || !pkt.has_detection()) {
            return;
        }
        const auto& det = pkt.detection();
        for (const auto& r : det.robots_blue()) {
            if (r.has_robot_id() && r.has_orientation()) {
                const uint32_t id = r.robot_id();
                if (id < kMaxRobots) {
                    m_vision[0][id] = {r.x(), r.y(), r.orientation(), true};
                }
            }
        }
        for (const auto& r : det.robots_yellow()) {
            if (r.has_robot_id() && r.has_orientation()) {
                const uint32_t id = r.robot_id();
                if (id < kMaxRobots) {
                    m_vision[1][id] = {r.x(), r.y(), r.orientation(), true};
                }
            }
        }
    }

signals:
    void sendRadioCommands(const SSLSimRobotControl& commands, bool isBlue, qint64 processingDelay);

private slots:
    void handleDatagrams() {
        while (m_server.hasPendingDatagrams()) {
            const qint64 start = m_timer->currentTime();
            auto datagram = m_server.receiveDatagram();
            const auto& data = datagram.data();

            if (data.size() != IBIS_PACKET_SIZE) {
                continue;
            }

            const uint8_t* buf = reinterpret_cast<const uint8_t*>(data.constData());

            SSLSimRobotControl blueControl{new sslsim::RobotControl};
            SSLSimRobotControl yellowControl{new sslsim::RobotControl};
            bool hasBlue = false, hasYellow = false;

            for (int slot = 0; slot < IBIS_ROBOT_SLOTS; ++slot) {
                const int offset = slot * IBIS_SLOT_SIZE;
                const uint8_t robot_id = buf[offset];
                if (robot_id >= IBIS_ROBOT_SLOTS) {
                    continue;
                }

                const uint8_t* cmd_data = buf + offset + 1;
                // Senders zero-fill the slots of robots they do not control.
                if (ibisSlotIsEmpty(cmd_data)) {
                    continue;
                }
                if (cmd_data[CHECK_COUNTER] == m_robotStates[robot_id].last_check_counter) {
                    continue;
                }

                const IbisCommand cmd = ibisDeserialize(cmd_data);
                m_robotStates[robot_id].last_check_counter = cmd.check_counter;

                // Team auto-detection: match vision_global_pos against cached positions.
                // Also keeps the matched vision entry for orientation lookup below.
                int teamIdx = -1;
                const IbisVisionState* vis = nullptr;
                double nearest = -1.0;
                for (int t = 0; t < 2; ++t) {
                    const IbisVisionState& v = m_vision[t][robot_id];
                    if (!v.valid) { continue; }
                    const float dx = v.x_mm / 1000.0f - cmd.vision_global_pos[0];
                    const float dy = v.y_mm / 1000.0f - cmd.vision_global_pos[1];
                    const double dist = std::hypot(dx, dy);
                    if (nearest < 0.0 || dist < nearest) {
                        nearest = dist;
                    }
                    if (dist < IBIS_POSITION_MATCH_THRESHOLD) {
                        teamIdx = t;
                        vis = &v;
                        break;
                    }
                }
                if (teamIdx < 0) {
                    warnPositionMismatch(robot_id, cmd, nearest);
                    continue;
                }
                m_robotStates[robot_id].match_warned = false;
                const bool ibisIsBlue = (teamIdx == 0);

                auto* robotCmd = ibisIsBlue
                    ? blueControl->add_robot_commands()
                    : yellowControl->add_robot_commands();
                robotCmd->set_id(robot_id);

                if (ibisShouldStop(cmd)) {
                    auto* lv = robotCmd->mutable_move_command()->mutable_local_velocity();
                    lv->set_forward(0.0f);
                    lv->set_left(0.0f);
                    lv->set_angular(0.0f);
                    m_robotStates[robot_id].prev_vx = 0.0;
                    m_robotStates[robot_id].prev_vy = 0.0;
                } else if (cmd.control_mode != IBIS_MODE_POLAR_VELOCITY_TARGET) {
                    // This adaptor emulates the robot's STM32 (G474) main board, which
                    // implements POLAR_VELOCITY_TARGET only. Any other mode means the
                    // chain is misconfigured -- most likely a POSITION_TARGET command
                    // that should have been consumed by the robot-side position loop
                    // (cm4_sim) before reaching the simulator. Hold the robot still and
                    // say why, rather than steering on reinterpreted mode args.
                    warnUnsupportedMode(robot_id, cmd.control_mode);
                    auto* lv = robotCmd->mutable_move_command()->mutable_local_velocity();
                    lv->set_forward(0.0f);
                    lv->set_left(0.0f);
                    lv->set_angular(0.0f);
                    m_robotStates[robot_id].prev_vx = 0.0;
                    m_robotStates[robot_id].prev_vy = 0.0;
                } else {
                    const double current_theta = vis->orientation_rad;

                    double theta_error = cmd.target_global_theta - current_theta;
                    while (theta_error >  M_PI) theta_error -= 2.0 * M_PI;
                    while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

                    double omega = IBIS_THETA_P_GAIN * theta_error;
                    omega = std::max(-static_cast<double>(cmd.angular_velocity_limit),
                                     std::min(omega, static_cast<double>(cmd.angular_velocity_limit)));

                    const double predicted_theta = current_theta + omega * IBIS_DT;
                    const double vel_angle = cmd.polar_velocity_theta - predicted_theta;
                    double target_vx = cmd.polar_velocity_r * std::cos(vel_angle);
                    double target_vy = cmd.polar_velocity_r * std::sin(vel_angle);

                    auto& state = m_robotStates[robot_id];
                    const double current_speed = std::hypot(state.prev_vx, state.prev_vy);
                    const double target_speed  = std::hypot(target_vx, target_vy);
                    double acc_limit = (target_speed < current_speed) ? m_accBrake : m_accSpeedup;
                    if (cmd.acceleration_limit > 0.0f && cmd.acceleration_limit < static_cast<float>(acc_limit)) {
                        acc_limit = cmd.acceleration_limit;
                    }

                    const double delta_vx   = target_vx - state.prev_vx;
                    const double delta_vy   = target_vy - state.prev_vy;
                    const double delta_norm = std::hypot(delta_vx, delta_vy);
                    const double max_delta  = acc_limit * IBIS_DT;

                    double out_vx, out_vy;
                    if (delta_norm > max_delta && delta_norm > 1e-9) {
                        out_vx = state.prev_vx + (delta_vx / delta_norm) * max_delta;
                        out_vy = state.prev_vy + (delta_vy / delta_norm) * max_delta;
                    } else {
                        out_vx = target_vx;
                        out_vy = target_vy;
                    }

                    const double out_speed = std::hypot(out_vx, out_vy);
                    if (cmd.linear_velocity_limit > 0.0f && out_speed > cmd.linear_velocity_limit) {
                        out_vx = out_vx / out_speed * cmd.linear_velocity_limit;
                        out_vy = out_vy / out_speed * cmd.linear_velocity_limit;
                    }

                    state.prev_vx = out_vx;
                    state.prev_vy = out_vy;

                    auto* lv = robotCmd->mutable_move_command()->mutable_local_velocity();
                    lv->set_forward(static_cast<float>(out_vx));
                    lv->set_left(static_cast<float>(out_vy));
                    lv->set_angular(static_cast<float>(omega));

                    if (cmd.kick_power > 0.001f) {
                        robotCmd->set_kick_speed(static_cast<float>(IBIS_MAX_KICK_SPEED * cmd.kick_power));
                        robotCmd->set_kick_angle(cmd.enable_chip ? static_cast<float>(IBIS_CHIP_ANGLE_DEG) : 0.0f);
                    }
                    if (cmd.dribble_power > 0.001f) {
                        // Match Amun's normalized 0..1 dribbler command conversion.
                        constexpr float kMaxDribblerSpeedRpm = static_cast<float>(150.0 * 60.0 * 0.5 / M_PI);
                        robotCmd->set_dribbler_speed(kMaxDribblerSpeedRpm * cmd.dribble_power);
                    }
                }

                if (ibisIsBlue) { hasBlue = true; } else { hasYellow = true; }
            }

            if (hasBlue) {
                emit sendRadioCommands(blueControl, true, start);
            }
            if (hasYellow) {
                emit sendRadioCommands(yellowControl, false, start);
            }
            warnLatency(m_timer->currentTime() - start);
        }
    }

private:
    // Logs at most once per second per robot, so a persistently misconfigured
    // chain produces a readable hint instead of a flood at the command rate.
    void warnUnsupportedMode(int robot_id, uint8_t mode) {
        constexpr qint64 kWarnIntervalNs = 1000LL * 1000LL * 1000LL;
        auto& state = m_robotStates[robot_id];
        const qint64 now = m_timer->currentTime();
        if (state.mode_warned && now - state.last_mode_warn_ns < kWarnIntervalNs) {
            return;
        }
        state.mode_warned = true;
        state.last_mode_warn_ns = now;
        if (mode == IBIS_MODE_POSITION_TARGET) {
            log(stdout,
                "ibis: robot %d sent POSITION_TARGET (mode %u), robot stopped. The simulator "
                "emulates the STM32 main board and does not close a position loop -- run the "
                "CM4 position controller (cm4_sim) between crane and the simulator. "
                "See docs/robot-side-position-control.md\n",
                robot_id, static_cast<unsigned>(mode));
        } else {
            log(stdout,
                "ibis: robot %d sent unsupported control mode %u, robot stopped "
                "(expected POLAR_VELOCITY_TARGET = %u)\n",
                robot_id, static_cast<unsigned>(mode),
                static_cast<unsigned>(IBIS_MODE_POLAR_VELOCITY_TARGET));
        }
    }

    // The command carries the sender's own estimate of where the robot is
    // (vision_global_pos). When it does not match any robot on the field the command
    // is dropped -- silently, until this warning was added. A silent drop is very hard
    // to tell apart from "the robot is commanded to hold still": the packets keep
    // arriving, check_counter keeps advancing, and nothing moves. Report the nearest
    // candidate so the reader can see whether the estimate is merely stale (slightly
    // over the threshold) or pointing somewhere else entirely.
    void warnPositionMismatch(int robot_id, const IbisCommand& cmd, double nearest) {
        constexpr qint64 kWarnIntervalNs = 1000LL * 1000LL * 1000LL;
        auto& state = m_robotStates[robot_id];
        const qint64 now = m_timer->currentTime();
        if (state.match_warned && now - state.last_match_warn_ns < kWarnIntervalNs) {
            return;
        }
        state.match_warned = true;
        state.last_match_warn_ns = now;
        if (nearest < 0.0) {
            log(stdout,
                "ibis: robot %d command dropped -- no robot with this id is on the field yet "
                "(command claims the robot is at %.3f, %.3f). Commands are ignored until "
                "vision reports the robot.\n",
                robot_id, cmd.vision_global_pos[0], cmd.vision_global_pos[1]);
        } else {
            log(stdout,
                "ibis: robot %d command dropped -- vision_global_pos (%.3f, %.3f) is %.3f m "
                "from the robot, over the %.2f m match threshold. The sender's position "
                "estimate disagrees with the simulator; the robot will not move until they "
                "agree. See docs/robot-side-position-control.md\n",
                robot_id, cmd.vision_global_pos[0], cmd.vision_global_pos[1],
                nearest, IBIS_POSITION_MATCH_THRESHOLD);
        }
    }

    static constexpr uint32_t kMaxRobots = 16;

    struct PerRobotState {
        double  prev_vx            = 0.0;
        double  prev_vy            = 0.0;
        uint8_t last_check_counter = 0xFF;
        qint64  last_mode_warn_ns  = 0;
        bool    mode_warned        = false;
        qint64  last_match_warn_ns = 0;
        bool    match_warned       = false;
    };

    // m_vision[0] = blue, m_vision[1] = yellow
    IbisVisionState m_vision[2][kMaxRobots]  = {};
    PerRobotState   m_robotStates[IBIS_ROBOT_SLOTS] = {};

    QUdpSocket m_server;
    Timer*     m_timer;
    double     m_accSpeedup;
    double     m_accBrake;
};

class RefereeTeamDetector : public QObject {
    Q_OBJECT
public:
    RefereeTeamDetector(const QString& teamName, bool localhost, quint16 port = SSL_GAME_CONTROLLER_PORT)
        : m_socket(this)
        , m_teamName(teamName.toLower().trimmed())
    {
        m_socket.bind(QHostAddress::AnyIPv4,
                      port,
                      QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
        if (!localhost) {
            m_socket.joinMulticastGroup(QHostAddress(SSL_GAME_CONTROLLER_ADDRESS));
        }
        connect(&m_socket, &QUdpSocket::readyRead, this, &RefereeTeamDetector::handleDatagrams);
    }

signals:
    void teamDetected(bool ibisIsBlue);

private slots:
    void handleDatagrams() {
        while (m_socket.hasPendingDatagrams()) {
            auto datagram = m_socket.receiveDatagram();
            SSL_Referee ref;
            if (!ref.ParseFromArray(datagram.data().data(), datagram.data().size())) {
                continue;
            }
            auto tryMatch = [&](const SSL_Referee::TeamInfo& info, bool isBlue) {
                if (!info.has_name()) { return false; }
                if (QString::fromStdString(info.name()).toLower().trimmed() != m_teamName) { return false; }
                disconnect(&m_socket, &QUdpSocket::readyRead, this, &RefereeTeamDetector::handleDatagrams);
                emit teamDetected(isBlue);
                return true;
            };
            if (ref.has_blue()   && tryMatch(ref.blue(),   true))  { return; }
            if (ref.has_yellow() && tryMatch(ref.yellow(), false)) { return; }
        }
    }

private:
    QUdpSocket m_socket;
    QString    m_teamName;
};

class IbisFeedbackAdaptor : public QObject {
    Q_OBJECT
public:
    // explicitColorSet: --ibis-team-color was given, so the colour is known up
    // front and the referee (if also enabled) may only correct it later.
    IbisFeedbackAdaptor(const QHostAddress& addr, quint16 portBase, bool useReferee,
                        bool explicitColorSet, bool explicitIsBlue)
        : m_sender(new PacketSenderThread())
        , m_addr(addr)
        , m_portBase(portBase)
        , m_ibisIsBlue(explicitColorSet ? explicitIsBlue : true)
        , m_refereeResolved(explicitColorSet || !useReferee)
    {}

    ~IbisFeedbackAdaptor() override {
        m_sender->stop();
        m_sender->wait();
        delete m_sender;
    }

public slots:
    // world::SimulatorState を受け取るたびに IbisVisionState を更新し、そのループの feedback を送信する（125Hz）
    // world::SimRobot の座標系: p_x/p_y はゲーム座標系のメートル単位（Bullet座標 / SIMULATOR_SCALE）
    // SSL座標系への変換: x_mm = p_y * 1000, y_mm = -p_x * 1000
    // 方向角: クォータニオン (i,j,k,real) の回転行列第1列から yaw を算出
    void handleGroundTruth(const QByteArray& data) {
        world::SimulatorState state;
        if (!state.ParseFromArray(data.data(), data.size())) {
            return;
        }
        auto updateTeam = [this](const auto& robots, int teamIdx) {
            for (const auto& r : robots) {
                const uint32_t id = r.id();
                if (id >= kMaxRobots) { continue; }
                const float qx = r.rotation().i();
                const float qy = r.rotation().j();
                const float qz = r.rotation().k();
                const float qw = r.rotation().real();
                const float dir_x = 1.0f - 2.0f * (qy*qy + qz*qz);
                const float dir_y = 2.0f * (qx*qy + qw*qz);
                m_vision[teamIdx][id] = {
                    r.p_y() * 1000.0f,
                    -r.p_x() * 1000.0f,
                    std::atan2(dir_y, dir_x),
                    true
                };
            }
        };
        updateTeam(state.blue_robots(),   0);
        updateTeam(state.yellow_robots(), 1);
        if (!m_refereeResolved) {
            // No feedback at all goes out until the colour is known. Say so:
            // a controller that closes its position loop on this feedback has
            // no position signal while this message is printing.
            if (++m_waitLogCount % 200 == 0) {
                log(stdout,
                    "ibis: NO FEEDBACK IS BEING SENT -- still waiting for the Game Controller "
                    "to identify team color. If the referee does not carry the team name, "
                    "pass --ibis-team-color blue|yellow instead of --ibis-use-referee.\n");
            }
            return;
        }

        const int teamIdx = m_ibisIsBlue ? 0 : 1;
        for (uint32_t id = 0; id < kMaxRobots; ++id) {
            const IbisVisionState& vis = m_vision[teamIdx][id];
            if (!vis.valid) { continue; }

            uint8_t buffer[IBIS_FEEDBACK_SIZE];
            ibisBuildFeedbackPacket(
                buffer,
                // Byte 3 echoes the AI command check counter on real hardware.
                // This adaptor does not see the command stream, so it sends a
                // free-running counter instead -- still usable for staleness
                // detection, but it does not correlate with a specific command.
                m_counters[id]++,
                m_txCycles[id]++,
                vis.orientation_rad,
                m_robotCache[id].ball_detected,
                0, // kick_status not tracked in ER-Force simulator
                vis.x_mm / 1000.0f,
                vis.y_mm / 1000.0f,
                m_robotCache[id].vel_x,
                m_robotCache[id].vel_y);

            m_sender->enqueue(
                QByteArray(reinterpret_cast<const char*>(buffer), IBIS_FEEDBACK_SIZE),
                m_addr,
                m_portBase + static_cast<quint16>(id));
        }
    }

    void handleRobotResponse(const QList<robot::RadioResponse>& responses) {
        if (!m_refereeResolved) {
            return;
        }

        const int teamIdx = m_ibisIsBlue ? 0 : 1;
        for (const auto& resp : responses) {
            if (!resp.has_is_blue() || resp.is_blue() != m_ibisIsBlue) { continue; }
            if (!resp.has_estimated_speed()) { continue; }
            const uint32_t id = resp.id();
            if (id >= kMaxRobots) { continue; }

            const IbisVisionState& vis = m_vision[teamIdx][id];
            if (!vis.valid) { continue; }

            // ロボットローカル速度（v_f=前方, v_s=左）をグローバルSSL座標に変換してキャッシュ
            const float theta = vis.orientation_rad;
            const float v_f = resp.estimated_speed().v_f();
            const float v_s = resp.estimated_speed().v_s();
            m_robotCache[id].vel_x = v_f * std::cos(theta) - v_s * std::sin(theta);
            m_robotCache[id].vel_y = v_f * std::sin(theta) + v_s * std::cos(theta);
            m_robotCache[id].ball_detected = resp.has_ball_detected() && resp.ball_detected();
        }
    }

    void handleRefereePacket(bool ibisIsBlue) {
        m_ibisIsBlue = ibisIsBlue;
        if (!m_refereeResolved) {
            m_refereeResolved = true;
            log(stdout, "ibis: team color resolved to %s from Game Controller\n",
                ibisIsBlue ? "BLUE" : "YELLOW");
        }
    }

private:
    static constexpr uint32_t kMaxRobots = 16;

    struct RobotCache {
        float vel_x = 0.0f;
        float vel_y = 0.0f;
        bool ball_detected = false;
    };

    // m_vision[0] = blue, m_vision[1] = yellow
    IbisVisionState m_vision[2][kMaxRobots] = {};
    RobotCache      m_robotCache[kMaxRobots] = {};
    uint8_t         m_counters[kMaxRobots]  = {};
    uint8_t         m_txCycles[kMaxRobots]  = {};
    int             m_waitLogCount          = 0;

    PacketSenderThread* m_sender;
    QHostAddress        m_addr;
    quint16             m_portBase;
    bool                m_ibisIsBlue;
    bool                m_refereeResolved;
};

#include "simulator.moc"



int main(int argc, char* argv[])
{
    // Line-buffer the logs. stdout is block-buffered when it is a pipe or a file --
    // which is how this runs under Docker, systemd, or a test harness -- and log()
    // never flushes, so a SIGTERM discards everything written since the last 4 KiB
    // boundary. In practice that means the log is empty exactly when something went
    // wrong and someone goes looking for it: a container stopped after a failed run
    // has produced zero lines here. Do it in the process rather than leaving it to
    // the caller to remember `stdbuf -oL`, which the published image's own entrypoint
    // does not do.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::setvbuf(stderr, nullptr, _IOLBF, 0);

    QCoreApplication app(argc, argv);
    app.setApplicationName("Simulator");
    app.setOrganizationName("ER-Force");

    std::setlocale(LC_NUMERIC, "C");

    qRegisterMetaType<QList<robot::RadioResponse>>("QList<robot::RadioResponse>");
    qRegisterMetaType<Status>("Status");
    qRegisterMetaType<Command>("Command");
    qRegisterMetaType<SSLSimRobotControl>("SSLSimRobotControl");
    qRegisterMetaType<SSLSimError>("SSLSimError");
    qRegisterMetaType<QList<SSLSimError>>("QList<SSLSimError>");
    qRegisterMetaType<camun::simulator::ErrorSource>("ErrorSource");

    QCommandLineParser parser;
    parser.setApplicationDescription("ER-Force simulator command line interface");
    parser.addHelpOption();

    QCommandLineOption geometryConfig({"g", "geometry"}, "The geometry file to load as default", "file", "2020");
    QCommandLineOption realismConfig("realism", "Simulator realism configuration (short file name without the .txt)", "realism", "Ibis");
    QCommandLineOption localhostConfig("localhost", "Use localhost as the output address for the simulator");
    parser.addOption(geometryConfig);
    parser.addOption(realismConfig);
    parser.addOption(localhostConfig);

    // ibis binary protocol options (always enabled on the ibis branch)
    QCommandLineOption ibisPortOpt("ibis-port", "ibis command receiver UDP port", "port", QString::number(IBIS_DEFAULT_PORT));
    QCommandLineOption ibisFeedbackAddrOpt("ibis-feedback-addr", "ibis feedback destination address", "addr", "127.0.0.1");
    QCommandLineOption ibisFeedbackPortBaseOpt("ibis-feedback-port-base", "ibis feedback base port (robotId is added)", "port", QString::number(IBIS_FEEDBACK_PORT_BASE));
    QCommandLineOption ibisFeedbackTeamNameOpt("ibis-feedback-team-name", "Team name to look up in Game Controller for color detection", "name", "ibis");
    QCommandLineOption ibisUseRefereeOpt("ibis-use-referee", "Use Game Controller referee to auto-detect ibis team color");
    QCommandLineOption ibisTeamColorOpt("ibis-team-color", "Set the ibis team color explicitly (blue|yellow), instead of detecting it from the Game Controller", "color", "");
    QCommandLineOption ibisAccSpeedupOpt("ibis-acc-speedup", "Acceleration limit for speedup [m/s^2]", "accel", "4.0");
    QCommandLineOption ibisAccBrakeOpt("ibis-acc-brake", "Acceleration limit for braking [m/s^2]", "accel", "6.0");
    QCommandLineOption ibisRefereePortOpt("ibis-referee-port", "Game Controller multicast port for team color detection", "port", QString::number(SSL_GAME_CONTROLLER_PORT));
    parser.addOption(ibisPortOpt);
    parser.addOption(ibisFeedbackAddrOpt);
    parser.addOption(ibisFeedbackPortBaseOpt);
    parser.addOption(ibisFeedbackTeamNameOpt);
    parser.addOption(ibisUseRefereeOpt);
    parser.addOption(ibisTeamColorOpt);
    parser.addOption(ibisAccSpeedupOpt);
    parser.addOption(ibisAccBrakeOpt);
    parser.addOption(ibisRefereePortOpt);

    parser.process(app);

    auto* desc = sslsim::RobotSpecs::descriptor();
    int real_fields = desc->field_count();
    auto* desc2 = sslsim::RobotSpecErForce::descriptor();
    real_fields += desc2->field_count();


    if (real_fields != expected_specs_fields) {
        std::string msg = "BUG: The number of fields for the specs has a different number compared to expected!";
        msg += " expected: " + std::to_string(expected_specs_fields);
        msg += ", but found: " + std::to_string(real_fields);
        msg += " (in ";
        msg += __FILE__;
        msg += ": ";
        msg += std::to_string(functionToFixForSpecs);
        log(stdout, "%s)", msg.c_str());
    }

    Timer timer;
    RobotCommandAdaptor blue{true, &timer}, yellow{false, &timer};
    SimProxy sim{&timer};
    SSLVisionServer vision{SSL_SIMULATED_VISION_PORT, parser.isSet(localhostConfig) ? SSL_VISION_ADDRESS_LOCALHOST : SSL_VISION_ADDRESS};
    SimulatorCommandAdaptor commands{&timer, &vision};

    blue.connect(&blue, &RobotCommandAdaptor::sendRadioCommands, &sim, &SimProxy::handleRadioCommands);
    blue.connect(&sim, &SimProxy::sendRadioResponses, &blue, &RobotCommandAdaptor::handleRobotResponse);
    yellow.connect(&yellow, &RobotCommandAdaptor::sendRadioCommands, &sim, &SimProxy::handleRadioCommands);
    yellow.connect(&sim, &SimProxy::sendRadioResponses, &yellow, &RobotCommandAdaptor::handleRobotResponse);


    vision.connect(&sim, &SimProxy::gotPacket, &vision, &SSLVisionServer::sendVisionData);
    commands.connect(&commands, &SimulatorCommandAdaptor::sendCommand, &sim, &SimProxy::handleCommand);


    commands.connect(&sim, &SimProxy::sendSSLSimError, &commands, &SimulatorCommandAdaptor::handleSimulatorError);
    blue.connect(&sim, &SimProxy::sendSSLSimError, &blue, &RobotCommandAdaptor::handleSimulatorError);
    yellow.connect(&sim, &SimProxy::sendSSLSimError, &yellow, &RobotCommandAdaptor::handleSimulatorError);

    Command c{new amun::Command};

    // start with default robots, take ER-Force specs.
    robot::Specs ERForce;
    robotSetDefault(&ERForce);

    auto* teamBlue = c->mutable_set_team_blue();
    auto* teamYellow = c->mutable_set_team_yellow();
    for(auto* team : {teamBlue, teamYellow}) {
        for(int i=0; i < 11; ++i){
            auto* robot = team->add_robot();
            robot->CopyFrom(ERForce);
            robot->set_id(i);
        }
    }

    if (!loadConfiguration("simulator/" + parser.value(geometryConfig), c->mutable_simulator()->mutable_simulator_setup(), false)) {
        exit(EXIT_FAILURE);
    }
    if (!loadConfiguration("simulator-realism/" + parser.value(realismConfig), c->mutable_simulator()->mutable_realism_config(), true)) {
        exit(EXIT_FAILURE);
    }

    emit commands.sendCommand(c);

    QThread rcv_thread;

    blue.moveToThread(&rcv_thread);
    yellow.moveToThread(&rcv_thread);
    vision.moveToThread(&rcv_thread);
    commands.moveToThread(&rcv_thread);

    // ibis binary protocol components (always enabled on the ibis branch)
    {
        const int cmdPort           = parser.value(ibisPortOpt).toInt();
        const double accSpeedup     = parser.value(ibisAccSpeedupOpt).toDouble();
        const double accBrake       = parser.value(ibisAccBrakeOpt).toDouble();
        const QHostAddress fbAddr   = QHostAddress(parser.value(ibisFeedbackAddrOpt));
        const quint16 fbPortBase    = static_cast<quint16>(parser.value(ibisFeedbackPortBaseOpt).toUInt());
        const bool useReferee       = parser.isSet(ibisUseRefereeOpt);

        // Team colour selects which team's ground truth the feedback carries.
        // Getting it wrong is silent and nasty: the feedback still flows, but
        // it describes the opponent's robots, so anything closing a position
        // loop on it steers on the wrong positions.
        const QString teamColorStr = parser.value(ibisTeamColorOpt).trimmed().toLower();
        bool explicitColorSet = false;
        bool explicitIsBlue   = true;
        if (!teamColorStr.isEmpty()) {
            if (teamColorStr == "blue") {
                explicitColorSet = true; explicitIsBlue = true;
            } else if (teamColorStr == "yellow") {
                explicitColorSet = true; explicitIsBlue = false;
            } else {
                log(stdout, "ibis: unknown --ibis-team-color '%s', expected blue or yellow\n",
                    teamColorStr.toStdString().c_str());
                return 1;
            }
        }
        if (explicitColorSet) {
            log(stdout, "ibis: team color set to %s by --ibis-team-color\n",
                explicitIsBlue ? "BLUE" : "YELLOW");
        } else if (useReferee) {
            log(stdout, "ibis: team color will be detected from the Game Controller "
                        "(team name '%s'); no feedback is sent until it resolves\n",
                parser.value(ibisFeedbackTeamNameOpt).toStdString().c_str());
        } else {
            log(stdout, "ibis: WARNING no team color given and referee detection is off -- "
                        "assuming BLUE. If ibis plays yellow, the feedback will carry the "
                        "opponent's positions. Pass --ibis-team-color blue|yellow.\n");
        }
        const quint16 refereePort   = static_cast<quint16>(parser.value(ibisRefereePortOpt).toUInt());

        auto* ibisCmd = new IbisCommandAdaptor(cmdPort, &timer, accSpeedup, accBrake);
        auto* ibisFb  = new IbisFeedbackAdaptor(fbAddr, fbPortBase, useReferee,
                                               explicitColorSet, explicitIsBlue);

        // IbisCommandAdaptor receives vision data to cache robot positions/orientations
        QObject::connect(&sim, &SimProxy::gotPacket,
                         ibisCmd, &IbisCommandAdaptor::handleVisionData);
        // IbisCommandAdaptor sends converted commands to the simulator
        QObject::connect(ibisCmd, &IbisCommandAdaptor::sendRadioCommands,
                         &sim, &SimProxy::handleRadioCommands);

        // IbisFeedbackAdaptor receives ground truth positions at 125Hz and emits one feedback packet per loop
        QObject::connect(&sim, &SimProxy::sendGroundTruth,
                         ibisFb, &IbisFeedbackAdaptor::handleGroundTruth);
        // IbisFeedbackAdaptor receives radio responses for velocity and ball detection
        QObject::connect(&sim, &SimProxy::sendRadioResponses,
                         ibisFb, &IbisFeedbackAdaptor::handleRobotResponse);

        if (useReferee) {
            auto* referee = new RefereeTeamDetector(
                parser.value(ibisFeedbackTeamNameOpt),
                parser.isSet(localhostConfig),
                refereePort);
            QObject::connect(referee, &RefereeTeamDetector::teamDetected,
                             ibisFb, &IbisFeedbackAdaptor::handleRefereePacket);
            referee->moveToThread(&rcv_thread);
        }

        ibisCmd->moveToThread(&rcv_thread);
        ibisFb->moveToThread(&rcv_thread);

        log(stdout, "ibis: command receiver on UDP port %d\n", cmdPort);
        log(stdout, "ibis: feedback sender to %s base port %d synchronized to simulator loop (125 Hz)\n",
            parser.value(ibisFeedbackAddrOpt).toStdString().c_str(),
            static_cast<int>(fbPortBase));
    }

    rcv_thread.start();

    return app.exec();
}
