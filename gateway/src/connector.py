#!/bin/python
# -*- encoding:utf-8-*-

from twisted.internet import reactor, protocol
from common import Event, EventType
from enum import Enum


class _ConnectorState(Enum):
    NEW = "new"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"


class LineReceiver(protocol.Protocol):
    def __init__(self):
        self._buffer = b''
        self.delimiter = b'\r\n'

    def dataReceived(self, data):
        lines = (self._buffer + data).split(self.delimiter)
        self._buffer = lines.pop(-1)
        for line in lines:
            if self.transport.disconnecting:
                return
            self.lineReceived(line)

    def lineReceived(self, line):
        raise NotImplementedError

    def sendLine(self, line):
        return self.transport.writeSequence((line, self.delimiter))


class Connector(protocol.ReconnectingClientFactory, LineReceiver):
    def __init__(self, event_bus, host, port):
        LineReceiver.__init__(self)
        self.__event_bus = event_bus
        self.__host = host
        self.__port = port
        self.__state = _ConnectorState.NEW
        self.__reactor_thread = None

    def buildProtocol(self, addr):
        return self

    def connectionMade(self):
        self.__state = _ConnectorState.CONNECTED
        self.resetDelay()
        self.transport.setTcpNoDelay(True)
        self.__event_bus.put(Event(EventType.ConnectionMadeEvent))

    def lineReceived(self, line):
        self.__event_bus.put(Event(EventType.DataReceivedEvent, line))

    def clientConnectionFailed(self, connector, reason):
        self.__state = _ConnectorState.DISCONNECTED
        self.__event_bus.put(Event(EventType.ConnectionDisconnectedEvent))
        protocol.ReconnectingClientFactory.clientConnectionFailed(
            self, connector, reason)

    def clientConnectionLost(self, connector, reason):
        self.__state = _ConnectorState.DISCONNECTED
        print "clientConnectionLost, reason: {}".format(reason)
        self.__event_bus.put(Event(EventType.ConnectionDisconnectedEvent))
        protocol.ReconnectingClientFactory.clientConnectionLost(
            self, connector, reason)

    def start(self):
        reactor.connectTCP(self.__host, self.__port, self)
        import thread
        self.__reactor_thread = thread.start_new_thread(reactor.run, (False, ))

    def shutdown(self):
        if self.transport:
            self.transport.loseConnection()
        reactor.callFromThread(reactor.stop)

    def _write_thread_safe(self, msg):
        self.transport.write(msg.encode("utf-8"))

    def write(self, data):
        if self.__state == _ConnectorState.DISCONNECTED:
            return False
        reactor.callFromThread(self._write_thread_safe, data + "\r\n")
        # reactor.callFromThread(self.sendLine, data)
        return True

    def isConnected(self):
        return self.__state == _ConnectorState.CONNECTED

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
