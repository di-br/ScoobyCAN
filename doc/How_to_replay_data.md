# How to replay recorded CAN frames...
**and use them with ScoobyCAN**

Prerequisites are [SocketCAN](https://en.wikipedia.org/wiki/SocketCAN) [userspace utilities and tools](https://github.com/linux-can/can-utils).

1. Assume you have dumped live data from your car with SocketCAN using `candump`.  
   An example dump can be found in the [repo](https://github.com/di-br/ScoobyCAN/blob/master/examples/candump.log).

2. Next, get your system ready for replaying the frames
```bash
# load necessary kernel modules
modprobe can
modprobe can_raw
modprobe vcan
# add the networking device
ip link add dev vcan0 type vcan
# bring device up
ip link set up vcan0
# show it's settings
ip link show vcan0
```

3. Note the interface used while recording (kept in the log, in the example `slcan0`) and replay CAN frames
```bash
canplayer -I candump.log vcan0=slcan0
```

4. Enjoy frames flying by, e.g. via `cansniffer`
```bash
cansniffer vcan0
```

5. Finally 'attach' ScoobyCAN
```bash
# TUI
ScoobyCAN vcan0
# dumping 'data' to command line
ScoobyCAN_dump vcan0
```
