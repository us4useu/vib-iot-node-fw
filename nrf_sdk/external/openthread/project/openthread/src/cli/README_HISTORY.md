# OpenThread CLI - History Tracker

History Tracker module records history of different events (e.g., RX and TX IPv6 messages or network info changes, etc.) as the Thread network operates. All tracked entries are timestamped.

All commands under `history` require `OPENTHREAD_CONFIG_HISTORY_TRACKER_ENABLE` feature to be enabled.

The number of entries recorded for each history list is configurable through a set of OpenThread config options, e.g. `OPENTHREAD_CONFIG_HISTORY_TRACKER_NET_INFO_LIST_SIZE` specifies the number of entries in Network Info history list. The History Tracker will keep the most recent entries overwriting oldest one when the list gets full.

## Command List

Usage : `history [command] ...`

- [help](#help)
- [netinfo](#netinfo)
- [rx](#rx)
- [rxtx](#rxtx)
- [tx](#tx)

## Timestamp Format

Recorded entries are timestamped. When the history list is printed, the timestamps are shown relative the time the command was issues (i.e., when the list was printed) indicating how long ago the entry was recorded.

```bash
> history netinfo
| Age                  | Role     | Mode | RLOC16 | Partition ID |
+----------------------+----------+------+--------+--------------+
|         02:31:50.628 | leader   | rdn  | 0x2000 |    151029327 |
|         02:31:53.262 | detached | rdn  | 0xfffe |            0 |
|         02:31:54.663 | detached | rdn  | 0x2000 |            0 |
Done
```

For example `02:31:50.628` indicates the event was recorded "2 hours, 31 minutes, 50 seconds, and 628 milliseconds ago". Number of days is added for events that are older than 24 hours, e.g., `1 day 11:25:31.179`, or `31 days 03:00:23.931`.

Timestamps use millisecond accuracy and are tacked up to 49 days. If the event is older than 49 days, the entry is still tracked in the list but the timestamp is shown as `more than 49 days`.

## Command Details

### help

Usage: `history help`

Print SRP client help menu.

```bash
> history help
help
netinfo
rx
rxtx
tx
Done
>
```

### netinfo

Usage `history netinfo [list] [<num-entries>]`

Print the Network Info history. Each Network Info provides

- Device Role
- MLE Link Mode
- RLOC16
- Partition ID

Print the Network Info history as a table.

```bash
> history netinfo
| Age                  | Role     | Mode | RLOC16 | Partition ID |
+----------------------+----------+------+--------+--------------+
|         00:00:10.069 | router   | rdn  | 0x6000 |    151029327 |
|         00:02:09.337 | child    | rdn  | 0x2001 |    151029327 |
|         00:02:09.338 | child    | rdn  | 0x2001 |    151029327 |
|         00:07:40.806 | child    | -    | 0x2001 |    151029327 |
|         00:07:42.297 | detached | -    | 0x6000 |            0 |
|         00:07:42.968 | disabled | -    | 0x6000 |            0 |
Done
```

Print the Network Info history as a list.

```bash
> history netinfo list
00:00:59.467 -> role:router mode:rdn rloc16:0x6000 partition-id:151029327
00:02:58.735 -> role:child mode:rdn rloc16:0x2001 partition-id:151029327
00:02:58.736 -> role:child mode:rdn rloc16:0x2001 partition-id:151029327
00:08:30.204 -> role:child mode:- rloc16:0x2001 partition-id:151029327
00:08:31.695 -> role:detached mode:- rloc16:0x6000 partition-id:0
00:08:32.366 -> role:disabled mode:- rloc16:0x6000 partition-id:0
Done
```

Print only the latest 2 entries.

```bash
> history netinfo 2
| Age                  | Role     | Mode | RLOC16 | Partition ID |
+----------------------+----------+------+--------+--------------+
|         00:02:05.451 | router   | rdn  | 0x6000 |    151029327 |
|         00:04:04.719 | child    | rdn  | 0x2001 |    151029327 |
Done
```

### rx

Usage `history rx [list] [<num-entries>]`

Print the IPv6 message RX history in either table or list format. Entries provide same information and follow same format as in `history rxtx` command.

Print the IPv6 message RX history as a table:

```bash
> history rx
| Age                  | Type             | Len   | Chksum | Sec | Prio | RSS  |Dir | Neighb | Radio |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0xbd26 |  no |  net |  -20 | RX | 0x4800 |  15.4 |
|         00:00:07.640 | src: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | HopOpts          |    44 | 0x0000 | yes | norm |  -20 | RX | 0x4800 |  15.4 |
|         00:00:09.263 | src: [fdde:ad00:beef:0:0:ff:fe00:4800]:0                                    |
|                      | dst: [ff03:0:0:0:0:0:0:2]:0                                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    12 | 0x3f7d | yes |  net |  -20 | RX | 0x4800 |  15.4 |
|         00:00:09.302 | src: [fdde:ad00:beef:0:0:ff:fe00:4800]:61631                                |
|                      | dst: [fdde:ad00:beef:0:0:ff:fe00:4801]:61631                                |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | ICMP6(EchoReqst) |    16 | 0x942c | yes | norm |  -20 | RX | 0x4800 |  15.4 |
|         00:00:09.304 | src: [fdde:ad00:beef:0:ac09:a16b:3204:dc09]:0                               |
|                      | dst: [fdde:ad00:beef:0:dc0e:d6b3:f180:b75b]:0                               |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | HopOpts          |    44 | 0x0000 | yes | norm |  -20 | RX | 0x4800 |  15.4 |
|         00:00:09.304 | src: [fdde:ad00:beef:0:0:ff:fe00:4800]:0                                    |
|                      | dst: [ff03:0:0:0:0:0:0:2]:0                                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0x2e37 |  no |  net |  -20 | RX | 0x4800 |  15.4 |
|         00:00:21.622 | src: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0xe177 |  no |  net |  -20 | RX | 0x4800 |  15.4 |
|         00:00:26.640 | src: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |   165 | 0x82ee | yes |  net |  -20 | RX | 0x4800 |  15.4 |
|         00:00:30.000 | src: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
|                      | dst: [fe80:0:0:0:a4a5:bbac:a8e:bd07]:19788                                  |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    93 | 0x52df |  no |  net |  -20 | RX | unknwn |  15.4 |
|         00:00:30.480 | src: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
|                      | dst: [fe80:0:0:0:a4a5:bbac:a8e:bd07]:19788                                  |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0x5ccf |  no |  net |  -20 | RX | unknwn |  15.4 |
|         00:00:30.772 | src: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
Done

```

Print the latest 5 entries of the IPv6 message RX history as a list:

```bash
> history rx list 4
00:00:13.368
    type:UDP len:50 cheksum:0xbd26 sec:no prio:net rss:-20 from:0x4800 radio:15.4
    src:[fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788
    dst:[ff02:0:0:0:0:0:0:1]:19788
00:00:14.991
    type:HopOpts len:44 cheksum:0x0000 sec:yes prio:norm rss:-20 from:0x4800 radio:15.4
    src:[fdde:ad00:beef:0:0:ff:fe00:4800]:0
    dst:[ff03:0:0:0:0:0:0:2]:0
00:00:15.030
    type:UDP len:12 cheksum:0x3f7d sec:yes prio:net rss:-20 from:0x4800 radio:15.4
    src:[fdde:ad00:beef:0:0:ff:fe00:4800]:61631
    dst:[fdde:ad00:beef:0:0:ff:fe00:4801]:61631
00:00:15.032
    type:ICMP6(EchoReqst) len:16 cheksum:0x942c sec:yes prio:norm rss:-20 from:0x4800 radio:15.4
    src:[fdde:ad00:beef:0:ac09:a16b:3204:dc09]:0
    dst:[fdde:ad00:beef:0:dc0e:d6b3:f180:b75b]:0
Done
```

### rxtx

Usage `history rxtx [list] [<num-entries>]`

Print the combined IPv6 message RX and TX history in either table or list format. Each entry provides:

- IPv6 message type: UDP, TCP, ICMP6 (and its subtype), etc.
- IPv6 payload length (excludes the IPv6 header).
- Source IPv6 address and port number.
- Destination IPv6 address and port number (port number is valid for UDP/TCP, it is zero otherwise).
- Whether or not link-layer security was used.
- Message priority: low, norm, high, net (for Thread control messages).
- Message checksum (valid for UDP, TCP, or ICMP6 message)
- RSS: Received Signal Strength (in dBm) - averaged over all received fragment frames that formed the message. For TX history `NA` (not applicable) is used.
- Whether the message was sent or received (`TX` or `RX`). A failed transmission (e.g., if tx was aborted or no ack from peer for any of the message fragments) is indicated with `TX-F` in the table format or `tx-success:no` in the list format.
- Short address (RLOC16) of neighbor to/from which the message was sent/received. If the frame is broadcast, it is shown as `bcast` in table format or `0xffff` in the list format. If the short address of neighbor is not available, it is shown as `unknwn` in the table format or `0xfffe` in the list format.
- Radio link on which the message was sent/received (useful when `OPENTHREAD_CONFIG_MULTI_RADIO` is enabled). Can be `15.4`, `trel`, or `all` (if sent on all radio links).

Print the IPv6 message RX and TX history as a table:

```bash
> history rxtx
| Age                  | Type             | Len   | Chksum | Sec | Prio | RSS  |Dir | Neighb | Radio |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | HopOpts          |    44 | 0x0000 | yes | norm |  -20 | RX | 0x0800 |  15.4 |
|         00:00:09.267 | src: [fdde:ad00:beef:0:0:ff:fe00:800]:0                                     |
|                      | dst: [ff03:0:0:0:0:0:0:2]:0                                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    12 | 0x6c6b | yes |  net |  -20 | RX | 0x0800 |  15.4 |
|         00:00:09.290 | src: [fdde:ad00:beef:0:0:ff:fe00:800]:61631                                 |
|                      | dst: [fdde:ad00:beef:0:0:ff:fe00:801]:61631                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | ICMP6(EchoReqst) |    16 | 0xc6a2 | yes | norm |  -20 | RX | 0x0800 |  15.4 |
|         00:00:09.292 | src: [fdde:ad00:beef:0:efe8:4910:cf95:dee9]:0                               |
|                      | dst: [fdde:ad00:beef:0:af4c:3644:882a:3698]:0                               |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | ICMP6(EchoReply) |    16 | 0xc5a2 | yes | norm |  NA  | TX | 0x0800 |  15.4 |
|         00:00:09.292 | src: [fdde:ad00:beef:0:af4c:3644:882a:3698]:0                               |
|                      | dst: [fdde:ad00:beef:0:efe8:4910:cf95:dee9]:0                               |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0xaa0d | yes |  net |  NA  | TX | 0x0800 |  15.4 |
|         00:00:09.294 | src: [fdde:ad00:beef:0:0:ff:fe00:801]:61631                                 |
|                      | dst: [fdde:ad00:beef:0:0:ff:fe00:800]:61631                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | HopOpts          |    44 | 0x0000 | yes | norm |  -20 | RX | 0x0800 |  15.4 |
|         00:00:09.296 | src: [fdde:ad00:beef:0:0:ff:fe00:800]:0                                     |
|                      | dst: [ff03:0:0:0:0:0:0:2]:0                                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0xc1d8 |  no |  net |  -20 | RX | 0x0800 |  15.4 |
|         00:00:09.569 | src: [fe80:0:0:0:54d9:5153:ffc6:df26]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0x3cb1 |  no |  net |  -20 | RX | 0x0800 |  15.4 |
|         00:00:16.519 | src: [fe80:0:0:0:54d9:5153:ffc6:df26]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0xeda0 |  no |  net |  -20 | RX | 0x0800 |  15.4 |
|         00:00:20.599 | src: [fe80:0:0:0:54d9:5153:ffc6:df26]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:1]:19788                                             |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |   165 | 0xbdfa | yes |  net |  -20 | RX | 0x0800 |  15.4 |
|         00:00:21.059 | src: [fe80:0:0:0:54d9:5153:ffc6:df26]:19788                                 |
|                      | dst: [fe80:0:0:0:8893:c2cc:d983:1e1c]:19788                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    64 | 0x1c11 |  no |  net |  NA  | TX | 0x0800 |  15.4 |
|         00:00:21.062 | src: [fe80:0:0:0:8893:c2cc:d983:1e1c]:19788                                 |
|                      | dst: [fe80:0:0:0:54d9:5153:ffc6:df26]:19788                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    93 | 0xedff |  no |  net |  -20 | RX | unknwn |  15.4 |
|         00:00:21.474 | src: [fe80:0:0:0:54d9:5153:ffc6:df26]:19788                                 |
|                      | dst: [fe80:0:0:0:8893:c2cc:d983:1e1c]:19788                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    44 | 0xd383 |  no |  net |  NA  | TX | bcast  |  15.4 |
|         00:00:21.811 | src: [fe80:0:0:0:8893:c2cc:d983:1e1c]:19788                                 |
|                      | dst: [ff02:0:0:0:0:0:0:2]:19788                                             |
Done
```

Print the latest 5 entries of the IPv6 message RX history as a list:

```bash
> history rxtx list 5

00:00:02.100
    type:UDP len:50 cheksum:0xd843 sec:no prio:net rss:-20 from:0x0800 radio:15.4
    src:[fe80:0:0:0:54d9:5153:ffc6:df26]:19788
    dst:[ff02:0:0:0:0:0:0:1]:19788
00:00:15.331
    type:HopOpts len:44 cheksum:0x0000 sec:yes prio:norm rss:-20 from:0x0800 radio:15.4
    src:[fdde:ad00:beef:0:0:ff:fe00:800]:0
    dst:[ff03:0:0:0:0:0:0:2]:0
00:00:15.354
    type:UDP len:12 cheksum:0x6c6b sec:yes prio:net rss:-20 from:0x0800 radio:15.4
    src:[fdde:ad00:beef:0:0:ff:fe00:800]:61631
    dst:[fdde:ad00:beef:0:0:ff:fe00:801]:61631
00:00:15.356
    type:ICMP6(EchoReqst) len:16 cheksum:0xc6a2 sec:yes prio:norm rss:-20 from:0x0800 radio:15.4
    src:[fdde:ad00:beef:0:efe8:4910:cf95:dee9]:0
    dst:[fdde:ad00:beef:0:af4c:3644:882a:3698]:0
00:00:15.356
    type:ICMP6(EchoReply) len:16 cheksum:0xc5a2 sec:yes prio:norm tx-success:yes to:0x0800 radio:15.4
    src:[fdde:ad00:beef:0:af4c:3644:882a:3698]:0
    dst:[fdde:ad00:beef:0:efe8:4910:cf95:dee9]:0
```

### tx

Usage `history tx [list] [<num-entries>]`

Print the IPv6 message TX history in either table or list format. Entries provide same information and follow same format as in `history rxtx` command.

Print the IPv6 message TX history as a table (10 latest entries):

```bash
> history tx
| Age                  | Type             | Len   | Chksum | Sec | Prio | RSS  |Dir | Neighb | Radio |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | ICMP6(EchoReply) |    16 | 0x932c | yes | norm |  NA  | TX | 0x4800 |  15.4 |
|         00:00:18.798 | src: [fdde:ad00:beef:0:dc0e:d6b3:f180:b75b]:0                               |
|                      | dst: [fdde:ad00:beef:0:ac09:a16b:3204:dc09]:0                               |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    50 | 0xce87 | yes |  net |  NA  | TX | 0x4800 |  15.4 |
|         00:00:18.800 | src: [fdde:ad00:beef:0:0:ff:fe00:4801]:61631                                |
|                      | dst: [fdde:ad00:beef:0:0:ff:fe00:4800]:61631                                |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    64 | 0xf7ba |  no |  net |  NA  | TX | 0x4800 |  15.4 |
|         00:00:39.499 | src: [fe80:0:0:0:a4a5:bbac:a8e:bd07]:19788                                  |
|                      | dst: [fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788                                 |
+----------------------+------------------+-------+--------+-----+------+------+----+--------+-------+
|                      | UDP              |    44 | 0x26d4 |  no |  net |  NA  | TX | bcast  |  15.4 |
|         00:00:40.256 | src: [fe80:0:0:0:a4a5:bbac:a8e:bd07]:19788                                  |
|                      | dst: [ff02:0:0:0:0:0:0:2]:19788                                             |
Done
```

Print the IPv6 message TX history as a list:

```bash
history tx list
00:00:23.957
    type:ICMP6(EchoReply) len:16 cheksum:0x932c sec:yes prio:norm tx-success:yes to:0x4800 radio:15.4
    src:[fdde:ad00:beef:0:dc0e:d6b3:f180:b75b]:0
    dst:[fdde:ad00:beef:0:ac09:a16b:3204:dc09]:0
00:00:23.959
    type:UDP len:50 cheksum:0xce87 sec:yes prio:net tx-success:yes to:0x4800 radio:15.4
    src:[fdde:ad00:beef:0:0:ff:fe00:4801]:61631
    dst:[fdde:ad00:beef:0:0:ff:fe00:4800]:61631
00:00:44.658
    type:UDP len:64 cheksum:0xf7ba sec:no prio:net tx-success:yes to:0x4800 radio:15.4
    src:[fe80:0:0:0:a4a5:bbac:a8e:bd07]:19788
    dst:[fe80:0:0:0:d03d:d3e7:cc5e:7cd7]:19788
00:00:45.415
    type:UDP len:44 cheksum:0x26d4 sec:no prio:net tx-success:yes to:0xffff radio:15.4
    src:[fe80:0:0:0:a4a5:bbac:a8e:bd07]:19788
    dst:[ff02:0:0:0:0:0:0:2]:19788
Done
```
