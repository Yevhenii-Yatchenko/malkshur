(echo "move,1500,1500,1500" && sleep 1) | telnet 192.168.88.251  2323
(echo "arm,0" && sleep 1) | telnet 192.168.88.251  2323
(echo "arm,1" && sleep 1) | telnet 192.168.88.251  2323

(echo "setHeight,1.5" && sleep 1) | telnet 192.168.88.251  2323
(echo "land,1.5" && sleep 1) | telnet 192.168.88.251  2323


# roll pitch yaw
# ліво
(echo "move,1485,1500,1500" && sleep 1) | telnet 192.168.88.251  2323
# право
(echo "move,1515,1500,1500" && sleep 1) | telnet 192.168.88.251  2323


# назад
(echo "move,1500,1485,1500" && sleep 1) | telnet 192.168.88.251  2323
# вперед
(echo "move,1500,1515,1500" && sleep 1) | telnet 192.168.88.251  2323


# roll+
(echo "move,1510,1500,1500" && sleep 1) | telnet 192.168.88.251  2323
# roll-
(echo "move,1490,1500,1500" && sleep 1) | telnet 192.168.88.251  2323
# pitch+
(echo "move,1500,1510,1500" && sleep 1) | telnet 192.168.88.251  2323
# pitch-
(echo "move,1500,1490,1500" && sleep 1) | telnet 192.168.88.251  2323

# roll+ pitch+
(echo "move,1510,1510,1500" && sleep 1) | telnet 192.168.88.251  2323
# roll- pitch+
(echo "move,1490,1510,1500" && sleep 1) | telnet 192.168.88.251  2323
# roll+ pitch-
(echo "move,1510,1490,1500" && sleep 1) | telnet 192.168.88.251  2323
# roll- pitch-
(echo "move,1490,1490,1500" && sleep 1) | telnet 192.168.88.251  2323


(echo "setHeight,1.5" && sleep 1) | telnet 192.168.88.251  2323
(echo "stabilize,1.5" && sleep 1) | telnet 192.168.88.251  2323


(echo "arm,0" && sleep 1) | telnet 192.168.1.40  2323
(echo "setHeight,3" && sleep 1) | telnet 192.168.1.40  2323
(echo "stabilize,1.5" && sleep 1) | telnet 192.168.1.40  2323



(echo "arm,0" && sleep 1) | telnet localhost  2323
(echo "setHeight,3" && sleep 1) | telnet localhost  2323
(echo "stabilize,1.5" && sleep 1) | telnet localhost  2323