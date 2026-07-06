# Compact task lifecycle state.

uint8 CREATED=1
uint8 PENDING=2
uint8 ASSIGNED=3
uint8 RUNNING=4
uint8 WAITING_CONFIRMATION=5
uint8 PAUSED=6
uint8 RESUMING=7
uint8 SUCCEEDED=8
uint8 FAILED=9
uint8 CANCELED=10
uint8 WAITING_RESOURCE=11

uint8 state
builtin_interfaces/Time stamp
string reason
