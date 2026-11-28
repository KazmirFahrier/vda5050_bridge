# A recorded change of a fault. Events are numbered; the numbers only grow,
# also across restarts of the fault manager.

uint8 TYPE_ACTIVATED = 0
uint8 TYPE_CLEARED = 1
uint8 TYPE_ACKNOWLEDGED = 2

uint64 sequence
uint8 type
string code
# One of the SEVERITY_ constants of Fault.
uint8 severity
builtin_interfaces/Time stamp
string message
string source
uint64 occurrence_count
