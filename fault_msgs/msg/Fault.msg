# The current state of one fault.

uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARNING = 1
uint8 SEVERITY_ERROR = 2
uint8 SEVERITY_CRITICAL = 3

uint8 STATE_INACTIVE = 0
# Failing, but not yet for long enough to count.
uint8 STATE_PENDING = 1
uint8 STATE_ACTIVE = 2
# Was active, healthy again, but not yet for long enough to clear.
uint8 STATE_HEALING = 3
# Latched fault whose cause is gone; stays until acknowledged.
uint8 STATE_AWAITING_ACK = 4

string code
uint8 severity
uint8 state

# Text from the catalogue.
string description
# Detail of the latest failing report.
string message
# Who reported it last.
string source

# False if the code is not in the catalogue.
bool known

# How many times this fault has become active, including before restarts.
uint64 occurrence_count
builtin_interfaces/Time first_activated
builtin_interfaces/Time last_activated
