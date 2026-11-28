# One result from a monitor: "this condition is failing" or "this condition is healthy".
#
# A monitor publishes its result regularly, or at least on every change. It does
# not decide whether the result amounts to a fault; the fault manager does that
# with the debounce and latch settings of the fault catalogue.

# Fault code as listed in the catalogue, e.g. "LIDAR-001".
string code

# True: the condition is present. False: the condition is absent.
bool failing

# Who reports, e.g. a node name. Optional.
string source

# Human-readable detail for this occurrence, e.g. "no scan for 0.6 s". Optional.
string message
