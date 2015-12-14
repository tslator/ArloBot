__author__ = 'tslator'

# The purpose of this node is to centralize all power information gathering for Arlobot
#
# Arlobot uses the M4ATX power supply which provides an API for obtaining information on all of the power rails: 12v, 5v,
# and 3.3v.  That API can be wrapped into a node that publishes relevant data to be consumed by this node.  Additional
# power and current monitoring maybe done, i.e., for the motors.

# Presently, there is a m4atx node which publishes information and also uses espeak to notify about voltage levels.
# Would like to uses this as the basis for a M4ATX node that is limited to reading the data from the M4ATX PS and
# publishing only the M4ATX relevant data.

# Note: the values being reported by the m4api.c does not match the measured voltage.  There is a newer version that
# needs to be checked out.

# This node will be the aggregator for all of the power/current monitoring nodes:
#   - m4atx - power supply voltages and temperature
#   - battery charge state - battery voltage, capacity (soc), and whether it is charging
#   - ac connected - whether AC is connected


class ArlobotPowerNode:
    pass


if __name__ == "__main__":
    try:
        apn = ArlobotPowerNode()
    except:
        pass

