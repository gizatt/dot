# Describes a series of position targets for
# a set of num_positions servos. The position targets
# should be encoded (in us, range ~0-2000) in data
# s.t. position i at break j is at data[j*num_positions + i].
# breaks_from_start_ should be offsets (in seconds) from the
# timestamp in the header at which each position set should be
# achieved.
# This is aggressively flattened and ugly because
# rosserial has been choking on "cleaner" array-of-array messages,
# and serial bandwidth is very precious.
std_msgs/Header header
uint16 num_positions
uint16 num_breaks
float32[] breaks_from_start
int16[] data