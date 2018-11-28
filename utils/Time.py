import numpy as np

# This object is similar to rospy.Time but operations such as substraction and
# comparison are much safer and better defined.
# As example, rospy.Time difference is defined as
#   return Duration(self.secs - other.secs, self.nsecs - other.nsecs)
# which overflows when other.nsecs < self.nsecs.
class Time:

  # There are 10^9 nanoseconds in a second
  SECOND_TO_NANOSECOND = 1000000000

  def __init__(self, secs, nsecs):

    assert( isinstance(secs, np.uint64) )
    assert( isinstance(nsecs, np.uint64) )
    assert( nsecs < self.SECOND_TO_NANOSECOND )

    self.secs = secs
    self.nsecs = nsecs

  # comparison

  def __ge__(self, other):
    """
    @brief >= test for time values.
    """
    return not self.__lt__( other )

  def __gt__(self, other):
    """
    @brief > test for time values (Inherited from roslib.rostime.TVal) 	source code
    """
    return not self.__leq__( other )

  def __le__(self, other):
    """
    @brief <= test for time values.
    """
    return self.secs < other.secs or ((self.secs == other.secs) and (self.nsecs <= other.nsecs))

  def __lt__(self, other):
    """
    @brief < test for time values.
    """
    return self.secs < other.secs or ((self.secs == other.secs) and (self.nsecs < other.nsecs))

  # equality

  def __eq__(self, other):
    """
    @brief Equals test for Time.
    """
    if not isinstance(other, Time):
      raise TypeError("Cannot compare to non-Time")
    return (self.secs == other.secs) and (self.nsecs == other.nsecs)

  def __ne__(self, other):
    """
    """
    return not self.__eq__( other )

  # arithmetic

  def __add__(self, other):
    """
    @brief Add duration to this time.
    """
    if not isinstance(other, Time):
      raise TypeError("Cannot operate with non-Time")

    secs = self.secs + other.secs
    nsecs = self.nsecs + other.nsecs

    # if self.SECOND_TO_NANOSECOND <= self.nsecs + other.nsecs
    if self.SECOND_TO_NANOSECOND - self.nsecs <= other.nsecs:
      secs = secs + np.uint64(1)
      nsecs = nsecs - self.SECOND_TO_NANOSECOND

    return Time(secs, nsecs)

  def __sub__(self, other):
    """
    @brief Subtract time or duration from this time.
    """
    if not isinstance(other, Time):
      raise TypeError("Cannot operate with non-Time")

    assert( other <= self )

    secs = self.secs - other.secs

    if self.nsecs < other.nsecs:
      secs = secs - np.uint64(1)
      nsecs = (self.nsecs + np.uint64(self.SECOND_TO_NANOSECOND)) - other.nsecs
    else:
      nsecs = self.nsecs - other.nsecs

    return Time(secs, nsecs)

  # other

  def is_zero(self):
    """
    @return: True if time is zero (secs and nsecs are zero)
    @rtype: bool
    """
    return self.secs == 0 and self.nsecs == 0

  def __nonzero__(self):
    """
    @brief Check if time value is zero.
    """
    return self.secs or self.nsecs

  def __hash__(self):
    """
    Time values are hashable.
    """
    return ("%u.%u" % (self.secs, self.nsecs)).__hash__()

  def __str__(self):
    return "(%u.%u)" % (self.secs, self.nsecs)

  def __repr__(self):
    return "Time[%u.%u]" % (self.secs, self.nsecs)

  # Time functions

  def to_sec(self):
    """
    @return: time as float seconds (same as time.time() representation)
    @rtype: float
    """
    return float(self.secs) + float(self.nsecs) / 1e9
