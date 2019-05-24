package pozyx_ros_examples;

public interface DeviceRange extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pozyx_ros_examples/DeviceRange";
  static final java.lang.String _DEFINITION = "uint32 timestamp\r\nuint32 distance\r\nint16 RSS\r\nstring device\r\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getTimestamp();
  void setTimestamp(int value);
  int getDistance();
  void setDistance(int value);
  short getRSS();
  void setRSS(short value);
  java.lang.String getDevice();
  void setDevice(java.lang.String value);
}
