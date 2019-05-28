package pozyx_ros_examples;

public interface EulerAngles extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pozyx_ros_examples/EulerAngles";
  static final java.lang.String _DEFINITION = "float32 yaw\nfloat32 roll\nfloat32 pitch\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getYaw();
  void setYaw(float value);
  float getRoll();
  void setRoll(float value);
  float getPitch();
  void setPitch(float value);
}
