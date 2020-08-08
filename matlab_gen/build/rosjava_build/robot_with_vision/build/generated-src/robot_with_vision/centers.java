package robot_with_vision;

public interface centers extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_with_vision/centers";
  static final java.lang.String _DEFINITION = "Header header\nstring[] x_centers\nstring[] y_centers\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getXCenters();
  void setXCenters(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getYCenters();
  void setYCenters(java.util.List<java.lang.String> value);
}
