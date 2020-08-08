package robot_with_vision;

public interface points_to_go_to extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_with_vision/points_to_go_to";
  static final java.lang.String _DEFINITION = "Header header\nstring[] x_points\nstring[] y_points\nstring[] z_points\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getXPoints();
  void setXPoints(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getYPoints();
  void setYPoints(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getZPoints();
  void setZPoints(java.util.List<java.lang.String> value);
}
