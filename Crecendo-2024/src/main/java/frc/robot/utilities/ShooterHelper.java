package frc.robot.utilities;

import java.lang.reflect.Array;

public class ShooterHelper {
public double[] coordinates(double theta, double distance){
  double x = Math.cos(theta)*distance;
  double y = Math.sin(theta)*distance;
return new double [] {x,y};
}
}
