package frc.robot.utilities.vission;

import java.util.HashMap;

public class RobotVision 
{

 HashMap<String, LimeLight> cameras = new HashMap<>();

 LimeLight defaultCamera;
 public RobotVision(HashMap<String, LimeLight> camerasPopulate)
 {
    defaultCamera = new LimeLight(new LimelightConfig("limelight", 0, 0));
    cameras = camerasPopulate;
 }

 public LimeLight getCamera(String key)
 {
    return(cameras.get(key));
 }

 public LimeLight getDefaultCamera()
 {
    return(defaultCamera);
 }
}
