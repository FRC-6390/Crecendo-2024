package frc.robot.utilities.vission;

import java.util.HashMap;

public class RobotVision {

   private HashMap<String, LimeLight> cameras;

   private LimeLight defaultCamera;

   public RobotVision(String... tableNames) {
      cameras = new HashMap<>();
      for (int i = 0; i < tableNames.length; i++) {
         LimeLight limeLight = new LimeLight(tableNames[i]);
         cameras.put(tableNames[i], limeLight);
      }
      defaultCamera = cameras.get(tableNames[0]);
   }

   public RobotVision(LimeLight... camerasPopulate) {
      cameras = new HashMap<>();
      defaultCamera = camerasPopulate[0];
      for (LimeLight limeLight : camerasPopulate) {
         cameras.put(limeLight.config.table(), limeLight);
      }
   }

   public LimeLight getCamera(String key) {
      return (cameras.get(key));
   }

   public LimeLight getDefaultCamera() {
      return (defaultCamera);
   }
}
