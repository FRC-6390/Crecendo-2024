// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SWERVEMODULE;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

public class Auto extends Command {
  
  // static PIDConstants XY_PID = new PIDConstants(1, 0, 0);
  // static PIDConstants THETA_PID = new PIDConstants(4.9, 0, 0);
  // private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries());

  

  public Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    
  }
    
    public static Command runAuto(String autoSelector) 
    {
        return AutoBuilder.buildAuto(autoSelector);
       // return AutoBuilder.fullAuto(PathPlannerAuto.getPathGroupFromAutoFile(autoSelector));
    }

   
}
