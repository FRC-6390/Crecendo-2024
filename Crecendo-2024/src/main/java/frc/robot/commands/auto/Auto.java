// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SWERVEMODULE;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

public class Auto extends Command {
  private static final ReplanningConfig configR = new ReplanningConfig();
  
  private static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, 0, configR);

  // static PIDConstants XY_PID = new PIDConstants(1, 0, 0);
  // static PIDConstants THETA_PID = new PIDConstants(4.9, 0, 0);
  // private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries());

  public Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    AutoBuilder.configureHolonomic(
          RobotContainer.driveTrain::getPose,
          RobotContainer.driveTrain::resetOdometry,
          RobotContainer.driveTrain::getSpeeds,
          RobotContainer.driveTrain::drive,
          config,
          RobotContainer.driveTrain::getSide,
          RobotContainer.driveTrain);
    
  }
    
    public static Command runAuto(String autoSelector) 
    {
        return AutoBuilder.buildAuto(autoSelector);
        //return autoBuilder.fullAuto(PathPlannerAuto.loadPathGroup(autoSelector, new PathConstraints(1.5, 0.8, 0, 0)));
    }

   
}
