// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.APRILTAGS;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class DebugCommand extends Command {
 
  public Drivetrain6390 drivetrain;
  

  public LimeLight limelight;

  public DebugCommand(Drivetrain6390 drivetrain, LimeLight limelight)
  {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d pose = limelight.getBot2DPosition();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Robot Rotation", drivetrain.getHeading());
    SmartDashboard.putNumber("x angle", limelight.getTargetVerticalOffset());
    SmartDashboard.putNumber("tARGET SKEW", limelight.getTargetSkew());
    SmartDashboard.putNumber("Des Rot", APRILTAGS.getByID(limelight.getAprilTagID()).getRotation());
    SmartDashboard.putNumber("Distance to target", limelight.getDistanceFromTarget(45, 0, 1.36652));
    SmartDashboard.putBoolean("Is configured", AutoBuilder.isConfigured());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
