// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightHelpers;
import frc.robot.utilities.vision.LimeLight.LedMode;

public class SeekMode extends Command {
  public String limelight; public Drivetrain6390 drivetrain; public PIDController controller = new PIDController(0.05, 0, 0);
  public double oldRot = 0;
  public boolean hasRecorded;
  public SeekMode(String limeLight, Drivetrain6390 drivetrain) {
    this.drivetrain = drivetrain; this.limelight = limeLight;
  }

  @Override
  public void initialize() 
  {

  }


  @Override
  public void execute() 
  { 
    if(LimelightHelpers.getTV(limelight))
      { 
        drivetrain.feedbackDrive(new ChassisSpeeds(-1, 0, controller.calculate(LimelightHelpers.getTX(limelight))));
      }
      else
      {
        drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    PPHolonomicDriveController.clearFeedbackOverrides();
    drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
