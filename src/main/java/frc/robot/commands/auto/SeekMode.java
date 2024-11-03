// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightHelpers;
import frc.robot.utilities.vission.LimeLight.LedMode;

public class SeekMode extends Command {
  public String limelight; public Drivetrain6390 drivetrain; public PIDController controller = new PIDController(0.07, 0, 0);
  public SeekMode(String limeLight, Drivetrain6390 drivetrain) {
    this.drivetrain = drivetrain; this.limelight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    drivetrain.setRobotRelative(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(LimelightHelpers.getTV(limelight))
    {
      drivetrain.drive(new ChassisSpeeds(drivetrain.getSpeeds().vxMetersPerSecond, 0, controller.calculate(LimelightHelpers.getTX(limelight))));
      drivetrain.setRobotRelative(false);
    }
    else
    {
      drivetrain.setRobotRelative(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.setRobotRelative(false);
    drivetrain.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
