// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Test;
import frc.robot.utilities.vission.LimeLight;

public class AutoAim extends Command {

  public Drivetrain6390 drivetrain;
  public LimeLight limelight;
  public Test test;

  
  /** Creates a new AutoAim. */
  public AutoAim(Drivetrain6390 drivetrain, LimeLight limelight, Test test) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.test = test;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limelight.hasValidTarget())
    {
      SmartDashboard.putNumber("Test set", distanceToAngle(limelight.getDistanceFromTarget(45, Units.inchesToMeters(9), 1.36652), 1.36652));
      test.setPosition(distanceToAngle(limelight.getDistanceFromTarget(45, Units.inchesToMeters(9), 1.36652), 1.36652));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    test.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double distanceToAngle(double distance, double height)
  {
    return (Math.atan(height / distance));
  }
}
