// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimeLight.LedMode;
import frc.robot.utilities.vission.LimelightHelpers;

public class IntakeRollers2 extends Command {
  /** Creates a new Intake. */
  public Intake intake;
  public double speed;
  public boolean useBeamBreak;
  public boolean isHomeSet;
  public IntakeRollers2(Intake intake, double speed, boolean useBeamBreak) {
    addRequirements(intake);
    this.intake = intake;
    this.speed = speed;
    this.useBeamBreak = useBeamBreak;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    intake.setRollersEnabled(true);
    intake.fullWidth(speed);
    intake.centerIntake(speed);
    intake.feed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double rotations = 0;
    if(DriverStation.isAutonomous())
    {
      rotations = 80;
    }
    else
    {
      rotations = 50;
    }
    if(intake.hasNote())
    {
      LimelightHelpers.setLEDMode_ForceOn("limelight");
      if(!isHomeSet)
      {
        intake.setHome();
        isHomeSet = true;
      }
      if(intake.getRotations() < rotations)
      {
        intake.setRollersEnabled(false);
        intake.setReversed(false);
     }
    }
    else
    {
      intake.setRollersEnabled(true);
      isHomeSet = false;
      LimelightHelpers.setLEDMode_ForceOff("limelight");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intake.setRollersEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
