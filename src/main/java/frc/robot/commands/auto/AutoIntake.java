// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.vission.LimelightHelpers;

public class AutoIntake extends Command {
  public Intake intake;
  public Drivetrain6390 drivetrain; 
  public String limelight;
  public boolean isDone;
  /** Creates a new AutoIntake. */
  public AutoIntake(Intake intake, Drivetrain6390 drivetrain, String limelight) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    drivetrain.setRobotRelative(true);
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(LimelightHelpers.getTV(limelight) && !intake.hasNote())
    {
      drivetrain.drive(new ChassisSpeeds(0,0.2,0));
    }
    else if(intake.hasNote())
    {
      isDone = true;
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
    return isDone;
  }
}
