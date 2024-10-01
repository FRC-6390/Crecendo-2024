// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.TurnCommand;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;

public class AmpAlign extends Command {
  public Drivetrain6390 drivetrain;
  public LimeLight limelight;
  public double tx;
  public PIDController alignPID = new PIDController(0.1, 0, 0);
  
  /** Creates a new AmpAlign. */
  public AmpAlign(Drivetrain6390 drivetrain ,LimeLight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  if(drivetrain.getSide())
  {
    limelight.setPriorityId(5);
  }
  else
  {
    limelight.setPriorityId(6);
  }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    CommandScheduler.getInstance().schedule(new TurnCommand(drivetrain, 90));
    if(limelight.hasValidTarget())
    {
      double xspeed = alignPID.calculate(limelight.getTargetHorizontalOffset(), 0);
      drivetrain.feedbackDrive(new ChassisSpeeds(0,xspeed,0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.feedbackDrive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
