// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class TurnAlign extends Command {
  
  //PID controllers
  public PIDController thetaController;
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;
  

  double kP3 = 0.04;
  double kI3 = 0;
  double kD3 = 0;

  double targetHeightMeters = 0.7112;
  public String direction;
  public LimeLight limelight;
  public double Ypos;
  public double Xpos;
  public static boolean isDone;
  public double offset;

  public TurnAlign(Drivetrain6390 drivetrain, LimeLight limelight, double offset)
  {
    this.drivetrain = drivetrain;
    this.offset = offset;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    thetaController = new PIDController(kP3, kI3, kD3);
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limelight.hasValidTarget())
    {
      drivetrain.drive(new ChassisSpeeds(0,0, thetaController.calculate(limelight.getTargetVerticalOffset(), 0) * -1));
     //yController.calculate(drivetrain.getHeading(), -180
      if(thetaController.calculate(limelight.getTargetVerticalOffset(), 0) < 0.2 && thetaController.calculate(limelight.getTargetVerticalOffset(), 0) > -0.2)
      {
        drivetrain.drive(new ChassisSpeeds(0,0,0));
        isDone = true;
      }
    }
    else
    {
      drivetrain.drive(new ChassisSpeeds(0,0,0));
      isDone = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
