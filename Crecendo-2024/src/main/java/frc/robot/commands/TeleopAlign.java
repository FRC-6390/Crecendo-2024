// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class TeleopAlign extends Command {
  
  //PID controller
  public PIDController controller;
  public PIDController yController;
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;
  
  //PID constants
  double kP = 0.065;
  double kI = 0.003;
  double kD = 0;

  double kP2 = 0.09;
  double kI2 = 0.003;
  double kD2 = 0;

  public LimeLight limelight;
  public double Ypos;
  public double Xpos;
  public static boolean isDone;

  public TeleopAlign(Drivetrain6390 drivetrain, LimeLight limelight, double Ypos, double Xpos)
  {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.Ypos = Ypos;
    this.Xpos = Xpos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    controller = new PIDController(kP, kI, kD);
    yController = new PIDController(kP2, kI2, kD2);
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limelight.hasValidTarget())
    {
      drivetrain.drive(new ChassisSpeeds(yController.calculate(limelight.getTargetHorizontalOffset(), Ypos),controller.calculate(limelight.getTargetVerticalOffset(), Xpos) * -1,0));

      if(yController.calculate(limelight.getTargetHorizontalOffset(), 0) < 0.2)
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
