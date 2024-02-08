// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class AutoDrive extends Command {
  
  //PID controller
  public PIDController controller;
  public PIDController yController;
  public PIDController thetaController;
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;

  public LimeLight limelight;
  public double Yspd;
  public double Xspd;
  public double rotspd;
  public static boolean isDone;

  public AutoDrive(Drivetrain6390 drivetrain, LimeLight limelight, double Yspd, double Xspd, double rotspd)
  {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.Yspd = Yspd;
    this.Xspd = Xspd;
    this.rotspd = rotspd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  if(!limelight.hasValidTarget())
  {
   drivetrain.drive(new ChassisSpeeds(Xspd, Yspd, rotspd)); 
  }
  else
  {
    isDone = true;
  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
