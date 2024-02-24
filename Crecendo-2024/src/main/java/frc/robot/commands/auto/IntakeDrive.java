// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Intake;


public class IntakeDrive extends Command {
  
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;
  public double Yspd;
  public double Xspd;
  public double rotspd;
  public static boolean isDone;

  public IntakeDrive(Drivetrain6390 drivetrain, double Yspd, double Xspd, double rotspd)
  {
    this.drivetrain = drivetrain;
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
  if(!Intake.getLowerBeamBreak())
  {
    drivetrain.drive(new ChassisSpeeds(Xspd, Yspd, rotspd));
    Intake.setRollers(-0.6, 2);
  }
  else
  {
    isDone=true;
  }
    
  }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    Intake.setRollers(0, 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
