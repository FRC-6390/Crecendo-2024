// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;


public class TurnCommand extends Command {
  
  public PIDController thetaController;
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;
 
  //PID constants
  // double kP = 0.02;
  // double kI = 0.00325;
  // double kD = 0;
  double kP;
  double kI = 0;
  double kD = 0;

  double kP2 = 0.09;
  double kI2 = 0.003;
  double kD2 = 0;

  double kP3 = 0.09;
  double kI3 = 0.003;
  double kD3 = 0;
  public double rot;
  public static boolean isDone;

  public TurnCommand(Drivetrain6390 drivetrain, double rot)
  {
    this.drivetrain = drivetrain;
    this.rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    thetaController = new PIDController(kP3, kI3, kD3);
    thetaController.enableContinuousInput(-180, 180);
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      drivetrain.drive(
        new ChassisSpeeds(drivetrain.getSpeeds().vxMetersPerSecond,drivetrain.getSpeeds().vyMetersPerSecond,
          thetaController.calculate(drivetrain.getHeading(), rot))
      );
     
      if(
        thetaController.calculate(drivetrain.getHeading(), rot) < 0.1 && thetaController.calculate(drivetrain.getHeading(), rot) > -0.1
      )
      {
        drivetrain.drive(new ChassisSpeeds(0,0,0));
        isDone = true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //drivetrain.drive(new ChassisSpeeds(0,0,0));
    System.out.println("Command Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
