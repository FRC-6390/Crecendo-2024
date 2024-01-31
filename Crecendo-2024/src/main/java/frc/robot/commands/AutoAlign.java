// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;


public class AutoAlign extends Command {
  
  //PID controller
  public PIDController controller;
  public PIDController yController;
  public PIDController thetaController;
  //Declare the drivetrain object
  public Drivetrain6390 drivetrain;
  
  // public NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  
  //PID constants
  double kP = 0.065;
  double kI = 0.003;
  double kD = 0;

  double kP2 = 0.09;
  double kI2 = 0.003;
  double kD2 = 0;

  double kP3 = 0.09;
  double kI3 = 0.003;
  double kD3 = 0;

  double targetHeightMeters = 0.7112;
  public String direction;
  public LimeLight limelight;
  public double Ypos;
  public double Xpos;
  public double rot;
  public static boolean isDone;

  public AutoAlign(Drivetrain6390 drivetrain, LimeLight limelight, double Ypos, double Xpos, double rot)
  {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.Ypos = Ypos;
    this.Xpos = Xpos;
    this.rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    controller = new PIDController(kP, kI, kD);
    yController = new PIDController(kP2, kI2, kD2);
    thetaController = new PIDController(kP3, kI3, kD3);
    thetaController.enableContinuousInput(-180, 180);
    //-Math.PI Math.PI
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limelight.hasValidTarget())
    {
      drivetrain.drive(new ChassisSpeeds(controller.calculate(limelight.getTargetVerticalOffset(), Xpos) * -1,yController.calculate(limelight.getTargetHorizontalOffset(), Ypos), thetaController.calculate(drivetrain.getHeading(), rot)));
     //yController.calculate(drivetrain.getHeading(), -180
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
