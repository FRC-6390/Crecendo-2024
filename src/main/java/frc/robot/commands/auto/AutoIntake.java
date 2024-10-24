// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.vission.LimeLight;

public class AutoIntake extends Command {
  public LimeLight limelight; public Drivetrain6390 drivetrain; public PIDController controller = new PIDController(0.1, 0, 0);
  public Intake intake;
  public boolean isDone = false;
  public boolean isIntakeMode = false;
  public AutoIntake(LimeLight limeLight, Drivetrain6390 drivetrain, Intake intake) {
    this.drivetrain = drivetrain; this.limelight = limeLight;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
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
    if(limelight.hasValidTarget() || !isIntakeMode)
    {
      drivetrain.drive(new ChassisSpeeds(drivetrain.getSpeeds().vxMetersPerSecond, drivetrain.getSpeeds().vyMetersPerSecond, controller.calculate(limelight.getTargetHorizontalOffset())));
    }
    if(controller.atSetpoint())
    {
      isIntakeMode = true;
      System.out.println("reached");
    }
    if(isIntakeMode)
    {
      Drivetrain6390.setRobotRelative(true);
      drivetrain.drive(new ChassisSpeeds(0,0.1,0));
      if(intake.hasNote())
      {
        isDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    isIntakeMode =false;
    Drivetrain6390.setRobotRelative(false);
    drivetrain.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
