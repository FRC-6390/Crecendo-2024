// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vission.LimeLight;

public class SeekMode extends Command {
  public LimeLight limelight; public Drivetrain6390 drivetrain; public PIDController controller = new PIDController(0.05, 0, 0);
  public SeekMode(LimeLight limeLight, Drivetrain6390 drivetrain) {
    this.drivetrain = drivetrain; this.limelight = limeLight;// Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limelight.hasValidTarget())
    {
      drivetrain.drive(new ChassisSpeeds(drivetrain.getSpeeds().vxMetersPerSecond, drivetrain.getSpeeds().vyMetersPerSecond, controller.calculate(limelight.getTargetHorizontalOffset())));
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
    return false;
  }
}
