// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Arm;
import frc.robot.utilities.vission.LimeLight;

public class AutoAim extends Command {

  public Drivetrain6390 drivetrain;
//   public LimeLight limelight;
  public Arm arm;
  public boolean isDone;

  
  /** Creates a new AutoAim. */
  public AutoAim(Drivetrain6390 drivetrain, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    // this.limelight = limelight;
    this.arm = arm;
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
    // LimeLight.getDistanceFromTarget(45, Units.inchesToMeters(9), 1.36652);
    arm.setPosition(distanceToAngle(drivetrain.getPose().getX(), 2.083) * -1);
    if(arm.atPosition())
    {
        isDone = true;
    }
    SmartDashboard.putNumber("Angle",distanceToAngle(drivetrain.getPose().getX(), 2.083) * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    arm.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }

  private double distanceToAngle(double distance, double height)
  {
    return (Math.atan(height / distance));
  }
}
