// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


// import edu.wpi.first.math.util.Units;
import java.math.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.TurnCommand;
import frc.robot.subsystems.Arm;
import frc.robot.utilities.vission.LimeLight;

public class AutoAim extends Command {

  public Drivetrain6390 drivetrain;
//   public LimeLight limelight;
  public Arm arm;
    public static Pose2d scoringPos = new Pose2d(1.24, 5.52, new Rotation2d());
  public static Pose2d scoringPosR = new Pose2d(15.26, 5.52, new Rotation2d());

  
  /** Creates a new AutoAim. */
  public AutoAim(Drivetrain6390 drivetrain, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    // this.limelight = limelight;
    addRequirements(arm);
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  if(drivetrain.getSide())
  {
    if(drivetrain.getVisionPose().getX() > 15)
    {
      CommandScheduler.getInstance().schedule(new ArmTest(arm, -0.211));
      RobotContainer.speed = -0.5;
    }
    if(drivetrain.getVisionPose().getX() < 15)
    {
      CommandScheduler.getInstance().schedule(new ArmTest(arm, -0.3970532722));
      RobotContainer.speed = -0.52;
    }

    double xdiff = scoringPosR.getX() - drivetrain.getVisionPose().getX();
    double ydiff = scoringPosR.getY() - drivetrain.getVisionPose().getY();
    double radians = Math.atan2(ydiff, xdiff);
    double degrees = Units.radiansToDegrees(radians);
    CommandScheduler.getInstance().schedule(new TurnCommand(drivetrain, degrees));
  }

    // if(drivetrain.getVisionPose().getX() > 15)
    // {
    //   arm.
    // }
    // LimeLight.getDistanceFromTarget(45, Units.inchesToMeters(9), 1.36652);
    // double pos = Math.max(-1, Math.min((distanceToAngle(drivetrain.getVisionPose().getX(), 2.083) * -1), 1));
    // arm.setPosition(pos);

    // if(arm.atPosition())
    // {
        
    // }
    // SmartDashboard.putNumber("Angle",distanceToAngle(drivetrain.getVisionPose().getX(), 2.083) * -1);
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
    return false;
  }

  private double distanceToAngle(double distance, double height)
  {
    return (Math.atan(height / distance));
  }
}
