// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


// import edu.wpi.first.math.util.Units;
import java.math.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.TurnCommand;
import frc.robot.subsystems.Arm;
import frc.robot.utilities.ShooterHelper;
import frc.robot.utilities.vission.LimeLight;

public class AutoAim extends Command {

  public Drivetrain6390 drivetrain;
//   public LimeLight limelight;
  public Arm arm;
  public static Pose2d scoringPos = new Pose2d(1.24, 5.52, new Rotation2d());
  public static Pose2d scoringPosR = new Pose2d(15.26, 5.52, new Rotation2d());
  public static Pose2d scoringPosC = new Pose2d(0.38, 5.52, new Rotation2d());
  public static Pose2d scoringPosRC = new Pose2d(16.29, 5.53, new Rotation2d());
  public double closestDistance = 10000;
  public double[] distances = new double[]{2.616,4.796137311277181};
  public double[] armAngles = new double[]{-0.3970532722, -0.4485960754};

  public double[] distanceToCircles = new double[]{};
  public int index = 0;
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
    // if(drivetrain.getVisionPose().getX() > 14.5)
    // {
    //   CommandScheduler.getInstance().schedule(new ArmTest(arm, -0.211));
    //   RobotContainer.speed = -0.5;
    //   System.out.println("CLOSE SHOTT");
    // }
    // if(drivetrain.getVisionPose().getX() < 14.5)
    // {
    //   System.out.println("FAR SHOTT");
    //   CommandScheduler.getInstance().schedule(new ArmTest(arm, -0.3970532722));
    //   RobotContainer.speed = -0.52;
    // }

    // CommandScheduler.getInstance().schedule(AutoBuilder.pathfindToPose(new Pose2d(scoringPosR.getX(), drivetrain.getVisionPose().getY(), drivetrain.getVisionPose().getRotation()), new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(540))));

    
    double xdiff = scoringPosRC.getX() - drivetrain.getVisionPose().getX();
    double ydiff = scoringPosRC.getY() - drivetrain.getVisionPose().getY();

    // if(ydiff < 0){
    //   ydiff *= -1;
    //   System.out.println("------------------FILLLIPPPEEDDD AHHHHH------------------------------");
    // }
    // xdiff = xdiff)
    // ydiff = Math.abs(ydiff);
    double csqrd = Math.pow(xdiff, 2) + Math.pow(ydiff, 2);
    double c = Math.sqrt(csqrd);

    
    for(int i =0; i< distances.length; i++)
    {
      double dis = Math.abs(c - distances[i]);
      if(closestDistance > dis)
      {
        closestDistance = dis;
        index = i;
      }
    }

    // System.out.println("FAR SHOTT");
    CommandScheduler.getInstance().schedule(new ArmTest(arm, armAngles[index]));
    RobotContainer.speed = -0.52;
    
    double radians = Math.atan2(ydiff, xdiff);
    
    // radians *= -1d;
    double degrees = Units.radiansToDegrees(radians);
    double[] coords = ShooterHelper.coordinates(radians, distances[index]);
    double desXCoord = (scoringPosRC.getX() - coords[0]);
    double desYCoord = (scoringPosRC.getY() - coords[1]);
    // SmartDashboard.putNumber("xdiff", xdiff);
    // SmartDashboard.putNumber("ydiff", ydiff);
    // SmartDashboard.putNumber("Coords X", coords[0]);
    // SmartDashboard.putNumber("Coords Y", coords[1]);
    // SmartDashboard.putNumber("combined X", desXCoord);
    // SmartDashboard.putNumber("combined Y", desYCoord);
    // SmartDashboard.putNumber("Distance", c);
    // SmartDashboard.putNumber("Degrees", degrees);
    // SmartDashboard.putNumber("Radians", radians);

    // CommandScheduler.getInstance().schedule(
    //   AutoBuilder.pathfindToPose
    //   (
    //     new Pose2d(desXCoord, desYCoord, Rotation2d.fromRadians(radians)), 
    //     new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(540))
    //   )
    // );
    // double clampedDegrees = Math.max(-, Math.min(0, 0));

      CommandScheduler.getInstance().schedule(new TurnCommand(drivetrain, degrees));
  }
  else
  {
    double xdiff = scoringPosC.getX() - drivetrain.getVisionPose().getX();
    double ydiff = scoringPosC.getY() - drivetrain.getVisionPose().getY();

    double csqrd = Math.pow(xdiff, 2) + Math.pow(ydiff, 2);
    double c = Math.sqrt(csqrd);

    
    for(int i =0; i< distances.length; i++)
    {
      double dis = Math.abs(c - distances[i]);
      if(closestDistance > dis)
      {
        closestDistance = dis;
        index = i;
      }
    }

    CommandScheduler.getInstance().schedule(new ArmTest(arm, armAngles[index]));
    RobotContainer.speed = -0.52;
    
    double radians = Math.atan2(ydiff, xdiff);
    
    double degrees = Units.radiansToDegrees(radians);
    double[] coords = ShooterHelper.coordinates(radians, distances[index]);
    double desXCoord = (scoringPosC.getX() + coords[0]);
    double desYCoord = (scoringPosC.getY() - coords[1]);
    // SmartDashboard.putNumber("xdiff", xdiff);
    // SmartDashboard.putNumber("ydiff", ydiff);
    // SmartDashboard.putNumber("Coords X", coords[0]);
    // SmartDashboard.putNumber("Coords Y", coords[1]);
    // SmartDashboard.putNumber("combined X", desXCoord);
    // SmartDashboard.putNumber("combined Y", desYCoord);
    // SmartDashboard.putNumber("Distance", c);
    // SmartDashboard.putNumber("Degrees", degrees);
    // SmartDashboard.putNumber("Radians", radians);

    CommandScheduler.getInstance().schedule(
      AutoBuilder.pathfindToPose
      (
        new Pose2d(desXCoord, desYCoord, Rotation2d.fromRadians(radians)), 
        new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(540))
      )
    );

    // CommandScheduler.getInstance().schedule(new TurnCommand(drivetrain, degrees));

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
    // drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
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
