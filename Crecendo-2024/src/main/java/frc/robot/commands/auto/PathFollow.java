// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;

public class PathFollow extends Command {
  /** Creates a new PathFollow. */
  public Drivetrain6390 drivetrain;

  public PathPlannerPath path = PathPlannerPath.fromPathFile("Saachi");

  public List<PathPoint> points = path.getAllPathPoints();

  public PathPlannerTrajectory traj = path.getTrajectory(new ChassisSpeeds(0,0,0), new Rotation2d(0));
  
  public PathPoint start = points.get(0);
  
  public PPHolonomicDriveController controller = new PPHolonomicDriveController(null, null, 0, 0);
  

  public PathFollow(Drivetrain6390 drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain =  drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  
    drivetrain.resetHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    controller.calculateRobotRelativeSpeeds(drivetrain.getPose(), traj.getEndState());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
