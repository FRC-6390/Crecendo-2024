// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Test;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;
import java.util.ArrayList;
import java.util.List;
import frc.robot.commands.auto.TurnAlign;

import org.json.simple.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Auto;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.auto.JanusAuto;
import frc.robot.commands.auto.TurnAlign;
import frc.robot.commands.auto.TurnCommand;

public class RobotContainer {
  public static Drivetrain6390 driveTrain = new Drivetrain6390();
  public static frc.robot.subsystems.Test test = new Test();
  public static LimeLight limelight = new LimeLight();

  
    
  public static DebouncedController controller = new DebouncedController(0);

  public RobotContainer() {
   //driveTrain.shuffleboard();
    driveTrain.init();
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    SmartDashboard.putNumber("Heading", driveTrain.getHeading());
    SmartDashboard.putNumber("Rotation2D", driveTrain.getRotation2d().getDegrees());
    
    configureBindings();

  }

  private void configureBindings() 
  {

     
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    controller.y.onTrue(new SequentialCommandGroup(new AutoAlign(driveTrain, limelight, 0, 0, 0, 0.06), new TurnAlign(driveTrain, limelight, 0)));
    controller.x.whileTrue(new DebugCommand(driveTrain, limelight));
    // controller.a.onTrue(new InstantCommand(test::setHome));
    //controller.b.onTrue(new AutoAim(driveTrain, limelight, test));
    controller.b.onTrue(new ArmTest(test, 0.5));
    controller.leftStick.onTrue(new ArmTest(test, 1));
    controller.rightStick.onTrue(new ArmTest(test, 0));


  }


  public Command getAutonomousCommand(){

    // Pose2d currentPose = driveTrain.getPose();
    // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    // Pose2d endPos = new Pose2d((new Translation2d(1, 0)), new Rotation2d());
    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    // PathPlannerPath path = new PathPlannerPath(bezierPoints, new PathConstraints(4, 3, Units.degreesToRadians(360), Units.degreesToRadians(540)), new GoalEndState(0.0, currentPose.getRotation()));
    // path.preventFlipping = true;

    //-----------------------Janus Autos---------------------------//

    //Right Side 2 piece
    // driveTrain.resetHeading();
    // driveTrain.resetOdometry(new Pose2d(0,0, RobotContainer.driveTrain.getRotation2d()));
    // return new SequentialCommandGroup
    // (
    // new JanusAuto(driveTrain, Constants.AUTO.RIGHT_SIDE_SEGMENT_1.build()),
    // new TurnAlign(driveTrain, limelight, 0),
    // new AutoAim(driveTrain, limelight, test)
    // );

    //Middle 2 piece
    // driveTrain.resetHeading();
    // driveTrain.resetOdometry(new Pose2d(0,0, RobotContainer.driveTrain.getRotation2d()));
    // return new SequentialCommandGroup
    // (
    // new JanusAuto(driveTrain, Constants.AUTO.RIGHT_SIDE_SEGMENT_1.build()),
    // new TurnAlign(driveTrain, limelight, 0),
    // new AutoAim(driveTrain, limelight, test)
    // );


    //Left 2 piece
    // driveTrain.resetHeading();
    // driveTrain.resetOdometry(new Pose2d(0,0, RobotContainer.driveTrain.getRotation2d()));
    // return new SequentialCommandGroup
    // (
    // new JanusAuto(driveTrain, Constants.AUTO.RIGHT_SIDE_SEGMENT_1.build()),
    // new TurnAlign(driveTrain, limelight, 0),
    // new AutoAim(driveTrain, limelight, test)
    // );


    //--------------------------Limelight Autos------------------------------//


    //Mid auto skeleton
    // return new SequentialCommandGroup
    // (
    //   new AutoDrive(driveTrain, limelight, 0, -0.2, 0),
    //   new AutoAlign(driveTrain, limelight, -3.47, 0, 0, 0), new ArmTest(test, 0.5)
    // );
    
    //Right Auto Skeleton
    // return new SequentialCommandGroup
    // (
       
    // new AutoDrive(driveTrain, limelight, 0, -0.75, 0),
    // new AutoAlign(driveTrain, limelight, -2.65, 0, 0 , 0),
    // new TurnAlign(driveTrain, limelight, 0)
    
    // );

    //Right Side 2 PIECE skeleton
    // return new SequentialCommandGroup
    // (
       
    // new AutoDrive(driveTrain, limelight, 0, -0.75, 0),
    // new AutoAlign(driveTrain, limelight, -2.75, 0, 0 , 0),
    // new AutoAlign(driveTrain, limelight, 6.9, 20.77, 0, 0.03),
    // new AutoAlign(driveTrain, limelight, -2.75, 0, 0 , 0),
    // new TurnAlign(driveTrain, limelight, 0)
    
    // );

    
    //Janus Test Auto
    // driveTrain.resetHeading();
    // driveTrain.resetOdometry(new Pose2d(0,0, RobotContainer.driveTrain.getRotation2d()));
    // return new SequentialCommandGroup
    // (
    // new JanusAuto(driveTrain, Constants.AUTO.RIGHT_SIDE_SEGMENT_1.build()),
    // new TurnAlign(driveTrain, limelight, 0),
    // new AutoAim(driveTrain, limelight, test)
    // );

//--------------------------Pathplanner Autos-----------------------------//

    // driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d()));

    // driveTrain.resetHeading();

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Saachi");


    // return AutoBuilder.followPath(path);

    Pose2d currentPose = driveTrain.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d wayPoint1 = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0.48, 1.61)), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0.52, -1.61)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, wayPoint1 , endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
  }

}