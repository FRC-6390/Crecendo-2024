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
    //controller..whileTrue(new TurnAlign(driveTrain, limelight, 0));
    // controller.b.onTrue(new Test(driveTrain, limelight, 0,0,0));
    controller.a.onTrue(new InstantCommand(test::setHome));
    controller.b.onTrue(new AutoAim(driveTrain, limelight, test));
    //controller.b.onTrue(new ArmTest(test, 0.5));
  }


  public Command getAutonomousCommand(){

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
//Saachi Shenanigans
    driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    PathPlannerPath path = PathPlannerPath.fromPathFile("New New New Path");

// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
PathConstraints constraints = new PathConstraints(
        0.1, 0.1,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
return AutoBuilder.followPath(path);
// return new exampleAuto(s_Swerve);
// //Saachi Shenanigans part 2
// PathPlannerPath path = PathPlannerPath.fromPathFile("Testing");

// // Create a path following command using AutoBuilder. This will also trigger event markers.
// return AutoBuilder.followPath(path);
    
    
  }

}



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.Drive;
// import frc.robot.subsystems.Drivetrain6390;
// import frc.robot.utilities.controller.DebouncedController;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
//  * subsystems, commands, and trigger mappings) should be declared here.
//  */
// public class RobotContainer {
//   // The robot's subsystems and commands are defined here...
//   public static final Drivetrain6390 driveTrain = new Drivetrain6390();

//   private final SendableChooser<Command> autoChooser;

//   public DebouncedController controller = new DebouncedController(0);
//   /** The container for the robot. Contains subsystems, OI devices, and commands. */
//   public RobotContainer() {
//     // Register named commands
//     NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
//     NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
//     NamedCommands.registerCommand("print hello", Commands.print("hello"));

//     driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    
//     // Configure the trigger bindings
//     configureBindings();

//     autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
//     SmartDashboard.putData("Auto Mode", autoChooser);
//   }

//   /**
//    * Use this method to define your trigger->command mappings. Triggers can be created via the
//    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
//    * predicate, or via the named factories in {@link
//    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
//    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
//    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
//    * joysticks}.
//    */
//   private void configureBindings() {
//     // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
//     //SmartDashboard.putData("Example Auto", new PathPlannerAuto("New Auto"));

//     // Add a button to run pathfinding commands to SmartDashboard
//     // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
//     //   new Pose2d(2.9, 5.4, Rotation2d.fromDegrees(0)), 
//     //   new PathConstraints(
//     //     1.0, 1, 
//     //     Units.degreesToRadians(360), Units.degreesToRadians(540)
//     //   ), 
//     //   0, 
//     //   2.0
//     // ));
//     // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
//     //   new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
//     //   new PathConstraints(
//     //     4.0, 4.0, 
//     //     Units.degreesToRadians(360), Units.degreesToRadians(540)
//     //   ), 
//     //   0, 
//     //   0
//     // ));

//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     return Commands.runOnce(() -> {
//       Pose2d currentPose = driveTrain.getPose();
      
//       // The rotation component in these poses represents the direction of travel
//       // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
//       // Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(-1,0)), new Rotation2d());

//       // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
//       // PathPlannerPath path = new PathPlannerPath(
//       //   bezierPoints, 
//       //   new PathConstraints(
//       //     1.0, 1.0, 
//       //     Units.degreesToRadians(360), Units.degreesToRadians(540)
//       //   ),  
//       //   new GoalEndState(0.0, currentPose.getRotation())
//       // );

//       driveTrain.resetOdometry(new Pose2d(new Translation2d(2.92, 5.53), new Rotation2d()));
      
//       PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");

//       // Prevent this path from being flipped on the red alliance, since the given positions are already correct
//       path.preventFlipping = true;

//       AutoBuilder.followPath(path).schedule();
//     });
//   }
// }



  
    
