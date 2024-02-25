// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
 
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain6390;
//import frc.robot.subsystems.Test;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;
 
import frc.robot.commands.auto.TurnAlign;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.commands.Auto;
import frc.robot.commands.*;
import frc.robot.commands.auto.IntakeDrive;
import frc.robot.commands.auto.TurnCommand;
 
public class RobotContainer {
  public static Drivetrain6390 driveTrain = new Drivetrain6390();
  public static Arm arm = new Arm();
  //public static Intake in = new Intake();
 // public static frc.robot.subsystems.Test test = new Test();
  public static LimeLight limelight = new LimeLight();
 
   
  public static DebouncedController controller = new DebouncedController(0);
 
  public RobotContainer() {
    //driveTrain.shuffleboard();
    driveTrain.init();
    NamedCommands.registerCommand("TurnAlign", new TurnAlign(driveTrain, limelight, 0));
    NamedCommands.registerCommand("TurnCommand", new TurnCommand(driveTrain, 0));
    NamedCommands.registerCommand("IntakeRollers", new IntakeRollers(-0.6));
    NamedCommands.registerCommand("IntakeDrive", new IntakeDrive(driveTrain, 0, -0.5, 0));
    NamedCommands.registerCommand("PivotMoveHalf", new ArmTest(arm, -0.5));
    NamedCommands.registerCommand("PivotMoveMax", new ArmTest(arm, -1));
    NamedCommands.registerCommand("PivotMoveLow", new ArmTest(arm, 0));
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
   
    SmartDashboard.putNumber("Heading", driveTrain.getHeading());
    SmartDashboard.putNumber("Rotation2D", driveTrain.getRotation2d().getDegrees());
 
    configureBindings();
 
  }
 
  private void configureBindings()
  {
 
     
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    //controller.y.onTrue(new SequentialCommandGroup(new AutoAlign(driveTrain, limelight, 0, 0, 0, 0.06), new TurnAlign(driveTrain, limelight, 0)));
   // controller.b.onTrue(new AutoAim(driveTrain, limelight, test));
    //controller.b.onTrue(new ArmTest(test, 0.5));
    //controller.leftStick.onTrue(new ArmTest(test, 1));
    //controller.rightStick.onTrue(new ArmTest(test, 0));
    controller.b.whileTrue(new IntakeRollers(0.6));  
    controller.a.whileTrue(new IntakeRollers(-0.6));
    //controller.a.onTrue(new IntakeDrive(driveTrain, 0, -0.3, 0));
    //controller.a.onTrue(new ArmTest(arm,0));
    //controller.x.onTrue(new ArmTest(arm, -1));
    //controller.b.onTrue(new ArmTest(arm, -0.5));
    //controller.y.onTrue(new InstantCommand(arm::setHome));
    //controller.a.whileTrue(new IntakeRollers(-0.2));

    //Controller ideas:

    // controller.y.onTrue(new ArmTest(arm, 0));
    // controller.l

 
  }
 
 
  public Command getAutonomousCommand()
  {
 
  //--------------------------Pathplanner Autos-----------------------------//
 
  driveTrain.resetHeading();
  return new PathPlannerAuto("Test");
 
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
 
 
 
  }
 
}