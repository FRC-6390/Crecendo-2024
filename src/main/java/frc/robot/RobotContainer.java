// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Test;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightHelpers;
import frc.robot.commands.auto.TurnAlign;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import frc.robot.commands.Auto;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoFeed;
import frc.robot.commands.auto.IntakeDrive;
import frc.robot.commands.auto.Shoot;
import frc.robot.commands.auto.TurnCommand;

public class RobotContainer {
  
  public static Arm arm;
//public static frc.robot.subsystems.Test test = new Test();
  public static LimeLight limelight = new LimeLight();
  public static double speed = -0.5;
  public static double threshold = 10;
  public static double armPos = 0;
  public static Drivetrain6390 driveTrain = new Drivetrain6390(limelight);
  public static Climber climber = new Climber();
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Pose2d scoringPos = new Pose2d(1.24, 5.52, new Rotation2d());
  public static Pose2d scoringPosR = new Pose2d(15.26, 5.52, new Rotation2d());
// public static EventLoop eventLoop = new EventLoop();
    
  public static DebouncedController controller = new DebouncedController(0);
  private DebouncedJoystick joystick = new DebouncedJoystick(1);

 
  public SendableChooser<String> autoChooser = new SendableChooser<>(); 

  public RobotContainer() {
    arm = new Arm(joystick);
    //driveTrain.shuffleboard();
    driveTrain.init();
    autoChooser.addOption("Baseline", "Baseline");
    autoChooser.addOption("AutoNonStageBonus", "AutoNonStageBonus");
    autoChooser.addOption("AutoNonStageWithMid", "AutoNonStageWithMid");
    autoChooser.addOption("AutoStageBonus", "AutoStageBonus");
    autoChooser.addOption("AutoStageWithMid", "AutoStageWithMid");
    autoChooser.addOption("AutoNonStageBasic", "AutoNonStageBasic");
    autoChooser.addOption("AutoCenterBasic", "AutoCenterBasic");
    autoChooser.addOption("Nothing", "Nothing");
    autoChooser.addOption("AllThreeNonStage", "AllThreeNonStage");
    autoChooser.addOption("AllThreeStage", "AllThreeStage");
    autoChooser.addOption("AutoStageBasic", "AutoStageBasic");
    autoChooser.addOption("StageSide", "StageSide");
    autoChooser.addOption("Center", "Center");
    autoChooser.addOption("NonStageSide", "NonStageSide");
    autoChooser.addOption("StageSideExtra", "StageSideExtra");
    autoChooser.addOption("NonStageSideExtra", "NonStageSideExtra");
    autoChooser.addOption("StageSideRed", "StageSideRed");

    //GOOOD
    autoChooser.addOption("StageSideExtraFast", "StageSideExtraFast");
    autoChooser.addOption("StageSideExtraFastV2", "StageSideExtraFastV2");
    autoChooser.addOption("StageSideExtraFastRed", "StageSideExtraFastRed");
    autoChooser.addOption("StageSideExtraFastV2Red", "StageSideExtraFastV2Red");
    autoChooser.addOption("Sweeper", "Sweeper");    
    autoChooser.addOption("testauto", "testauto");

    SmartDashboard.putData("AutoChoose", autoChooser);
    SmartDashboard.putNumber("Shooter Speed", speed);
    SmartDashboard.putNumber("Shooter Speed Setpoint", threshold);
    SmartDashboard.putNumber("Arm Position Changer", armPos);
    
    NamedCommands.registerCommand("TurnAlign", new TurnAlign(driveTrain, limelight, 0));
    NamedCommands.registerCommand("TurnCommand", new TurnCommand(driveTrain, 0));
    NamedCommands.registerCommand("Turn25", new TurnAlign(driveTrain,limelight, -0.5));
    NamedCommands.registerCommand("IntakeRollers", new IntakeRollers(-0.6, intake));
    NamedCommands.registerCommand("IntakeStop", new IntakeOverride(0, intake));
    NamedCommands.registerCommand("IntakeDrive", new IntakeDrive(driveTrain, 0, -0.5, 0, intake));
    NamedCommands.registerCommand("PivotMoveHalf", new ArmTest(arm, -0.3970532722));
    NamedCommands.registerCommand("PivotMoveLow", new ArmTest(arm, 0));
    NamedCommands.registerCommand("PivotMoveHigh", new ArmTest(arm, -0.211));
    NamedCommands.registerCommand("Shoot", new ShooterRollers(-0.5, shooter, intake, 25));
    // NamedCommands.registerCommand("WShoot", new ShooterRollers(1, shooter, intake, 80));
    NamedCommands.registerCommand("Feed", new Feed(-1, shooter, intake));
    NamedCommands.registerCommand("AutoFeed", new AutoFeed(-1, shooter, intake));
    NamedCommands.registerCommand("FastIntake", new IntakeDrive(driveTrain, 0, -0.8, 0, intake));
    NamedCommands.registerCommand("AutoAim", new AutoAim(driveTrain, arm));
    

    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    intake.setDefaultCommand(new IntakeRollers(-0.4, intake));

    configureBindings();

  }

  private void configureBindings() 
  {

    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    
    //Near Shot
    //-9.21875
    //-0.5

    //Truss shot
    // -17.355
    //-0.65


  //---------------------------COMP CONTROLS---------------------------------//
    
  //AUTO AIM TO TRUSS
  controller.leftBumper.onTrue(new AutoAim(driveTrain, arm));

  //AUTO AIM TO AMP
  if(!driveTrain.getSide())
  {
    controller.top.onTrue(AutoBuilder.pathfindToPose(new Pose2d(1.88, 7.79, new Rotation2d(-90)), new PathConstraints(4, 4,Units.degreesToRadians(180), Units.degreesToRadians(540))));
  }
  else
  {
    controller.top.onTrue(AutoBuilder.pathfindToPose(new Pose2d(14.71, 7.79, new Rotation2d(90)), new PathConstraints(4, 4,Units.degreesToRadians(180), Units.degreesToRadians(540))));
  }

  //TUNING SHOT (SOON TO BE TRUSS SHOT)
  controller.x.whileTrue(new SequentialCommandGroup(new ArmTest(arm, armPos),new ShooterRollers(speed, shooter, intake, threshold)));
  controller.x.onFalse(new Feed(-1, shooter, intake));

  //SUBWOOFER SHOT
  controller.rightBumper.whileTrue(new SequentialCommandGroup(new ArmTest(arm, -0.211),new ShooterRollers(-0.3, shooter, intake, 10)));
  controller.rightBumper.onFalse(new Feed(-1, shooter, intake));
  //AMP SHOT
  controller.y.whileTrue(new SequentialCommandGroup(new ArmTest(arm, -1),new ShooterRollers(-0.1, shooter, intake, 1)));
  controller.y.onFalse(new Feed(-1, shooter, intake));
  //HALF COURT SHOT
  controller.b.whileTrue(new SequentialCommandGroup(new ArmTest(arm, -0.4),new ShooterRollers(-0.3, shooter, intake, 15)));
  controller.b.onFalse(new Feed(-1, shooter, intake)); 

    //HOME POS
    joystick.seven.onTrue(new ArmTest(arm, 0));
    //SUBWOOFER POS
    joystick.eleven.onTrue(new ArmTest(arm, -0.211));
    //HALF COURT POS
    joystick.nine.onTrue(new ArmTest(arm, -0.469006));
    //AMP POS
    joystick.eight.onTrue(new ArmTest(arm, -1));
    //INTAKE
    joystick.ten.whileTrue(new IntakeOverride(-0.6, intake));
    //INTAKE STOP
    joystick.three.whileTrue(new IntakeOverride(0, intake));
    //BACKFEED
    joystick.four.whileTrue(new Reverse(0.6, intake));
    //SHOOTER STOP
    joystick.six.whileTrue(new StopShooter(shooter));

  }


  public Command getAutonomousCommand()
  {

  //--------------------------Pathplanner Autos-----------------------------//

  driveTrain.resetHeading();
  arm.setHome();

  return new PathPlannerAuto(autoChooser.getSelected());
  // return new PathPlannerAuto("testauto");

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

    // controller.x.onTrue(Commands.runOnce(() -> 
    // {
     
    //   Pose2d startPos = new Pose2d(driveTrain.getPose().getTranslation(), driveTrain.getRotation2d());
    //   double ydis = scoringPos.getY() - startPos.getY();
    //   double xdis = scoringPos.getX() - startPos.getX();
    //   Pose2d endPos = new Pose2d(startPos.getTranslation().plus(new Translation2d(xdis, ydis)), new Rotation2d());
    //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    //   PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints, 
    //     new PathConstraints(
    //       4.0, 4.0, 
    //       Units.degreesToRadians(360), Units.degreesToRadians(540)
    //     ),  
    //     new GoalEndState(0.0, startPos.getRotation())
    //   );

    //   // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //   path.preventFlipping = true;

    //   AutoBuilder.followPath(path).schedule();


    // }));




  }

}