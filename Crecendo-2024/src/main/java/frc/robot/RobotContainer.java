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

import frc.robot.commands.auto.TurnAlign;
import frc.robot.commands.ClimberHook;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    autoChooser.addOption("StageSideExtraFast", "StageSideExtraFast");
    autoChooser.addOption("StageSideRed", "StageSideRed");
    
    SmartDashboard.putData("AutoChoose", autoChooser);
    NamedCommands.registerCommand("TurnAlign", new TurnAlign(driveTrain, limelight, 0));
    NamedCommands.registerCommand("TurnCommand", new TurnCommand(driveTrain, 0));
    NamedCommands.registerCommand("Turn25", new TurnAlign(driveTrain,limelight, -0.5));
    NamedCommands.registerCommand("IntakeRollers", new IntakeRollers(-0.6, intake));
    NamedCommands.registerCommand("IntakeDrive", new IntakeDrive(driveTrain, 0, -0.5, 0, intake));
    NamedCommands.registerCommand("PivotMoveHalf", new ArmTest(arm, -0.5));
    NamedCommands.registerCommand("PivotMoveLow", new ArmTest(arm, -1));
    NamedCommands.registerCommand("PivotMoveHigh", new ArmTest(arm, -0.25));
    NamedCommands.registerCommand("Shoot", new ShooterRollers(1, shooter, intake, 80));
    // NamedCommands.registerCommand("WShoot", new ShooterRollers(1, shooter, intake, 80));
    NamedCommands.registerCommand("Feed", new Feed(-1, shooter, intake));
    NamedCommands.registerCommand("AutoFeed", new AutoFeed(-1, shooter, intake));
    NamedCommands.registerCommand("FastIntake", new IntakeDrive(driveTrain, 0, -0.8, 0, intake));
    
    NamedCommands.registerCommand("AutoAim", new AutoAim(driveTrain, arm));
    

    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    // intake.setDefaultCommand(new IntakeRollers(-0.6, intake));
    

    // SmartDashboard.putNumber("Heading", driveTrain.getHeading());
    // SmartDashboard.putNumber("Rotation2D", driveTrain.getRotation2d().getDegrees());
    // //ShuffleboardTab Tab = Shuffleboard.getTab("COMP");
    


    

    configureBindings();

  }

  private void configureBindings() 
  {

    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    
    //Near Shot
    //-9.21875
    //-0.5



  //---------------------------COMP CONTROLS---------------------------------//
    
  controller.leftBumper.onTrue(new TurnAlign(driveTrain, limelight, 0));
  controller.rightBumper.whileTrue(new ShooterRollers(-0.5, shooter, intake, 60));
  controller.rightBumper.onFalse(new Feed(-1, shooter, intake));
    Command pathFind; 
                        
    if(!driveTrain.getSide())
    {
      pathFind = AutoBuilder.pathfindToPose(scoringPos, new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(540)));
    }
    else
    {
      pathFind = AutoBuilder.pathfindToPose(scoringPosR, new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(540)));
    }

    controller.b.onTrue(pathFind);

    controller.y.onTrue
    (
      AutoBuilder.pathfindToPose
      (
        new Pose2d(1.24, 5.52, new Rotation2d(3.142)), 
        new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(540))
      )
    );
    

    //joystick.eight.onTrue(new ArmTest(arm, 0.08));
    joystick.two.onTrue(new AutoAim(driveTrain, arm));
    joystick.seven.onTrue(new ArmTest(arm, -1));
    joystick.eleven.onTrue(new ArmTest(arm, -0.211));
    joystick.nine.onTrue(new ArmTest(arm, -0.5));
    //joystick.ten.whileTrue(new ClimberHook(0.2, climber));
    //joystick.twelve.whileTrue(new
    //  ClimberHook(-0.2 , climber);
    
    
    joystick.three.whileTrue(new StopIntake(0, intake));
    joystick.four.whileTrue(new Reverse(0.6, intake));
   // joystick.six.whileTrue(new StopShooter(shooter));
   // joystick.five.whileTrue(new ShooterRollers (1, shooter, intake, 70));
    joystick.five.onFalse(new Feed(-1, shooter, intake));

    //RAND
    // joystick.two.whileTrue(new ShooterRollers(10, shooter, intake));
    // joystick.two.onFalse(new Feed(-1, shooter, intake));

  }


  public Command getAutonomousCommand()
  {

  //--------------------------Pathplanner Autos-----------------------------//

  driveTrain.resetHeading();
  arm.setHome();

  return new PathPlannerAuto(autoChooser.getSelected());

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