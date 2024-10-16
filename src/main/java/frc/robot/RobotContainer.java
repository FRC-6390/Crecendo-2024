// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.commands.auto.TurnAlign;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoFeed;
import frc.robot.commands.auto.TurnCommand;

public class RobotContainer {
  
  public static Arm arm;
  public static LimeLight limelight = new LimeLight();
  public static Drivetrain6390 driveTrain = new Drivetrain6390(limelight);
  public static Climber climber = new Climber();
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Pose2d scoringPos = new Pose2d(1.24, 5.52, new Rotation2d());
  public static Pose2d scoringPosR = new Pose2d(15.26, 5.52, new Rotation2d());
    
  public static DebouncedController controller = new DebouncedController(0);
  private DebouncedJoystick joystick = new DebouncedJoystick(1);

 
  public SendableChooser<String> autoChooser = new SendableChooser<>(); 

  public RobotContainer() {
    arm = new Arm(joystick);
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

    SmartDashboard.putData("AutoChoose", autoChooser);
    
    NamedCommands.registerCommand("TurnAlign", new TurnAlign(driveTrain, limelight, 0));
    NamedCommands.registerCommand("TurnCommand", new TurnCommand(driveTrain, 0));
    NamedCommands.registerCommand("Turn25", new TurnAlign(driveTrain,limelight, -0.5));
    NamedCommands.registerCommand("IntakeRollers", new IntakeRollers2(intake, -0.6, true));
    NamedCommands.registerCommand("PivotMoveHalf", new ArmTest(arm, -0.3970532722));
    NamedCommands.registerCommand("PivotMoveLow", new ArmTest(arm, 0));
    NamedCommands.registerCommand("PivotMoveHigh", new ArmTest(arm, -0.211));
    NamedCommands.registerCommand("Shoot", new ShooterRollers(-0.5, shooter, intake, 25, false));
    NamedCommands.registerCommand("Feed", new Feed(-1, shooter, intake));
    NamedCommands.registerCommand("AutoFeed", new AutoFeed(-1, shooter, intake));
    NamedCommands.registerCommand("AutoAim", new AutoAim(driveTrain, arm));
    

    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    intake.setDefaultCommand(new IntakeRollers2(intake, -0.4, true));

    configureBindings();

  }

  private void configureBindings() 
  {

    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));


  //---------------------------COMP CONTROLS---------------------------------//
    
  //AUTO AIM
  controller.leftBumper.onTrue(new AutoAim(driveTrain, arm));

  //SUBWOOFER SHOT
  controller.rightBumper.whileTrue(new ShooterRollers(-0.5, shooter, intake, 30, false));
  controller.rightBumper.onFalse(new Feed(-1, shooter, intake));
  //AMP SHOT
  controller.y.whileTrue(new ShooterRollers(-0.1, shooter, intake, 1, false));
  controller.y.onFalse(new Feed(-1, shooter, intake));
  //HALF COURT SHOT
  controller.b.whileTrue(new ShooterRollers(-0.5, shooter, intake, 30, false));
  controller.b.onFalse(new Feed(-1, shooter, intake)); 

    //HOME POS
    joystick.seven.onTrue(new ArmTest(arm, 0));
    //SUBWOOFER POS
    joystick.eleven.onTrue(new ArmTest(arm, -0.211));
    //HALF COURT POS
    joystick.nine.onTrue(new ArmTest(arm, -0.469006));
    //AMP POS
    joystick.eight.onTrue(new ArmTest(arm, -1));
    //BACKFEED
    joystick.four.whileTrue(new IntakeRollers2(intake, -0.6, false));
    joystick.three.whileTrue(new IntakeRollers2(intake, 0.6, false));

  }


  public Command getAutonomousCommand()
  {
  //--------------------------Pathplanner Autos-----------------------------//

  driveTrain.resetHeading();
  arm.setHome();

  return new PathPlannerAuto(autoChooser.getSelected());
  }

}