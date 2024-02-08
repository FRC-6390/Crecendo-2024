// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Test;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Auto;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.auto.JanusAuto;
import frc.robot.commands.auto.TurnAlign;

public class RobotContainer {
  public static Drivetrain6390 driveTrain = new Drivetrain6390();
  public static frc.robot.subsystems.Test test = new Test();
  public static LimeLight limelight = new LimeLight();

  public static DebouncedController controller = new DebouncedController(0);

  public RobotContainer() {
    //driveTrain.shuffleboard();
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    SmartDashboard.putNumber("Heading", driveTrain.getHeading());
    SmartDashboard.putNumber("Rotation2D", driveTrain.getRotation2d().getDegrees());
    configureBindings();
  }

  private void configureBindings() 
  {
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    controller.y.onTrue(new AutoAlign(driveTrain, limelight, 0, 0, 178, 0.06));
    controller.x.whileTrue(new DebugCommand(driveTrain, limelight));
    //controller.a.whileTrue(new TurnAlign(driveTrain, limelight, 0));
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
    return new SequentialCommandGroup
    (
       
    new AutoDrive(driveTrain, limelight, 0, -0.75, 0),
    new AutoAlign(driveTrain, limelight, -2.75, 0, 0 , 0),
    new AutoAlign(driveTrain, limelight, 6.9, 20.77, 0, 0.03),
    new AutoAlign(driveTrain, limelight, -2.75, 0, 0 , 0),
    new TurnAlign(driveTrain, limelight, 0)
    
    );



    //Janus Test Auto
    //return new JanusAuto(driveTrain, Constants.AUTO.TEST_THETA_AUTO_PATH.build());
      
        
  
  }
}
