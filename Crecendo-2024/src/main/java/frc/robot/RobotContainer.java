// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
// import frc.robot.commands.Auto;

public class RobotContainer {
  public static Drivetrain6390 driveTrain = new Drivetrain6390();

  public LimeLight limelight = new LimeLight();

  public static DebouncedController controller = new DebouncedController(0);

  public RobotContainer() {
    driveTrain.shuffleboard();
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));
    SmartDashboard.putNumber("Heading", driveTrain.getHeading());
    SmartDashboard.putNumber("Rotation2D", driveTrain.getRotation2d().getDegrees());
    configureBindings();
  }

  private void configureBindings() 
  {
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    controller.y.onTrue(new AutoAlign(driveTrain, limelight, 0, 0, 0));
    controller.x.whileTrue(new DebugCommand(driveTrain, limelight));
    controller.a.whileTrue(new TurnAlign(driveTrain, limelight, 0));
    controller.b.onTrue(new Test(driveTrain, limelight, 0,0,0));
  } 


  public Command getAutonomousCommand(){
     return new AutoAlign(driveTrain, limelight, 0,0, 0);
     //return new SequentialCommandGroup(new AutoAlign(driveTrain, limelight, 0,0, 0), new AutoAlign(driveTrain, limelight, 2, 0, 0));
  }
}
