// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Test;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
// import frc.robot.commands.Auto;

public class RobotContainer {
  public static Drivetrain6390 driveTrain = new Drivetrain6390();
  public static frc.robot.subsystems.Test test = new Test();
  public LimeLight limelight = new LimeLight();

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
    // controller.y.onTrue(new AutoAlign(driveTrain, limelight, 0, 0, 0));
    // controller.x.whileTrue(new DebugCommand(driveTrain, limelight));
    // controller.a.whileTrue(new TurnAlign(driveTrain, limelight, 0));
    // controller.b.onTrue(new Test(driveTrain, limelight, 0,0,0));
    controller.a.onTrue(new InstantCommand(test::setHome));
    controller.b.onTrue(new ArmTest(test, 0.6));
  

   
  }
}




     //return new AutoAlign(driveTrain, limelight, 0,0, 0);
     //return new SequentialCommandGroup(new AutoAlign(driveTrain, limelight, 0,0, 0), new AutoAlign(driveTrain, limelight, 2, 0, 0));
    //  public Command getAutonomousCommand(){
    //   //"Middle 1 Game Piece and Balance" //Left Side 2 Game Piece //Bar 2 Game Piece
    //   //New Auto Options: "Left Side 2 Game Piece Far", "Right side", "Left Side Bump 1 Piece", "Right Side Bump 1 Piece"
    //   return AutoPathPlanner.runAuto("Right side red");
    //   //autoPathChooser.getSelected()
    // }
  
    
