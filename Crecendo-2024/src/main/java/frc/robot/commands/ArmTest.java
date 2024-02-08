// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmTest extends Command {
  double pos;
  frc.robot.subsystems.Test test;

  /** Creates a new ArmTest. */
  public ArmTest(frc.robot.subsystems.Test test, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pos = pos;
    this.test = test;

    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("hi");
    test.setPosition(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    test.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}