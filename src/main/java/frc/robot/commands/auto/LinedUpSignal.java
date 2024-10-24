// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimeLight.LedMode;

public class LinedUpSignal extends Command {
  public LimeLight limelight;
  public Intake intake; 

  /** Creates a new LinedUpSignal. */
  public LinedUpSignal(LimeLight limeLight, Intake intake) {
    this.limelight = limeLight;// Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
  if(!limelight.hasValidTarget())
  {
    limelight.setLedMode(LedMode.OFF);
  }
  else
  {
    limelight.setLedMode(LedMode.ON);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
