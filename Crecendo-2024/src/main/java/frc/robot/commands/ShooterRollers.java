package frc.robot.commands;


import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;


public class ShooterRollers extends Command{

  private double velocity;
  public static Shooter shooter;
  
  public ShooterRollers(double velocity, Shooter shooter) {
    this.velocity = velocity;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    
    

  }

  double speed = 0;

  @Override
  public void execute() {

    
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRollers(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}