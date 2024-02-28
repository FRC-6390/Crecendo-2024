package frc.robot.commands;


import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterRollers extends Command{

  public double speed;
  public Shooter shooter;
  public boolean isDone;
  public int numPresses = 0;
  
  public ShooterRollers(double speed, Shooter shooter) {
    this.speed = speed;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // isDone = false;
    // numPresses++;
    // if(numPresses > 2)
    // {
    //   numPresses = 1;
    // }

  }


  @Override
  public void execute() 
  {
  // if(numPresses == 1)
  // {
    shooter.setPID(speed);
    System.out.println("SetpointAt: " + shooter.atSetpoint());
    System.out.println("Velocity: " + shooter.getRotorVelocity());
  //   if(shooter.atSetpoint())
  //   {
  //     isDone = true;
  //   }
  // }
  // else if(numPresses == 2)
  // {
  //   if(Intake.getUpperBeamBreak())
  //   {
  //     intake.feed(-0.8);
  //   }
  //   else
  //   {
  //     intake.feed(0);
  //     isDone = true;
  //   }
  // }
  
  }

  @Override
  public void end(boolean interrupted) 
  {
  // if(numPresses == 2)
  // {
    shooter.setPID(0);
    
  //intake.feed(0);
  //} 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}