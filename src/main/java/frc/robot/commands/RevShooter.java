package frc.robot.commands;


import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class RevShooter extends Command{

  public double speed;
  public Shooter shooter;
  public boolean isDone;
  public int numPresses = 0;
public Intake intake;
  public double startTime;
  // public double threshold;
  
  public RevShooter(double speed, Shooter shooter, Intake intake) {
    this.speed = speed;
    this.shooter = shooter;
    this.intake = intake;
    // this.threshold = threshold;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
   
isDone = false;
    startTime = System.currentTimeMillis();
    // numPresses++;
    // if(numPresses > 2)
    // {
    //   numPresses = 1;
    // }

  }


  @Override
  public void execute() 
  {
  // double curTime = System.currentTimeMillis();
  shooter.setRollers(speed);


// if(shooter.atSetpoint() || (curTime - startTime) > 3000)

  // if(Math.abs(shooter.getRotorVelocity()) >= threshold)
  //     {
  //       isDone = true;  
  //       intake.feed(-1);
  //       intake.centerIntake(-0.6);
  //       //shooter.stopShooter();
  //     }
    System.out.println("SetpointAt: " + shooter.atSetpoint());
    System.out.println("Velocity: " + shooter.getRotorVelocity());
  System.out.println("Beam" + Intake.getUpperBeamBreak());
  }

  @Override
  public void end(boolean interrupted) 
  {
//intake.feed(0);
  shooter.stopShooter();
  
}

  @Override
  public boolean isFinished() {
    // if(DriverStation.isAutonomous())
    // {
    //   return isDone;
    // }
    // else{
    return false;
    //}
  }
}