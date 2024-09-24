package frc.robot.commands;


import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterRollers extends Command{

  public double speed;
  public Shooter shooter;
  public boolean isDone;
  public int numPresses = 0;
public Intake intake;
  public double startTime;
  public double threshold;
    // public static Orchestra orchestra;
  

  
  public ShooterRollers(double speed, Shooter shooter, Intake intake, double threshold) {
    this.speed = speed;
    this.shooter = shooter;
    this.intake = intake;
    this.threshold = threshold;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
    
  // orchestra = new Orchestra();
   
isDone = false;
    startTime = Timer.getFPGATimestamp();
    // numPresses++;
    // if(numPresses > 2)
    // {
    //   numPresses = 1;
    // }

  }


  @Override
  public void execute() 
  {
  double curTime =Timer.getFPGATimestamp();
  shooter.setRollers(speed);
  
  // orchestra.loadMusic("output.chrp");
  // orchestra.addInstrument(shooter.sh)


// if(shooter.atSetpoint() || )

  if((curTime - startTime) > 3)
      {
        isDone = true;  
        intake.feed(-0.75);
        intake.centerIntake(-0.6);
        intake.fullWidth(-0.6);
        //shooter.stopShooter();
      }
    System.out.println("SetpointAt: " + shooter.atSetpoint());
    System.out.println("Velocity: " + shooter.getRotorVelocity());
  System.out.println("Beam" + Intake.getUpperBeamBreak());
  }

  @Override
  public void end(boolean interrupted) 
  {
//intake.feed(0);
  //shooter.stopShooter();
  System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
      }

  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous())
    {
      return isDone;
    }
    else{
    return false;
    }
  }
}