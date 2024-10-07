package frc.robot.commands;


import frc.robot.subsystems.Arm;
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
  public boolean stop =false;
    // public static Orchestra orchestra;
  

  
  public ShooterRollers(double speed, Shooter shooter, Intake intake, double threshold, boolean stop) {
    this.speed = speed;
    this.shooter = shooter;
    this.intake = intake;
    this.threshold = threshold;
    this.stop = stop;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {

  
    intake.setOverride(false);
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
  if(stop = true)
  {
   shooter.setIdle(false);
  }
  SmartDashboard.putNumber("Timer", curTime-startTime);
    if(shooter.atSetpoint(threshold) || (curTime - startTime) > 2)
    {
      if(!intake.hasNote())
      {
      isDone = true;  
      }
    
        intake.setOverride(true);
        if(DriverStation.isAutonomous())
        {
          intake.centerIntake(-0.6);
          intake.feed(-0.6);
          intake.fullWidth(-0.6);
        }
      }
  }

  

  @Override
  public void end(boolean interrupted) 
  {
//intake.feed(0);
// intake.setReversed(false);
  shooter.stopShooter();
  shooter.setIdle(true);
  intake.setOverride(false);
  }

  @Override
  public boolean isFinished() 
  {
    return isDone; 
  }
}