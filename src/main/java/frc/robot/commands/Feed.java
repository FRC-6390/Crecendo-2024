package frc.robot.commands;


import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Feed extends Command{

  public double speed;
  public Shooter shooter;
  public boolean isDone;
  public int numPresses = 0;
  public Intake intake;
  public double startTime;
  
  public Feed(double speed, Shooter shooter, Intake intake) {
    this.speed = speed;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter, intake);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    isDone = false;
    // numPresses++;
    // if(numPresses > 2)
    // {
    //   numPresses = 1;
    // }

  }


  @Override
  public void execute() 
  {
    double curTime = System.currentTimeMillis();
    // System.out.println(startTime);
    // System.out.println(startTime - curTime);
    
    intake.feed(-1);
    intake.centerIntake(-0.6);
    intake.fullWidth(-0.6);
    if((curTime - startTime) > 1500)
    {
      //shooter.setPID(0);
      shooter.stopShooter();
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) 
  {  
  //intake.feed(0);
  shooter.stopShooter();
  intake.feed(0);
  intake.centerIntake(0);
  intake.fullWidth(0);
  CommandScheduler.getInstance().schedule(new ArmTest(RobotContainer.arm, 0));
  //shooter.setPID(0);
  //shooter.setPID(0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}