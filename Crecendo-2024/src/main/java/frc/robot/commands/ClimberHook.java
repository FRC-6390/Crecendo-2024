package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
 
public class ClimberHook extends Command {
 
  public double speed;
  public double pos;
  public boolean isHit = false;
  public Climber climber = new Climber();

 
 

 
 
  public ClimberHook(double speed, Climber climber) {
    this.speed = speed;
    this.climber = climber;
    addRequirements(climber);
  }
 
  @Override
  public void initialize() {
    isHit = false;

 
  }
 
  @Override
  public void execute(){
    climber.setRollers(speed);
  }
  
   
 
 
 
  @Override
  public void end(boolean interrupted) {

  climber.setRollers(0);
  }
 
  @Override
  public boolean isFinished() {
   
    return false;
   
  }
}