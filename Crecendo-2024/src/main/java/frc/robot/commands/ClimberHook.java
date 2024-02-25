package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
 
public class ClimberHook extends Command {
 
  public double speed;
  public double pos;

  public Climber climber = new Climber();

 
 

 
 
  public ClimberHook(double speed, Climber climber) {
    this.speed = speed;
    this.climber = climber;
    addRequirements(climber);
  }
 
  @Override
  public void initialize() {


 
  }
 
  @Override
  public void execute(){
    climber.setRollers(speed);
  }
  
   
 
 
 
  @Override
  public void end(boolean interrupted) {
  // if(Intake.getLowerBeamBreak()==false){
  //   Intake.setRollers(0,2);
  // }
  climber.setRollers(0);
  }
 
  @Override
  public boolean isFinished() {
   
    return false;
   
  }
}