package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeRollers extends Command {
  
  double speed;
  double pos;
  

  public boolean isHomeSet;

  public IntakeRollers(double speed) {
    this.speed = speed;

  }



  
  @Override
  public void initialize() {
    isHomeSet = false;
  }

  @Override
  public void execute() {
  
    if(Intake.getBeamBreak() == true){
      Intake.setRollers(speed);
    }

    else{
      System.out.println(Intake.centerIntakeRoller.getPosition().refresh().getValueAsDouble());
      if (isHomeSet == false){
        Intake.centerIntakeRoller.setPosition(0);
        isHomeSet = true;
      }
      
          if (Intake.centerIntakeRoller.getPosition().refresh().getValueAsDouble() < 50.0){
            Intake.setRollers(speed);
          }
          else{

            Intake.setRollers(0);
          }
          }
        }


  

  @Override
  public void end(boolean interrupted) {
  if(Intake.getBeamBreak()==true){
    Intake.setRollers(0); 
  }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}