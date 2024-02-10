package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.auto.JanusRoute;

public class JanusAuto extends Command {

    private Drivetrain6390 driveTrain;
    private JanusRoute route;

    public JanusAuto(Drivetrain6390 driveTrain, JanusRoute route) {
        this.driveTrain = driveTrain;
        this.route = route;
        addRequirements(driveTrain);
    }
  
    @Override
    public void initialize() {
        route.init(driveTrain::getPose);
    }
  
    @Override
    public void execute() {

        if(route.isCommand()){
            route.runCommand();
        }else{
            System.out.println(route.calculate());
            driveTrain.drive(route.calculate());
        }
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }