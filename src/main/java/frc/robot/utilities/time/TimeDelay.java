package main.java.frc.robot.utilities.time;

import edu.wpi.first.wpilibj.Timer;

public class TimeDelay {
    public double startTime;

    public TimeDelay(){


    }


    public void start(){
        startTime = Timer.getFPGATimestamp();

    }


    public boolean isPassed(double time){
       double current = Timer.getFPGATimestamp();
        if(current-startTime>time){
           return true;
        }
        return false;
    }
}


