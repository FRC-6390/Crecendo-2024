// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.TurnAlign;
import frc.robot.utilities.auto.JanusConfig;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModuleConfig;

public interface Constants {
    public interface AUTO{
        
        PIDConfig XY_PID_CONFIG = new PIDConfig(0.7, 0, 0);
        PIDConfig THETA_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);
        PIDConfig ROLL_PITCH_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);
        PIDConfig ALIGN_XY_PID_CONFIG = new PIDConfig(1, 0, 0);
        PIDConfig ALIGN_THETA_PID_CONFIG = new PIDConfig(0.1, 0, 0).setContinuous(-Math.PI, Math.PI);

        JanusConfig CONFIG = new JanusConfig(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND, XY_PID_CONFIG, THETA_PID_CONFIG);

        JanusRouteFactory RIGHT_SIDE_SEGMENT_1 = new JanusRouteFactory(CONFIG).to(-1, 0);

        JanusRouteFactory TEST_X_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0).to(-1, 0);
        JanusRouteFactory TEST_Y_AUTO_PATH = new JanusRouteFactory(CONFIG).to(0, 1).to(0, -1).to(0, 0);
        JanusRouteFactory TEST_XY_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0).to(-1, 0).to(0, 0).to(0, 1).to(0, -1).to(0, 0).to(1,1).to(-1,-1).to(1,-1).to(-1,1).to(0,0);
        JanusRouteFactory TEST_THETA_AUTO_PATH = new JanusRouteFactory(CONFIG).to(0, 0,90).to(0, 0, 270).to(0, 0, 180).to(0, 0, 0);
        JanusRouteFactory TEST_MOVEMENT_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 90).to(-1, 0).to(0, 0, 0).to(0, 1, 180).to(0, -1).to(0, 0, 0).to(1,1, 270).to(-1,-1, 90).to(1,-1, 180).to(-1,1, 90).to(0,0, 0);
        JanusRouteFactory TEXT_COMMAND_1_AUTO_PATH = new JanusRouteFactory(CONFIG).run(null);
        JanusRouteFactory TEXT_COMMAND_2_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 0).run(null).to(0, 0, 0);
        JanusRouteFactory TEXT_COMMAND_3_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 0).run(null, true).to(0, 0, 0);

        TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);
    }
    

    public interface DRIVETRAIN{

        String CANBUS = "can";

        int PIGEON = 0;

        int REV_PDH = 1;

        Translation2d[] SWERVE_MODULE_LOCATIONS = {ROBOT.FRONT_LEFT, ROBOT.FRONT_RIGHT, ROBOT.BACK_LEFT, ROBOT.BACK_RIGHT};

        int FRONT_LEFT_DRIVE = 1;
        int FRONT_LEFT_ROTATION = 5;
        int FRONT_LEFT_ENCODER = 9;
        int FRONT_RIGHT_DRIVE = 2;
        int FRONT_RIGHT_ROTATION = 6;
        int FRONT_RIGHT_ENCODER = 10;
        int BACK_LEFT_DRIVE = 3;
        int BACK_LEFT_ROTATION = 7;
        int BACK_LEFT_ENCODER = 11;
        int BACK_RIGHT_DRIVE = 4;
        int BACK_RIGHT_ROTATION = 8;
        int BACK_RIGHT_ENCODER = 12;


        //Below is what Mathias and I finished with 
        double FRONT_LEFT_OFFSET = 0.39111328125; 
        double FRONT_RIGHT_OFFSET =  -0.99755859375; 
        double BACK_LEFT_OFFSET = 0.11181640625;
        double BACK_RIGHT_OFFSET =  0.6298828125;



        SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_LEFT_DRIVE, false, FRONT_LEFT_ROTATION, false, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_RIGHT_DRIVE, false, FRONT_RIGHT_ROTATION, false, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(BACK_LEFT_DRIVE, false, BACK_LEFT_ROTATION, false, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(BACK_RIGHT_DRIVE, false, BACK_RIGHT_ROTATION, false, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET, CANBUS);

    }

    public interface SWERVEMODULE {
        double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_SPEED_METERS_PER_SECOND_SQUARED = Units.feetToMeters(13.5) * Units.feetToMeters(13.5);
        double MAX_ANGULAR_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_ACCELERATION_METERS_PER_SECOND = 2.75;
        double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND = 3.85;
        double ROTATION_GEAR_RATIO = 1d/(150d/7d);
        double DRIVE_GEAR_RATIO = 1d/(8.14);
        double ROTATION_ENCODER_CONVERSION_RADIANS = ROTATION_GEAR_RATIO * 2 * Math.PI;
        double ROTATION_ENCODER_CONVERSION_RADIANS_PER_SECOND = ROTATION_ENCODER_CONVERSION_RADIANS / 60;
        double DRIVE_ENCODER_CONVERSION_METERS = (DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS); //the 0.625 is a quick fix to correct the odometry
        double DRIVE_ENCODER_CONVERSION_METERS_PER_SECOND = DRIVE_ENCODER_CONVERSION_METERS / 60;
        PIDConfig ROTATION_PID = new PIDConfig(0.5, 0, 0).setContinuous(-Math.PI, Math.PI);
    }

    public interface INTAKE {
        int INTAKE_MOTOR = 21;//was 23, changed for testing
        int LIMIT_SWITCH = 0;
    }

    

    public interface TEST{
        int ARM_MOTOR = 14;
        double ARM_MAX = -7.284;
        PIDConfig PID_config = new PIDConfig(0.07, 0, 0);
    }
    
    public interface ROBOT {
        double TRACKWIDTH_METERS = 0.61;
        double WHEELBASE_METERS = 0.61;

        int CANDLE_ID = 22; //unkown tbd

        Translation2d FRONT_LEFT = new Translation2d(TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d FRONT_RIGHT = new Translation2d(TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        Translation2d BACK_LEFT = new Translation2d(-TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d BACK_RIGHT = new Translation2d(-TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        int BLINKIN_PORT = 1;
        
        
    }

    public enum APRILTAGS {
    
        INVALID(0, 0),
        
        
        RED_SOURCE_RIGHT(1, -45),
        RED_SOURCE_LEFT(2,-45),
        
        RED_SPEAKER_RIGHT(3, 0),
        RED_SPEAKER_MID(4, 0),

        RED_AMP(5, -90),

        BLUE_AMP(6, 90);



        // Blue right X: 138cm - 41cm = 0.97m Y: 65cm+23.5cm + 5cm + 5cm = 0.985m
        // Blue middle X: 138cm - 41cm = 0.97m Y: 191cm + 2.5cm+ 47cm+ + 5cm + 23.5cm = 2.69m
        // Blue left X: 138cm - 41cm = 0.97m Y: 191cm + 168cm + 2.5cm + 47cm + 5cm + 23.5cm = 4.37m
        
        // Red loading X: 36cm = 0.36m Y: 550cm + 117cm = 6.67m
        // Blue loading X: 1654cm - 36cm = 16.18m Y: 550cm + 117cm = 6.67m
        
        // Red right X: 1654cm - 138cm + 41cm = 15.57m Y: 65cm+23.5cm + 5cm + 5cm = 0.985m
        // Red middle X: 1654cm - 138cm + 41cm = 15.57m Y: 191cm + 2.5cm+ 47cm+ + 5cm + 23.5cm = 2.69m
        // Red left X: 1654cm - 138cm + 41cm = 15.57m Y: 191cm + 168cm + 2.5cm + 47cm + 5cm + 23.5cm = 4.37m

        long id;
        double heading;
        private APRILTAGS(long id, double heading){
            this.id = id;
            this.heading = heading;
            
        }
       
        public static APRILTAGS getByID(long id){
            for (APRILTAGS tag : values()) {
                if(tag.getID() == id) return tag;
            }
            return INVALID;
        }

        public long getID(){
            return id;
        }

        public double getRotation()
        {
            return heading;
        }

    }


  

    }


