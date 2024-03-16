package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {

  public static final class Analog {
    public static final class SwerveModule {
      public static final class Channel_ID {
        public static final int FL = 1;
      }
    }
  }
  public static final class CanId {

    public static final class CanCoder {
      public static final int GYRO = 10;
      public static final int FL = 43;
      public static final int FR = 44;
      public static final int BL = 42;
      public static final int BR = 41;
      public static final int SMALL_ARM = 22;
      public static final int BIG_ARM = 21;
    }

    public static final class Swerve {
      public static final class Angle {
        public static final int FL = 15;
        public static final int FR = 16;
        public static final int BL = 11;
        public static final int BR = 18;
      }

      public static final class Drive {
        public static final int FL = 17;
        public static final int FR = 12;
        public static final int BL = 13;
        public static final int BR = 14;
      }
    }

    public static final class Arm {
      public static final class Motor {
        public static final int ARM_1 = 1;
        public static final int ARM_2 = 2;
        public static final int SHOOTER_1 = 3;
        public static final int SHOOTER_2 = 4;
        public static final int INTAKE = 5;
        public static final int CLIMBER = 20; 
      }
    }

    public static final class Angle {
      public static final int FL = 15;
      public static final int FR = 16;
      public static final int BL = 11;
      public static final int BR = 18;
    }

    public static final class Motor {
      public static final int FL = 17;
      public static final int FR = 12;
      public static final int BL = 13;
      public static final int BR = 14;
    }
  }

  public static final class Position {
    public static final class MainArm {
      public static final class AUTON {
        public static final double CLOSE = 10;
        public static final double SIDE = 11.942383; //Get Rid of
        public static final double MIDDLESIDE = 20;
      }

      public static final double AMP = 49.416016;
      public static final double SHOOTING = 8.5;
      public static final double PICKUP = 0;
      public static final double CLIMBER = 0;
    }
    
    public static final class Climber { // PLACEHOLDERS
      public static final double UP = 1;
      public static final double DOWN = 0;
    }
  }

  public static final class Inverted {
    // This is for motors
    public static final boolean FL = false;
    public static final boolean FR = true;
    public static final boolean BL = false;
    public static final boolean BR = true;
  }

  public static final class Canbus {
    public static final String DEFAULT = "rio";
    public static final String DRIVE_TRAIN = "drivetrain";
  }

  public static final class Sensor {
    public static final int CUBE_CHANNEL = 2;
    public static final int CONE_CHANNEL = 3;
  }

  public static final class Gamepad {

    public static final class Controller {
      public static final int DRIVE = 0;
      public static final int ARM = 1;
    }

    public static final class Axis {
      public static final int LEFT_X = 0;
      public static final int LEFT_Y = 1;
      public static final int RIGHT = 4;
    }

    public static final class Trigger {
      public static final int LEFT = 2;
      public static final int RIGHT = 3;
    }

    public static final class Button {
      public static final int A = 1;
      public static final int B = 2;
      public static final int X = 3;
      public static final int Y = 4;
      public static final int LEFT = 5;
      public static final int RIGHT = 6;
      public static final int BACK = 7;
      public static final int START = 8;
      public static final int L_JOYSTICK = 9;
      public static final int R_JOYSTICK = 10;
    }
  }

  public static final class WheelOffset {
    // Converting rotations to radians
    public static final double FL = (0.504639) * 2 * Math.PI;
    public static final double FR = (0.302002) * 2 * Math.PI;
    public static final double BL = (0.088623) * 2 * Math.PI;
    public static final double BR = (0.169189) * 2 * Math.PI;
  }

  public static final class MotorGearRatio {
    public static final double DRIVE = 6.75;
    public static final double ANGLE =
        150.0/7.0;//12.8; // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final int BIG = 10;
    public static final int SMALL = 7;
  }

  public static final class Measurement {
    // trackWidth - lateral distance between pairs of wheels on different sides of
    // the robot
    // wheelBase - distance between pairs of wheels on the same side of the robot
    // driveBaseRadius - distance from robot center to furthest module.
    // radiusFactor - to account for real world factors of the wheel radius
    // THIS IS IMPORTANT FOR A RECTANGULAR ROBOT
    // In meters
    public static final double TRACK_WIDTH = 18.75 * 0.0254; // Inches to meters
    public static final double WHEEL_BASE = 22.75 * 0.0254; // Inches to meters
    public static final double WHEELRADIUS = 2.003 * 0.0254; //2024 robot radius from inches to meters
    public static final double DRIVEBASERADIUS = 14.942 * 0.0254; // Inches to meters
    public static final double WHEELRADIUSFACTOR = 1.155;
  }

  public static final class Speed {
    public static final double ROBOT_SPEED_DIVISOR =
        1; // what the max speed should be divided by, 1 is max power
    public static final double SHOOTER = 65; // speed of the arms when adjusting manually in rotations per second
    public static final double INTAKE = 25; // rotations per second
    public static final double ARM = 30; // rotations per second
    public static final double CLIMBER = 70;
  }

  public static final class Distance {
    // CHARGE STATION COMMUNITY DISTANCE:
    public static final double TO_BLUE_CHARGE_STATION = 96.4694981;
    public static final double TO_RED_CHARGE_STATION = 99;
    public static final double TO_CENTER_COMMUNITY = 100;
    public static final double TO_OUTSIDE_COMMUNITY = 87.30208217;
    // 1.832716884 is the number of inches per 1 encoder value
    // ~80 (plus offset) to the center of the charge station for robot
    // ~160 is the distance to leave the community plus some extra cushion
  }

  public static final class AngleInaccuracy {
    public static final double MAX = Math.PI / 24;
  }

  public static final class PidGains {
    public static final class PathPlanner {
      public static final PID translation = new PID(9, 0, 0.0);
      public static final PID rotation = new PID(.4, 0, .004);
    }
    
    public static final class Limelight {
      public static final PID DRIVE_CONTROLLER = new PID(0.0025, 0, 0);
      public static final PID TURN_CONTROLLER = new PID(.1, 0, 0.0001);;
      public static final PID SCORE_DRIVE_CONTROLLER = new PID(0.0056, 0, 0);
    }

    public static final class SwerveCommand {
      public static final PID X_CONTROLLER = new PID(0.0001, 0, 0);
      public static final PID Y_CONTROLLER = new PID(0.0001, 0, 0);
    }

    public static final class DriveDistance {
      public static final PID DRIVE_DISTANCE = new PID(0.1, 0, 0);
    }

    public static final class GetOnChargeStation {
      public static final PID GET_ON_CHARGE_STATION = new PID(0.1, 0, 0);
    }

    public static final class SwerveDistance {
      public static final PID SWERVE_DISTANCE = new PID(0.1, 0, 0);
      public static final PID SWERVE_COMMAND_XCONTROLLER = new PID(1, 0, 0);
      public static final PID SWERVE_COMMAND_YCONTROLLER = new PID(1, 0, 0);
    }

    public static final class TurnAngle {
      public static final double[] TURN_ANGLE = {Math.PI / 6, 0, 0};
    }

    public static final class SwerveModule {
      public static final PID DRIVE_PID_CONTROLLER = new PID(.5, 0, 0);
      public static final PID TURNING_PID_CONTROLLER = new PID(7, 0, 0.3, 0);
    }
  }
}
