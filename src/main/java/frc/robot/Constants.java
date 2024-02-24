package frc.robot;

/*
 * This class should hold any static configuration data about the robot
 * All variables in this class should be marked public static and final
 */
public final class Constants {
  public static final class CanId {

    public static final class CanCoder {
      public static final int GYRO = 0;
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
    public static final class MainArm { // THE 0'S ARE CURRENTLY PLACEHOLDERS
      public static final double AMP = 0;
      public static final double SHOOTING = 0;
      public static final double PICKUP = 0;
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
    public static final String DRIVE_TRAIN = "rio";
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
  }

  public static final class WheelOffset {
    // Converting rotations to radians
    public static final double FL = (0.612305) * 2 * Math.PI;
    public static final double FR = (0.500977) * 2 * Math.PI;
    public static final double BL = (0.083008) * 2 * Math.PI;
    public static final double BR = (0.443115) * 2 * Math.PI;
  }

  public static final class MotorGearRatio {
    public static final double DRIVE = 8.14;
    public static final double ANGLE =
        12.8; // https://www.swervedrivespecialties.com/products/mk4-swerve-module
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
    public static final double WHEELBASE = 22.75 * 0.0254; // Inches to meters
    public static final double WHEELRADIUS = 2.003 * 0.0254; //2024 robot radius from inches to meters
    public static final double DRIVEBASERADIUS = 14.942 * 0.0254; // Inches to meters
    public static final double WHEELRADIUSFACTOR = 1.1;
  }

  public static final class Speed {
    public static final double ROBOT_SPEED_DIVISOR =
        1; // what the max speed should be divided by, 1 is max power
    public static final double SHOOTER = 100; // speed of the arms when adjusting manually in rotations per second
    public static final double INTAKE = 25; // rotations per second
    public static final double ARM = 15; // rotations per second

  }

  public static final class Distance {}

  public static final class AngleInaccuracy {
    public static final double MAX = Math.PI / 24;
  }

  public static final class PidGains {
    public static final class PathPlanner {
      public static final PID translation = new PID(5.5, 0.055, 0.02);
      public static final PID rotation = new PID(1, 0, 0);
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
      public static final PID TURNING_PID_CONTROLLER = new PID(.2, 0, 0);
    }
  }
}
