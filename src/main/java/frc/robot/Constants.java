package frc.robot;

import java.util.Optional;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // not used
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, 8 * 0.0254), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  // used
  public static final double MAX_SPEED = 4.4196; // meters per second
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final Pose3d redHubPose = new Pose3d(Units.Inches.of(468.56), Units.Inches.of(158.32),
        Units.Inches.of(72.0), new Rotation3d());
    public static final Pose3d blueHubPose = new Pose3d(Units.Inches.of(152.56), Units.Inches.of(158.32),
        Units.Inches.of(72.0), new Rotation3d());

    public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    public static final Angle epsilonAngleToGoal = Degrees.of(1.0);

    public static final Pose3d getHubPose() {
      Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;

      return pose;
    }

    public static final Pose3d getFerryPose(Translation2d robotPose) {
      if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) {
        if (robotPose.getDistance(redFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose
            .getDistance(redFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return redFerryPoseOutpost;
        } else {
          return redFerryPoseDepot;
        }
      } else {
        if (robotPose.getDistance(blueFerryPoseDepot.getTranslation().toTranslation2d()) > robotPose
            .getDistance(blueFerryPoseOutpost.getTranslation().toTranslation2d())) {
          return blueFerryPoseOutpost;
        } else {
          return blueFerryPoseDepot;
        }
      }
    }

  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class IntakeConstants {
    public static final int INTAKE_ID = 16;
    public static final double INTAKE_SPEED = 0.7;
    public static final double OUTTAKE_SPEED = -0.7;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_KICKER_ID = 11;
    public static final int SHOOTER_RIGHT_ID = 9;
    public static final int SHOOTER_LEFT_ID = 17;
    public static final int HOOD_ID = 18;

    public static final double KICKER_SPEED = 3100;
    public static final double SHOOTER_SPEED = 5770; // RPM
    public static final double STOP = 0;
    public static final double IDLE = 0.1; // % voltage -1 --> 1

    public static double HOOD_UP_SPEED = 0.05;
    public static double HOOD_DOWN_SPEED = -0.05;

  }

  public static class HopperConstants {
    public static final int HOPPER_PUSHDOWN_ID = 19;
    public static final int HOPPER_TRANSFER_ID = 15;

    public static final double HOPPER_PUSHDOWN_SPEED = 0.7;
    public static final double HOPPER_TRANSFER_SPEED = 0.7;

    public static final double REVERSE_HOPPER_PUSHDOWN_SPEED = -0.7;
    public static final double REVERSE_HOPPER_TRANSFER_SPEED = -0.7;

    public static final int six_seven = 67;

  }

}