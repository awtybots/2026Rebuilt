package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;

/**
 * Shoot-on-the-move helper:
 * - Driver controls translation (field-relative).
 * - Command controls rotation to face the hub (no turret).
 * - Shooter RPM + Time-of-Flight (TOF) come from LUT (measured) :contentReference[oaicite:4]{index=4}
 * - Uses a latency constant you tune (camera+net+mechanism lag) :contentReference[oaicite:5]{index=5}
 */
public class AimHubShootOnMoveLUT extends Command {

  private final SwerveSubsystem drive;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Kicker kicker;

  // Driver translation inputs (typically joystick Y = +forward, X = +left; adapt as needed).
  private final DoubleSupplier xCmdSupplier; // field-relative m/s command (normalized -1..1)
  private final DoubleSupplier yCmdSupplier; // field-relative m/s command (normalized -1..1)

  // Optional: only run aiming when held (e.g., right trigger).
  private final BooleanSupplier enableSupplier;

  // Robot state
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldVelSupplier; // field-relative vx/vy (m/s)

  // Field target (hub center) in FIELD coordinates
  private final Translation2d hubFieldPos;

  // Shooter exit offset from robot center (meters). Measure this.
  private final Translation2d shooterExitOffsetRobot;

  // LUTs: distance (m) -> shooter rpm, and distance (m) -> time-of-flight (s)
  private final InterpolatingDoubleTreeMap rpmLut = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap tofLut = new InterpolatingDoubleTreeMap();

  // Heading control
  private final PIDController headingPid;

  // Translation limiting (keep your SOTM “inside the LUT envelope”)
  private final double maxShootSpeedMps;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;

  // Rotation limiting
  private final double maxOmegaRadPerSec;

  // Tuned constant: total latency between “now” and “ball actually leaves” :contentReference[oaicite:6]{index=6}
  private final double latencySec;

  // Aim tolerance / gating
  private final double aimToleranceRad;

  // Last computed setpoints (useful for logging or feeding decisions)
  private double lastRpm = 0.0;
  private double lastTof = 0.0;
  private Rotation2d lastDesiredYaw = new Rotation2d();

  public AimHubShootOnMoveLUT(
      SwerveSubsystem drive,
      Shooter shooter,
      Hopper hopper,
      Kicker kicker,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldVelSupplier,
      Translation2d hubFieldPos,
      Translation2d shooterExitOffsetRobot,
      DoubleSupplier xCmdSupplier,
      DoubleSupplier yCmdSupplier,
      BooleanSupplier enableSupplier,
      double maxShootSpeedMps,
      double maxShootAccelMps2,
      double maxOmegaRadPerSec,
      double headingKp,
      double headingKd,
      double latencySec,
      double aimToleranceDeg
  ) {
    this.drive = drive;
    this.shooter = shooter;
    this.hopper = hopper;
    this.kicker = kicker;
    this.poseSupplier = poseSupplier;
    this.fieldVelSupplier = fieldVelSupplier;

    this.hubFieldPos = hubFieldPos;
    this.shooterExitOffsetRobot = shooterExitOffsetRobot;

    this.xCmdSupplier = xCmdSupplier;
    this.yCmdSupplier = yCmdSupplier;
    this.enableSupplier = enableSupplier;

    this.maxShootSpeedMps = maxShootSpeedMps;
    this.xLimiter = new SlewRateLimiter(maxShootAccelMps2);
    this.yLimiter = new SlewRateLimiter(maxShootAccelMps2);

    this.maxOmegaRadPerSec = maxOmegaRadPerSec;

    this.headingPid = new PIDController(headingKp, 0.0, headingKd);
    this.headingPid.enableContinuousInput(-Math.PI, Math.PI);

    this.latencySec = latencySec;
    this.aimToleranceRad = Math.toRadians(aimToleranceDeg);

    addRequirements(drive, shooter);

    // ---- Populate LUTs here or pass them in (recommended) ----
    // Your existing RPM table idea is already in your ShootOnTheMoveCommand :contentReference[oaicite:7]{index=7}.
    // Add TOF points the same way (measure with phone video) :contentReference[oaicite:8]{index=8}.
    //
    // Example ONLY (replace with your measured data):
    rpmLut.put(2.0, 1620.0);
    rpmLut.put(3.0, 1770.0);
    rpmLut.put(4.0, 2020.0);

    tofLut.put(2.0, 0.55);
    tofLut.put(3.0, 0.70);
    tofLut.put(4.0, 0.85);
  }

  @Override
  public void initialize() {
    headingPid.reset();
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    if (!enableSupplier.getAsBoolean()) {
      // Don’t fight driver when not enabled.
      // Optionally: stop shooter setpoint here.
      return;
    }

    final Pose2d robotPose = poseSupplier.get();
    final Rotation2d robotYaw = robotPose.getRotation();

    // Shooter exit point in FIELD coordinates (robot center + rotated offset)
    final Translation2d shooterExitField =
        robotPose.getTranslation().plus(shooterExitOffsetRobot.rotateBy(robotYaw));

    // Field-relative robot velocity (vx, vy) for lead
    final ChassisSpeeds fieldVel = fieldVelSupplier.get();
    final Translation2d vField = new Translation2d(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond);

    // --- Iterative solve (2 passes) to reduce “toward/away” TOF mismatch ---
    // The writeup notes TOF/required velocity can skew when moving toward/away; iterating stabilizes it. :contentReference[oaicite:9]{index=9}
    Translation2d virtualHub = hubFieldPos;
    double tof = 0.0;

    for (int i = 0; i < 2; i++) {
      final double dist = shooterExitField.getDistance(virtualHub);
      tof = tofLut.get(dist);

      // Total lead time includes latency + flight time (latency is the tuning knob) :contentReference[oaicite:10]{index=10}
      final double leadTime = latencySec + tof;
      virtualHub = hubFieldPos.minus(vField.times(leadTime));
    }

    // Use virtual target distance for setpoint selection (usually small difference, but consistent)
    final double effectiveDist = shooterExitField.getDistance(virtualHub);
    final double rpm = rpmLut.get(effectiveDist);

    lastRpm = rpm;
    lastTof = tof;

    // Desired yaw: point shooter exit toward the virtual hub
    final Rotation2d desiredYaw = virtualHub.minus(shooterExitField).getAngle();
    lastDesiredYaw = desiredYaw;

    // Heading PID -> omega
    final double yawErrorRad = MathUtil.angleModulus(desiredYaw.getRadians() - robotYaw.getRadians());
    double omegaCmd = headingPid.calculate(robotYaw.getRadians(), desiredYaw.getRadians());
    omegaCmd = MathUtil.clamp(omegaCmd, -maxOmegaRadPerSec, maxOmegaRadPerSec);

    // Driver translation (normalized) -> m/s, then limit speed + accel
    double vxCmd = xCmdSupplier.getAsDouble() * maxShootSpeedMps;
    double vyCmd = yCmdSupplier.getAsDouble() * maxShootSpeedMps;

    // Optional: circular speed clamp
    double mag = Math.hypot(vxCmd, vyCmd);
    if (mag > maxShootSpeedMps) {
      vxCmd = vxCmd / mag * maxShootSpeedMps;
      vyCmd = vyCmd / mag * maxShootSpeedMps;
    }

    // Slew-rate limit (accel cap)
    vxCmd = xLimiter.calculate(vxCmd);
    vyCmd = yLimiter.calculate(vyCmd);

    // Apply chassis command (field-relative translation, controlled omega).
    // If your drive method expects robot-relative speeds, convert before sending.
    drive.driveFieldOriented(new ChassisSpeeds(vxCmd, vyCmd, omegaCmd));

    // Shooter setpoint:
    // Your current Shooter uses a fixed SHOOTER_SPEED constant :contentReference[oaicite:11]{index=11}.
    // You MUST add a "setTargetRPM(rpm)" style method (patch below) to actually use the LUT.
    shooter.setTargetRPM(rpm);

    // You can also gate feeding on yaw error + shooter speed:
    // boolean aimed = Math.abs(yawErrorRad) < aimToleranceRad;
     boolean upToSpeed = shooter.isAtTargetRPM();  // implement using targetRPM not constant :contentReference[oaicite:12]{index=12}
    // if (aimed && upToSpeed) feed...
    if (upToSpeed) {
      // feed balls into shooter
      hopper.HopperToShooter();
      kicker.Kick();
    } else {
      // optionally stop feeding when not aimed or not up to speed
      hopper.stopHopper();
      kicker.stopKicking();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Decide your behavior:
    // - usually keep drive under default command automatically
    // - stop shooter if you want
    shooter.stopShooting();
    hopper.stopHopper();
      kicker.stopKicking();
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }

  // Optional getters for logging/telemetry
  public double getLastRpm() { return lastRpm; }
  public double getLastTof() { return lastTof; }
  public Rotation2d getLastDesiredYaw() { return lastDesiredYaw; }
}
