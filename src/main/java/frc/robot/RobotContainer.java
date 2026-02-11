// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs.ShooterSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pushout;


/**
* This class is where the bulk of the robot should be declared. Since
* Command-based is a "declarative" paradigm, very
* little robot logic should actually be handled in the {@link Robot} periodic
* methods (other than the scheduler calls).
* Instead, the structure of the robot (including subsystems, commands, and
* trigger mappings) should be declared here.
*/
public class RobotContainer {

 // Replace with CommandPS4Controller or CommandJoystick if needed
 final CommandXboxController driverXbox = new CommandXboxController(0);
 final CommandXboxController operatorXbox = new CommandXboxController(1);
 // The robot's subsystems and commands are defined here...
 private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
     "swerve"));


 // Instantiate Subsystems
 private final Intake m_intake = new Intake();
 private final Hopper m_hopper = new Hopper();
 private final Shooter m_shooter = new Shooter();
 private final Climber m_climber = new Climber();
 private final Kicker m_kicker = new Kicker();
private final Pushout m_pushout = new Pushout();


 // Establish a Sendable Chooser that will be able to be sent to the
 // SmartDashboard, allowing selection of desired auto
 private final SendableChooser<Command> autoChooser;
 private LoggedDashboardChooser<Command> loggedAutoChooser;


 /**
  * Converts driver input into a field-relative ChassisSpeeds that is controlled
  * by angular velocity.
  */
 SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
     () -> driverXbox.getLeftY() * -1,
     () -> driverXbox.getLeftX() * -1)
     .withControllerRotationAxis(() -> driverXbox.getRightX()*-1)
     .deadband(OperatorConstants.DEADBAND)
     .scaleTranslation(0.8)
     .allianceRelativeControl(true);


 /**
  * Clone's the angular velocity input stream and converts it to a fieldRelative
  * input stream.
  */
 SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
     driverXbox::getRightY)
     .headingWhile(true);


 /**
  * Clone's the angular velocity input stream and converts it to a robotRelative
  * input stream.
  */
 SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
     .allianceRelativeControl(false);


 SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
     () -> -driverXbox.getLeftY(),
     () -> -driverXbox.getLeftX())
     .withControllerRotationAxis(() -> driverXbox.getRawAxis(
         2))
     .deadband(OperatorConstants.DEADBAND)
     .scaleTranslation(0.8)
     .allianceRelativeControl(true);
 // Derive the heading axis with math!
 SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
     .withControllerHeadingAxis(() -> Math.sin(
         driverXbox.getRawAxis(
             2) *
             Math.PI)
         *
         (Math.PI *
             2),
         () -> Math.cos(
             driverXbox.getRawAxis(
                 2) *
                 Math.PI)
             *
             (Math.PI *
                 2))
     .headingWhile(true)
     .translationHeadingOffset(true)
     .translationHeadingOffset(Rotation2d.fromDegrees(
         0));


 // Parallel Commands
 private final Trigger RTtransfer_kick_shoot = driverXbox.rightTrigger(); // transfer to kicker, kick, and shoot only when up to speed
 private final Trigger LTpushout_and_intake  = driverXbox.rightBumper(); // pushout the intake and intake fuel
 private final Trigger retract_and_stop = driverXbox.leftBumper(); // retract 4 bar and stop intake
 private final Trigger transfer = driverXbox.povRight(); // transfer to kicker and kicks
 private final Trigger unjam = driverXbox.povLeft(); // run hopper in reverse and kick backwards to unjam


 // Shooter
 private final Trigger shootFuel = driverXbox.y();
 private final Trigger speedUpShooter = driverXbox.leftTrigger();

 // Intake
 private final Trigger runIntake = driverXbox.x();
 private final Trigger runOuttake = driverXbox.a();

 // Pushout
 private final Trigger extendIntake = operatorXbox.y();
 private final Trigger agitate = operatorXbox.a();

 // Climber
 private final Trigger Climb = driverXbox.povUp();
 private final Trigger ClimbDown = driverXbox.povDown();


 /**
  * The container for the robot. Contains subsystems, OI devices, and commands.
  */
 public RobotContainer() {


   // Configure the trigger bindings
   configureBindings();
   DriverStation.silenceJoystickConnectionWarning(true);
//    SmartDashboard.putNumber("Heading Bias Deg", 0.0);
//    // Tunable gain: radians of bias -> radians/sec of angular velocity
//    SmartDashboard.putNumber("Heading Bias Gain", 0);


   // Create the NamedCommands that will be used in PathPlanner
   NamedCommands.registerCommand("test", Commands.print("I EXIST"));
   NamedCommands.registerCommand("extend intake", m_pushout.PushCommand().withTimeout(4));
   NamedCommands.registerCommand("retract intake", m_pushout.RetractCommand().withTimeout(4));
   NamedCommands.registerCommand("kick", m_kicker.kickCommand().withTimeout(8));
   NamedCommands.registerCommand("kick backwards", m_kicker.kickBackwardsCommand().withTimeout(8));
   NamedCommands.registerCommand("shoot", m_shooter.shootFuelCommand().withTimeout(8));
   NamedCommands.registerCommand("speed up shooter", m_shooter.SpeedUpShooterCommand().withTimeout(15));
   NamedCommands.registerCommand("transfer", m_hopper.runHopperToShooterCommand().withTimeout(6.7));
   NamedCommands.registerCommand("reverse hopper", m_hopper.runReverseHopperCommand().withTimeout(6.7));
   NamedCommands.registerCommand("intake", m_intake.runIntakeCommand().withTimeout(4));
   NamedCommands.registerCommand("outtake", m_intake.runOuttakeCommand().withTimeout(6.7));
  //  NamedCommands.registerCommand("climb up", m_climber.runClimbCommand().withTimeout(6.7));
  //  NamedCommands.registerCommand("climb down", m_climber.runClimberDownCommand().withTimeout(6.7));




   // Have the autoChooser pull in all PathPlanner autos as options
   autoChooser = AutoBuilder.buildAutoChooser();


   // Set the default auto (do nothing)
   autoChooser.setDefaultOption("Do Nothing", Commands.none());


   // // Add a simple auto option to have the robot drive forward for 1 second then
   // // stop
   // autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));


   // Put the autoChooser on the SmartDashboard
   SmartDashboard.putData("Auto Chooser", autoChooser);


   loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);


   // SmartDashboard.putData("SysId/Shooter Quasistatic Forward", m_shooter.sysIdQuasistaticForward());
   // SmartDashboard.putData("SysId/Shooter Quasistatic Reverse", m_shooter.sysIdQuasistaticReverse());
   // SmartDashboard.putData("SysId/Shooter Dynamic Forward", m_shooter.sysIdDynamicForward());
   // SmartDashboard.putData("SysId/Shooter Dynamic Reverse", m_shooter.sysIdDynamicReverse());
 }


 /**
  * Use this method to define your trigger->command mappings. Triggers can be
  * created via the
  * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
  * an arbitrary predicate, or via the
  * named factories in
  * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
  * for
  * {@link CommandXboxController
  * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
  * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
  * Flight joysticks}.
  */
 private void configureBindings() {

   // transfer + kick + shoot command, only runs if the shooter is up to speed
   RTtransfer_kick_shoot.whileTrue(Commands.parallel(m_hopper.runHopperToShooterCommand(), m_kicker.kickCommand(), m_shooter.shootFuelCommand()).onlyIf(m_shooter::isShooterFast));
   LTpushout_and_intake.whileTrue(Commands.parallel(m_pushout.PushCommand(), m_intake.runIntakeCommand()));
   retract_and_stop.whileTrue(Commands.parallel(m_pushout.RetractCommand()));

   // Pushout Commands
   extendIntake.whileTrue(m_pushout.PushCommand());
   agitate.whileTrue(m_pushout.AgitateCommand().repeatedly());

   // Intake Commands
   runIntake.whileTrue(m_intake.runIntakeCommand());
   runOuttake.whileTrue(m_intake.runOuttakeCommand());
   

   // Hopper Commands
   transfer.whileTrue(Commands.parallel(m_hopper.runHopperToShooterCommand(), m_kicker.kickCommand()));
   unjam.whileTrue(Commands.parallel(m_hopper.runReverseHopperCommand(), m_kicker.kickBackwardsCommand()));


   // Shooter Commands
   shootFuel.whileTrue(m_shooter.shootFuelCommand());
   speedUpShooter.whileTrue(m_shooter.SpeedUpShooterCommand());

   // Climber Commands
  //  Climb.whileTrue(m_climber.runClimbCommand());
  //  ClimbDown.whileTrue(m_climber.runClimberDownCommand());

    // SysId: run shooter quasistatic forward.
    operatorXbox.a().whileTrue(m_shooter.sysIdQuasistaticForward());
    // SysId: run shooter quasistatic reverse.
    operatorXbox.b().whileTrue(m_shooter.sysIdQuasistaticReverse());
    // SysId: run shooter dynamic forward.
    operatorXbox.x().whileTrue(m_shooter.sysIdDynamicForward());
    // SysId: run shooter dynamic reverse.
    operatorXbox.y().whileTrue(m_shooter.sysIdDynamicReverse());




   // Swerve Drive Commands
   driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));


   Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(() -> applyHeadingBias(driveDirectAngle.get()));
   Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
       () -> applyHeadingBias(driveAngularVelocity.get()));
   Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
   Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
       driveDirectAngle);
   Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(
       () -> applyHeadingBias(driveDirectAngleKeyboard.get()));
   Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(
       () -> applyHeadingBias(driveAngularVelocityKeyboard.get()));
   Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
       driveDirectAngleKeyboard);


   if (RobotBase.isSimulation()) {
     drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
   } else {
     if (Constants.USE_ROBOT_RELATIVE) {
       drivebase.setDefaultCommand(
           drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
     } else {
       drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
     }
   }


   if (Robot.isSimulation()) {
     Pose2d target = new Pose2d(new Translation2d(1, 4),
         Rotation2d.fromDegrees(90));
     // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
     driveDirectAngleKeyboard.driveToPose(() -> target,
         new ProfiledPIDController(5,
             0,
             0,
             new Constraints(5, 2)),
         new ProfiledPIDController(5,
             0,
             0,
             new Constraints(Units.degreesToRadians(360),
                 Units.degreesToRadians(180))));
     driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
     driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
     driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
         () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));


     // driverXbox.b().whileTrue(
     // drivebase.driveToPose(
     // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
     // );
   
   }
   if (DriverStation.isTest())
   {
   if (Constants.USE_ROBOT_RELATIVE) {
     drivebase.setDefaultCommand(
         drivebase.run(() -> drivebase.drive(driveRobotOriented.get())));
   } else {
     drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
   }


   driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
   driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
   driverXbox.back().whileTrue(drivebase.centerModulesCommand());
   driverXbox.leftBumper().onTrue(Commands.none());
   driverXbox.rightBumper().onTrue(Commands.none());
   } else
   {
   driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
   driverXbox.start().whileTrue(Commands.none());
   driverXbox.back().whileTrue(Commands.none());
   driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
   driverXbox.rightBumper().onTrue(Commands.none());
   }


 }


 /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
 public Command getAutonomousCommand() {
   // Pass in the selected auto from the SmartDashboard as our desired autnomous
   // commmand
   return loggedAutoChooser.get();
 }


 public void setMotorBrake(boolean brake) {
   drivebase.setMotorBrake(brake);
 }


 public void logControllerInputs()
 {
   // Driver left stick X (-1..1).
   Logger.recordOutput("Input/Driver/LeftX", driverXbox.getLeftX());
   // Driver left stick Y (-1..1).
   Logger.recordOutput("Input/Driver/LeftY", driverXbox.getLeftY());
   // Driver right stick X (-1..1).
   Logger.recordOutput("Input/Driver/RightX", driverXbox.getRightX());
   // Driver right stick Y (-1..1).
   Logger.recordOutput("Input/Driver/RightY", driverXbox.getRightY());
   // Driver left trigger (0..1).
   Logger.recordOutput("Input/Driver/LeftTrigger", driverXbox.getLeftTriggerAxis());
   // Driver right trigger (0..1).
   Logger.recordOutput("Input/Driver/RightTrigger", driverXbox.getRightTriggerAxis());


   // Operator left stick X (-1..1).
   Logger.recordOutput("Input/Operator/LeftX", operatorXbox.getLeftX());
   // Operator left stick Y (-1..1).
   Logger.recordOutput("Input/Operator/LeftY", operatorXbox.getLeftY());
   // Operator right stick X (-1..1).
   Logger.recordOutput("Input/Operator/RightX", operatorXbox.getRightX());
   // Operator right stick Y (-1..1).
   Logger.recordOutput("Input/Operator/RightY", operatorXbox.getRightY());
   // Operator left trigger (0..1).
   Logger.recordOutput("Input/Operator/LeftTrigger", operatorXbox.getLeftTriggerAxis());
   // Operator right trigger (0..1).
   Logger.recordOutput("Input/Operator/RightTrigger", operatorXbox.getRightTriggerAxis());
 }


 private ChassisSpeeds applyHeadingBias(ChassisSpeeds speeds) {
       // Toggle to enable heading bias; false means pass-through.
       boolean headingBiasEnabled = SmartDashboard.getBoolean("headingBiasEnabled", false);
       if (!headingBiasEnabled) {
           return speeds;
       }
       // Requested heading bias in degrees; 0 means disabled.
       double biasDeg = SmartDashboard.getNumber("Heading Bias Deg", 0.0);
       // Gain mapping bias radians -> added omega (rad/sec).
       double gain = SmartDashboard.getNumber("Heading Bias Gain", 0.0);


       // Default to normal driving (no bias).
       double omega = speeds.omegaRadiansPerSecond;
       if (biasDeg != 0.0 && gain != 0.0) {
           // Convert degrees to radians, then scale into an omega offset.
           double biasRad = Units.degreesToRadians(biasDeg);
           double additionalOmega = gain * biasRad;
           // Leave vx/vy alone; only add a small angular velocity component.
           omega += additionalOmega;
       }


       return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
 }
 }
