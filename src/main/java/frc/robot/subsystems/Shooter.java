package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import frc.robot.Configs;

public class Shooter extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    private SparkFlex ShooterKickerMotor = new SparkFlex(ShooterConstants.SHOOTER_KICKER_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterkickerController = ShooterKickerMotor.getClosedLoopController(); 

    private SparkFlex ShooterRightMotor = new SparkFlex(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterrightController = ShooterRightMotor.getClosedLoopController(); 

    private SparkFlex ShooterLeftMotor = new SparkFlex(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterleftController = ShooterLeftMotor.getClosedLoopController(); 

    private SparkFlex HoodMotor = new SparkFlex(ShooterConstants.HOOD_ID, MotorType.kBrushless);
    private SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();

    private final RelativeEncoder shooterRightEncoder = ShooterRightMotor.getEncoder();
    private final RelativeEncoder shooterLeftEncoder = ShooterLeftMotor.getEncoder();
    private final RelativeEncoder kickerEncoder = ShooterKickerMotor.getEncoder();

        private double targetRPM = 0.0;
    private double targetKickerRPM = 0.0;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        ShooterRightMotor.setVoltage(voltage);
                        ShooterLeftMotor.setVoltage(voltage);
                    },
                    log -> {
                        double rightRps = shooterRightEncoder.getVelocity() / 60.0;
                        double leftRps = shooterLeftEncoder.getVelocity() / 60.0;
                        double rightVolts = ShooterRightMotor.getAppliedOutput() * ShooterRightMotor.getBusVoltage();
                        double leftVolts = ShooterLeftMotor.getAppliedOutput() * ShooterLeftMotor.getBusVoltage();

                        log.motor("shooter-right")
                                .voltage(Units.Volts.of(rightVolts))
                                .angularPosition(Units.Rotations.of(shooterRightEncoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(rightRps));
                        log.motor("shooter-left")
                                .voltage(Units.Volts.of(leftVolts))
                                .angularPosition(Units.Rotations.of(shooterLeftEncoder.getPosition()))
                                .angularVelocity(Units.RotationsPerSecond.of(leftRps));
                    },
                    this));

    public Shooter() {

        ShooterKickerMotor.configure(Configs.ShooterSubsystem.ShooterKickerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        ShooterRightMotor.configure(Configs.ShooterSubsystem.ShooterRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        ShooterLeftMotor.configure(Configs.ShooterSubsystem.ShooterLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        HoodMotor.configure(Configs.ShooterSubsystem.HoodMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void shootFuel() {
        targetRPM = ShooterConstants.SHOOTER_SPEED;
        targetKickerRPM = ShooterConstants.KICKER_SPEED;
        shooterkickerController.setSetpoint(ShooterConstants.KICKER_SPEED, ControlType.kMAXMotionVelocityControl);
        shooterrightController.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kMAXMotionVelocityControl);
        shooterleftController.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kMAXMotionVelocityControl);
    }

    public void stopShooting() {
         targetRPM = 0.0;
        targetKickerRPM = 0.0;
        ShooterLeftMotor.set(ShooterConstants.IDLE);
        ShooterRightMotor.set(ShooterConstants.IDLE);
        ShooterKickerMotor.set(ShooterConstants.STOP);
        // shooterkickerController.setSetpoint(ShooterConstants.STOP, ControlType.kMAXMotionVelocityControl);
        // shooterleftController.setSetpoint(ShooterConstants.STOP, ControlType.kMAXMotionVelocityControl);
        // shooterrightController.setSetpoint(ShooterConstants.STOP, ControlType.kMAXMotionVelocityControl);
    }

    public void RotateHoodUp()
    {
        HoodMotor.set(ShooterConstants.HOOD_UP_SPEED);
    }

    public void RotateHoodDown()
    {
        HoodMotor.set(ShooterConstants.HOOD_DOWN_SPEED);
    }

    public void StopHood()
    {
        HoodMotor.set(0);
    }

    public Command shootFuelCommand() {
         
        return new RunCommand(() -> shootFuel(), this)
                .finallyDo(interrupted -> stopShooting());
    }

    public Command stopShootingCommand() {
        return new RunCommand(() -> stopShooting(), this);
    }

    public Command RotateHoodUpCommand()
    {
        return new RunCommand(() -> RotateHoodUp(), this)
                .finallyDo(interrupted -> StopHood());
    }

    public Command RotateHoodDownCommand()
    {
        return new RunCommand(() -> RotateHoodDown(), this)
                .finallyDo(interrupted -> StopHood());
    }

    public Command sysIdQuasistaticForward()
    {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse()
    {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward()
    {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse()
    {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

   
    @Override
    public void periodic() {
        // AdvantageKit Logging
        double rightRPM = ShooterRightMotor.getEncoder().getVelocity();
        double leftRPM = ShooterLeftMotor.getEncoder().getVelocity();
        double kickerRPM = ShooterKickerMotor.getEncoder().getVelocity();

        Logger.recordOutput("Shooter/RightRPM", rightRPM);
        Logger.recordOutput("Shooter/LeftRPM", leftRPM);
        Logger.recordOutput("Shooter/KickerRPM", kickerRPM);

        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/TargetKickerRPM", targetKickerRPM);

        Logger.recordOutput("Shooter/RightAppliedVolts", ShooterRightMotor.getAppliedOutput() * ShooterRightMotor.getBusVoltage());
        Logger.recordOutput("Shooter/LeftAppliedVolts", ShooterLeftMotor.getAppliedOutput() * ShooterLeftMotor.getBusVoltage());
        Logger.recordOutput("Shooter/KickerAppliedVolts", ShooterKickerMotor.getAppliedOutput() * ShooterKickerMotor.getBusVoltage());
    }
}
