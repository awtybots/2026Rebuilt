package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
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

    private SparkFlex HoodMotor = new SparkFlex(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();


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
        shooterkickerController.setSetpoint(ShooterConstants.KICKER_SPEED, ControlType.kMAXMotionVelocityControl);
        shooterrightController.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kMAXMotionVelocityControl);
        shooterleftController.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kMAXMotionVelocityControl);
    }

    public void stopShooting() {
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

    @Override
    public void periodic() {
    }
}
