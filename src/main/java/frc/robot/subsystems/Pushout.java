package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PushoutConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;



public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex PushoutLeftMotor = new SparkFlex(PushoutConstants.PUSHOUT_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutLeftController = PushoutLeftMotor.getClosedLoopController();
    private SparkFlex PushoutRightMotor = new SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutRightController = PushoutRightMotor.getClosedLoopController();

    private final RelativeEncoder pushoutLeftEncoder = PushoutLeftMotor.getEncoder();
    private final RelativeEncoder pushoutRightEncoder = PushoutRightMotor.getEncoder();

    private double PushoutRightExtended = PushoutConstants.PUSHOUT_EXTENDED_POS;
    private double PushoutLeftExtended = PushoutConstants.PUSHOUT_EXTENDED_POS;
    private double PushoutRightRetracted = PushoutConstants.PUSHOUT_RETRACTED_POS;
    private double PushoutLeftRetracted = PushoutConstants.PUSHOUT_RETRACTED_POS;



    public Pushout() {
        PushoutLeftMotor.configure(Configs.PushoutSubsystem.PushoutLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        PushoutRightMotor.configure(Configs.PushoutSubsystem.PushoutRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pushoutLeftEncoder.setPosition(0.0);
        pushoutRightEncoder.setPosition(0.0);
    }

    public void PushIntake() {
        PushoutLeftController.setSetpoint(PushoutLeftExtended, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutRightExtended, ControlType.kMAXMotionPositionControl);

    }
    public void RetractIntake() {
        PushoutLeftController.setSetpoint(PushoutLeftRetracted, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutRightRetracted, ControlType.kMAXMotionPositionControl);
    }

    public void StopPushing() {
        PushoutLeftController.setSetpoint(0, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(0, ControlType.kMAXMotionPositionControl);
    }



    public Command Push() {
        return new RunCommand(() -> PushIntake(), this)
                .finallyDo(interrupted -> StopPushing());
    }

    public Command Retract() {
        return new RunCommand(() -> RetractIntake(), this)
                .finallyDo(interrupted -> StopPushing());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Pushout/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        Logger.recordOutput("Pushout/AppliedVolts", PushoutLeftMotor.getAppliedOutput() * PushoutLeftMotor.getBusVoltage());
    }
}
