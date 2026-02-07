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

import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;



public class Intake extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex IntakeLeftMotor = new SparkFlex(IntakeConstants.INTAKE_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController intakeLeftController = IntakeLeftMotor.getClosedLoopController();
    private SparkFlex IntakeRightMotor = new SparkFlex(IntakeConstants.INTAKE_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController intakeRightController = IntakeRightMotor.getClosedLoopController();

    private SparkFlex PushoutLeftMotor = new SparkFlex(IntakeConstants.PUSHOUT_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutLeftController = PushoutLeftMotor.getClosedLoopController();
    private SparkFlex PushoutRightMotor = new SparkFlex(IntakeConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutRightController = PushoutRightMotor.getClosedLoopController();

    private final RelativeEncoder pushoutLeftEncoder = PushoutLeftMotor.getEncoder();
    private final RelativeEncoder pushoutRightEncoder = PushoutRightMotor.getEncoder();

    private double PushoutRightExtended = IntakeConstants.PUSHOUT_EXTENDED_POS;
    private double PushoutLeftExtended = IntakeConstants.PUSHOUT_EXTENDED_POS;
    private double PushoutRightRetracted = IntakeConstants.PUSHOUT_RETRACTED_POS;
    private double PushoutLeftRetracted = IntakeConstants.PUSHOUT_RETRACTED_POS;



    public Intake() {
        IntakeLeftMotor.configure(Configs.IntakeSubsystem.IntakeLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        IntakeRightMotor.configure(Configs.IntakeSubsystem.IntakeRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        PushoutLeftMotor.configure(Configs.IntakeSubsystem.PushoutLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        PushoutRightMotor.configure(Configs.IntakeSubsystem.PushoutRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pushoutLeftEncoder.setPosition(0.0);
        pushoutRightEncoder.setPosition(0.0);
    }

    public void extendIntake() {
        PushoutLeftController.setSetpoint(PushoutLeftExtended, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutRightExtended, ControlType.kMAXMotionPositionControl);

    }
    public void retractIntake() {
        PushoutLeftController.setSetpoint(PushoutLeftRetracted, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutRightRetracted, ControlType.kMAXMotionPositionControl);
    }


    public void runIntake() {
        extendIntake();
        desiredPercent = IntakeConstants.INTAKE_SPEED;
        IntakeLeftMotor.set(IntakeConstants.INTAKE_SPEED);
        IntakeRightMotor.set(IntakeConstants.INTAKE_SPEED);
        
    }

    public void runOuttake() {
        retractIntake();
        desiredPercent = IntakeConstants.OUTTAKE_SPEED;
        IntakeLeftMotor.set(IntakeConstants.OUTTAKE_SPEED);
        IntakeRightMotor.set(IntakeConstants.OUTTAKE_SPEED);
    }

    public void stopIntake() {
        desiredPercent = 0.0;
        IntakeLeftMotor.set(0);
        IntakeRightMotor.set(0);        
    }

    public Command runIntakeCommand() {
        return new RunCommand(() -> runIntake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        return new RunCommand(() -> runOuttake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Intake/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        Logger.recordOutput("Intake/AppliedVolts", IntakeLeftMotor.getAppliedOutput() * IntakeLeftMotor.getBusVoltage());
    }
}
