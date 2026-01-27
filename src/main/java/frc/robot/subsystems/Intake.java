package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex intakeMotor = new SparkFlex(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    private SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

    public Intake() {
        intakeMotor.configure(Configs.IntakeSubsystem.IntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void runIntake() {
        desiredPercent = IntakeConstants.INTAKE_SPEED;
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void runOuttake() {
        desiredPercent = IntakeConstants.OUTTAKE_SPEED;
        intakeMotor.set(IntakeConstants.OUTTAKE_SPEED);
    }

    public void stopIntake() {
        desiredPercent = 0.0;
        intakeMotor.set(0);
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
        Logger.recordOutput("Intake/DesiredPercent", desiredPercent);
        Logger.recordOutput("Intake/AppliedVolts", intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    }
}