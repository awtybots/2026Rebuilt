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
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;



public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    // private SparkFlex PushoutLeftMotor = new SparkFlex(PushoutConstants.PUSHOUT_LEFT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController PushoutLeftController = PushoutLeftMotor.getClosedLoopController();
    private SparkFlex PushoutRightMotor = new SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutRightController = PushoutRightMotor.getClosedLoopController();

    // private final RelativeEncoder pushoutLeftEncoder = PushoutLeftMotor.getEncoder();
    private final RelativeEncoder pushoutRightEncoder = PushoutRightMotor.getEncoder();

    private double PushoutRightExtended = PushoutConstants.PUSHOUT_EXTENDED_POS;
    // private double PushoutLeftExtended = PushoutConstants.PUSHOUT_EXTENDED_POS;
    private double PushoutRightRetracted = PushoutConstants.PUSHOUT_RETRACTED_POS;
    // private double PushoutLeftRetracted = PushoutConstants.PUSHOUT_RETRACTED_POS;
    private double PushoutRightExtendedAgitate = PushoutConstants.PUSHOUT_EXTENDED_AGITATE_POS;
    // private double PushoutLeftExtendedAgitate = PushoutConstants.PUSHOUT_EXTENDED_AGITATE_POS;
    private double PushoutRightRetractedAgitate = PushoutConstants.PUSHOUT_RETRACTED_AGITATE_POS;
    // private double PushoutLeftRetractedAgitate = PushoutConstants.PUSHOUT_RETRACTED_AGITATE_POS;

    public Pushout() {
        // PushoutLeftMotor.configure(Configs.PushoutSubsystem.PushoutLeftMotorConfig, ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
        PushoutRightMotor.configure(Configs.PushoutSubsystem.PushoutRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void PushIntake() {
        // finds the setpoint compared to encoder positions so the pushout moves by the offsets from where it is
        // double leftNow = pushoutLeftEncoder.getPosition();
        double rightNow = pushoutRightEncoder.getPosition();
        // PushoutLeftController.setSetpoint(leftNow + PushoutLeftExtended, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(rightNow + PushoutRightExtended, ControlType.kMAXMotionPositionControl);

    }
    public void RetractIntake() {
        // double leftNow = pushoutLeftEncoder.getPosition();
        double rightNow = pushoutRightEncoder.getPosition();
        // PushoutLeftController.setSetpoint(leftNow + PushoutLeftRetracted, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(rightNow + PushoutRightRetracted, ControlType.kMAXMotionPositionControl);
    }

    public void SmallPush() 
    {
        // double leftNow = pushoutLeftEncoder.getPosition();
        double rightNow = pushoutRightEncoder.getPosition();
        // PushoutLeftController.setSetpoint(leftNow + PushoutLeftExtendedAgitate, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(rightNow + PushoutRightExtendedAgitate, ControlType.kMAXMotionPositionControl);
    }

    public void SmallRetract() 
    {
        // double leftNow = pushoutLeftEncoder.getPosition();
        double rightNow = pushoutRightEncoder.getPosition();
        // PushoutLeftController.setSetpoint(leftNow + PushoutLeftRetractedAgitate, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(rightNow + PushoutRightRetractedAgitate, ControlType.kMAXMotionPositionControl);
    }

    public void StopPushing() {
        // PushoutLeftController.setSetpoint(0, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(0, ControlType.kMAXMotionPositionControl);
    }

    public void Agitate()
    {
        while (true) {
            SmallPush();
            Timer.delay(0.5);
            SmallRetract();
        }
    }


    public Command PushCommand() {
        return new RunCommand(() -> PushIntake(), this)
                .finallyDo(interrupted -> PushIntake());
    }

    public Command RetractCommand() {
        return new RunCommand(() -> RetractIntake(), this)
                .finallyDo(interrupted -> RetractIntake());
    }

    public Command AgitateCommand() {
        return new RunCommand(() -> Agitate(), this)
                .finallyDo(interrupted -> RetractIntake());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Pushout/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        // Logger.recordOutput("Pushout/AppliedVolts", PushoutLeftMotor.getAppliedOutput() * PushoutLeftMotor.getBusVoltage());
    }
}
