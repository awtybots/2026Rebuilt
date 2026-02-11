package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HopperConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;
public class Hopper extends SubsystemBase {

    // AdvantageKit logging
    private double TwindexerRightDesiredPercent = 0.0;
    private double TwindexerLeftDesiredPercent = 0.0;

    private int six_seven = HopperConstants.six_seven; // <---------- HISTORICAL MONUMENT

    // Instantiates push down and transfer motors
    private SparkFlex TwindexerRightMotor = new SparkFlex(HopperConstants.TWINDEXER_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController TwindexerRightController = TwindexerRightMotor.getClosedLoopController(); // idk what this is

    private SparkFlex TwindexerLeftMotor = new SparkFlex(HopperConstants.TWINDEXER_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController TwindexerLeftController = TwindexerLeftMotor.getClosedLoopController(); // idk what this is

    public Hopper(){
        TwindexerRightMotor.configure(Configs.HopperSubsystem.TwindexerRightControllerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        TwindexerLeftMotor.configure(Configs.HopperSubsystem.TwindexerLeftControllerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void HopperToShooter(){
        TwindexerRightDesiredPercent = HopperConstants.TWINDEXER_RIGHT_SPEED;
        TwindexerLeftDesiredPercent = HopperConstants.REVERSE_TWINDEXER_LEFT_SPEED;
        // TwindexerRightMotor.set(HopperConstants.TWINDEXER_RIGHT_SPEED);
        TwindexerLeftMotor.set(HopperConstants.TWINDEXER_LEFT_SPEED);
    }
    public void ReverseHopper(){ 
        TwindexerRightDesiredPercent = HopperConstants.TWINDEXER_RIGHT_SPEED;
        TwindexerLeftDesiredPercent = HopperConstants.REVERSE_TWINDEXER_LEFT_SPEED;
        // TwindexerRightMotor.set(HopperConstants.REVERSE_TWINDEXER_RIGHT_SPEED);
        TwindexerLeftMotor.set(HopperConstants.REVERSE_TWINDEXER_LEFT_SPEED);
    }
    public void stopHopper(){
        TwindexerRightDesiredPercent = 0.0;
        TwindexerLeftDesiredPercent = 0.0;
        TwindexerRightMotor.set(0);
        TwindexerLeftMotor.set(0);
    }
    
    public Command runHopperToShooterCommand(){
        return new RunCommand(() -> HopperToShooter(), this)
            .finallyDo(interrupted -> stopHopper());
    }

    public Command runReverseHopperCommand(){
        return new RunCommand(() -> ReverseHopper(), this)
            .finallyDo(interrupted -> stopHopper());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded pushdown motor percent output.
        Logger.recordOutput("Hopper/PushdownDesiredPercent", TwindexerRightDesiredPercent);
        // Commanded transfer motor percent output.
        Logger.recordOutput("Hopper/TransferDesiredPercent", TwindexerLeftDesiredPercent);
        // Applied voltage to pushdown motor.
        Logger.recordOutput("Hopper/PushdownAppliedVolts", TwindexerRightMotor.getAppliedOutput() * TwindexerRightMotor.getBusVoltage());
        // Applied voltage to transfer motor.
        Logger.recordOutput("Hopper/TransferAppliedVolts", TwindexerLeftMotor.getAppliedOutput() * TwindexerLeftMotor.getBusVoltage());
    }
}
