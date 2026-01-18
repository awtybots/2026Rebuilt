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
public class Hopper extends SubsystemBase {

    // Instantiates push down and transfer motors
    private SparkFlex HopperPushDownMotor = new SparkFlex(HopperConstants.HOPPER_PUSHDOWN_ID, MotorType.kBrushless);
    private SparkClosedLoopController HopperPushDownController = HopperPushDownMotor.getClosedLoopController(); // idk what this is

    private SparkFlex HopperTransferMotor = new SparkFlex(HopperConstants.HOPPER_TRANSFER_ID, MotorType.kBrushless);
    private SparkClosedLoopController HopperTransferController = HopperTransferMotor.getClosedLoopController(); // idk what this is

    public Hopper(){
        HopperPushDownMotor.configure(Configs.HopperSubsystem.HopperPushDownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        HopperTransferMotor.configure(Configs.HopperSubsystem.HopperTransferMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void HopperToShooter(){
        HopperPushDownMotor.set(HopperConstants.HOPPER_PUSHDOWN_SPEED);
        HopperTransferMotor.set(HopperConstants.HOPPER_TRANSFER_SPEED);
    }
    public void ReverseHopper(){ 
        HopperPushDownMotor.set(HopperConstants.REVERSE_HOPPER_PUSHDOWN_SPEED);
        HopperTransferMotor.set(HopperConstants.REVERSE_HOPPER_TRANSFER_SPEED);
    }
    public void stopHopper(){
        HopperPushDownMotor.set(0);
        HopperTransferMotor.set(0);
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
    }
}