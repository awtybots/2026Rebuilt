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
import frc.robot.Configs;
public class Shooter extends SubsystemBase {
   
    // Instantiating the hopper to shooter motor
    private SparkFlex ShooterKickerMotor = new SparkFlex(ShooterConstants.SHOOTER_KICKER_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterkickerController = ShooterKickerMotor.getClosedLoopController(); // idk what this is

    private SparkFlex ShooterRightMotor = new SparkFlex(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterrightController = ShooterRightMotor.getClosedLoopController(); // idk what this is
    
    private SparkFlex ShooterLeftMotor = new SparkFlex(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterleftController = ShooterLeftMotor.getClosedLoopController(); // idk what this is



    public Shooter(){
        ShooterKickerMotor.configure(Configs.ShooterSubsystem.ShooterKickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        ShooterRightMotor.configure(Configs.ShooterSubsystem.ShooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        ShooterLeftMotor.configure(Configs.ShooterSubsystem.ShooterLeftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      
    }

    public void shootFuel(){
    ShooterKickerMotor.set(ShooterConstants.KICKER_SPEED);
    // Use the closed-loop controller's setSetpoint so the configured PID slot is used
    shooterrightController.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kVelocity);
    shooterleftController.setSetpoint(ShooterConstants.SHOOTER_SPEED, ControlType.kVelocity);
    }
    public void stopShooting(){
        ShooterKickerMotor.set(0);
        ShooterRightMotor.set(0);
        ShooterLeftMotor.set(0);
    }
    
    public Command shootFuelCommand(){
        return new RunCommand(() -> shootFuel(), this)
            .finallyDo(interrupted -> stopShooting());
    }

    @Override
    public void periodic() {
    }
}
