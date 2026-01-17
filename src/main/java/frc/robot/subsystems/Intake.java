package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
public class Intake extends SubsystemBase {
   
    private SparkFlex intakeMotor = new SparkFlex(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    private SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

    public Intake(){
        intakeMotor.configure(Configs.IntakeSubsystem.IntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void runIntake(){
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }
    public void runOuttake(){
        intakeMotor.set(IntakeConstants.OUTTAKE_SPEED);
    }
    public void stopIntake(){
        intakeMotor.set(0);
    }
    
    public Command runIntakeCommand(){
        return new RunCommand(() -> runIntake(), this)
            .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand(){
        return new RunCommand(() -> runOuttake(), this)
            .finallyDo(interrupted -> stopIntake());
    }

    @Override
    public void periodic() {
    }
}
