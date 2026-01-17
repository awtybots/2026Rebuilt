package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.spark.FeedbackSensor;

public final class Configs {

        public static final class IntakeSubsystem {
                public static final SparkFlexConfig l_elevatorMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig r_elevatorMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig r_armMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig l_armMotorConfig = new SparkFlexConfig();
                // public static final SparkFlexConfig r_armMotorSlowConfig = new SparkFlexConfig();
                // public static final SparkFlexConfig l_armMotorSlowConfig = new SparkFlexConfig();
                public static final SparkFlexConfig wristMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

                static {

                        r_armMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        l_armMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        //r_armMotorConfig.inverted(true);
                        r_armMotorConfig.follow(12, true);

                        r_armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.6)
                                        .d(0.5)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(2200)
                                        .maxAcceleration(2000)
                                        .allowedClosedLoopError(.1);

                        l_armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.6)
                                        .d(0.5)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(2200)
                                        .maxAcceleration(2000)
                                        .allowedClosedLoopError(.1);

                        // r_armMotorSlowConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        // l_armMotorSlowConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        // //r_armMotorSlowConfig.inverted(true);
                        // r_armMotorSlowConfig.follow(12, true);
                        
                        // r_armMotorSlowConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //                 .p(0.2)
                        //                 .d(.4)
                        //                 .outputRange(-0.5, 0.5).maxMotion
                        //                 .maxVelocity(1400)
                        //                 .maxAcceleration(1300)
                        //                 .allowedClosedLoopError(.25);

                        // l_armMotorSlowConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //                 .p(0.2)
                        //                 .d(.4)
                        //                 .outputRange(-0.5, 0.5).maxMotion
                        //                 .maxVelocity(1400)
                        //                 .maxAcceleration(1300)
                        //                 .allowedClosedLoopError(.25);

                        l_elevatorMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        r_elevatorMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                        r_elevatorMotorConfig.follow(10, true);

                        l_elevatorMotorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.3)
                                        .d(.6)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(8000)
                                        .maxAcceleration(8000)
                                        .allowedClosedLoopError(.5);

                        r_elevatorMotorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.3)
                                        .d(.6)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(8000)
                                        .maxAcceleration(8000)
                                        .allowedClosedLoopError(.5);

                        wristMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

                        wristMotorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .p(0.01)
                                        .d(0.01)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(6000)
                                        .maxAcceleration(8000)
                                        .allowedClosedLoopError(.009);

                        intakeMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(50);

                }

        };

        public static final class FunnelIntakeSubsystem {
                public static final SparkFlexConfig l_funnelMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig r_funnelMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig funnelWristMotorConfig = new SparkFlexConfig();

                static {

                        funnelWristMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        funnelWristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .p(0.1)
                                        .outputRange(-1, 1).maxMotion
                                        .maxVelocity(2000)
                                        .maxAcceleration(10000)
                                        .allowedClosedLoopError(.25);

                        l_funnelMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
                        r_funnelMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(50);

                }
        };

        public static final class ClimberSubsystem {
                public static final SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

                static {
                        climberMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);

                }
        };
}
