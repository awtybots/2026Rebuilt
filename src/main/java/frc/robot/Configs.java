package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public final class Configs 
{

        public static final class IntakeSubsystem {
                
            public static final SparkFlexConfig IntakeMotorConfig = new SparkFlexConfig();

                static {

                        IntakeMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        IntakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.4)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1);
                        //     // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

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

                }

        };

        public static final class ShooterSubsystem {

                public static final SparkFlexConfig kickerLeftMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig kickerRightMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig kickerTransferMotorConfig = new SparkFlexConfig();

                public static final SparkFlexConfig ShooterRightMotor1Config = new SparkFlexConfig(); 
                public static final SparkFlexConfig ShooterRightMotor2Config = new SparkFlexConfig(); 

                public static final SparkFlexConfig ShooterLeftMotor1Config = new SparkFlexConfig();
                public static final SparkFlexConfig ShooterLeftMotor2Config = new SparkFlexConfig();
                
                        static {

                                kickerLeftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                kickerRightMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                kickerTransferMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

                                ShooterRightMotor1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                ShooterRightMotor2Config.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                
                                ShooterLeftMotor1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                ShooterLeftMotor2Config.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                

                                ShooterRightMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;              

                                ShooterRightMotor1Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                                ShooterRightMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;

                                ShooterRightMotor2Config.closedLoop
                                .maxMotion.maxAcceleration(10000);
                                
                                ShooterLeftMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;              

                                ShooterLeftMotor1Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                                ShooterLeftMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;

                                ShooterLeftMotor2Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                }

 
        }

        public static final class HopperSubsystem {
                
            public static final SparkFlexConfig HopperPushDownMotorConfig = new SparkFlexConfig();
            public static final SparkFlexConfig HopperTransferMotorConfig = new SparkFlexConfig();

                static {

                        HopperPushDownMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        HopperTransferMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        HopperPushDownMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.4)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1);
                            // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
                        

                            HopperTransferMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.4)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1);
                            // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

                     

                }
        }

        public static final class ClimberSubsystem {
                
            public static final SparkFlexConfig ClimbMotorLeftConfig = new SparkFlexConfig();
            public static final SparkFlexConfig ClimbMotorRightConfig = new SparkFlexConfig();

                static {

                        ClimbMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        ClimbMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        ClimbMotorLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(0.4)
                                .i(0)
                                .d(0)
                                .outputRange(-1, 1);
                        ClimbMotorRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)

                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.4)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1);
                        //     // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

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

                }

        };
}

