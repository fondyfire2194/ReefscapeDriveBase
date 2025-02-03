// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class CommandFactory {

        SwerveSubsystem m_swerve;

        Pose2d finalReefPose = new Pose2d();

        public CommandFactory(SwerveSubsystem swerve) {
                m_swerve = swerve;

        }

        public Command driveToPose() {
                return m_swerve.driveToPose(finalReefPose);
        }

        public Command getReefPose() {
                return Commands.runOnce(() -> finalReefPose = m_swerve.reefFinalTargetPose);
        }

        public Command rumble(CommandXboxController controller, RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> controller.getHID().setRumble(type,
                                                                                1.0)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
        }

        public boolean isRedAlliance() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red;
                } else {
                        return false;
                }
        }

        public boolean isBlueAlliance() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Blue;
                } else {
                        return false;
                }
        }

        public enum Setpoint {
                kFeederStation,
                kLevel1,
                kLevel2,
                kLevel3,
                kLevel4;
        }

        public static final class ElevatorSetpoints {
                public static final int kFeederStation = 15;
                public static final int kLevel1 = 25;
                public static final int kLevel2 = 30;
                public static final int kLevel3 = 40;
                public static final int kLevel4 = 50;

        }

        public static final class ArmSetpoints {
                public static final double kFeederStation = 5;
                public static final double kLevel1 = 80;
                public static final double kLevel2 = 80;
                public static final double kLevel3 = 80;
                public static final double kLevel4 = 90;
        }

        public static final class CoralSetpoints {
                public static final double kFeederStation = 1100;
                public static final double kReefPlaceL123 = -1100;
                public static final double kReefPlaceL4 = -2000;
                public static final double kStop = 0;
        }

        public static final class AlgaeSetpoints {
                public static final double kReefPickUpL123 = -1100;
                public static final double kDiliver = 2000;
                public static final double kStop = 0;
        }

}
