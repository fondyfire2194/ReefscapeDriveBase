// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.FindCurrentReefZone;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LedStrip;

/** Add your docs here. */
public class CommandFactory {

        SwerveSubsystem m_swerve;

        LedStrip m_ls;

        Pose2d GHZonePoseBlue = new Pose2d(6.75, 4, Rotation2d.fromDegrees(180));
        Pose2d GHZonePoseRed = new Pose2d(10.8, 4, Rotation2d.fromDegrees(0));

        public CommandFactory(SwerveSubsystem swerve, LedStrip ls) {
                m_swerve = swerve;
                m_ls = ls;
        }

        public Command driveToGHZoneCommand() {
                return Commands.parallel(
                                new ConditionalCommand(
                                                m_swerve.driveToPose(GHZonePoseBlue),
                                                m_swerve.driveToPose(GHZonePoseRed),
                                                () -> m_swerve.isBlueAlliance()),
                                new FindCurrentReefZone(m_swerve, m_ls));
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
                kCoralStation,
                kLevel1,
                kLevel2,
                kLevel3,
                kLevel4,
                kProcessorDeliver,
                KAlgaeDeliverBarge,
                kAlgaePickUpL3,
                KAlgaePickUpL2;
        }

        public static final class ElevatorSetpoints {
                public static final int kHome = 0;
                public static final int kProcessorDeliver = 5;
                public static final int kCoralStation = 15;
                public static final int kLevel1 = 25;
                public static final int kLevel2 = 40;
                public static final int kLevel3 = 60;
                public static final int kLevel4 = 75;
                public static final int kBarge = 80;
        }

        public static final class ArmSetpoints {
                public static final int kHome = 0;
                public static final int kProcessorDeliver = 2;
                public static final double kCoralStation = 50;
                public static final double kLevel1 = 80;
                public static final double kLevel2 = 80;
                public static final double kLevel3 = 80;
                public static final double kLevel4 = 90;
                public static final double kAlgaeIntake = 100;
                public static final double kAlgaeBargeDeliver = 120;

        }

        public static final class CoralRPMSetpoints {
                public static final double kCoralStation = 1100;
                public static final double kReefPlaceL123 = -1100;
                public static final double kReefPlaceL4 = -2000;
                public static final double kStop = 0;
        }

        public static final class AlgaeRPMSetpoints {
                public static final double kReefPickUpL123 = -1100;
                public static final double kProcessorDeliver = 2000;
                public static final double kStop = 0;
        }

}
