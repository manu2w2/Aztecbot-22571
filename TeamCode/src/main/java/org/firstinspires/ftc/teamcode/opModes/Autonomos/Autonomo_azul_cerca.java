package org.firstinspires.ftc.teamcode.opModes.Autonomos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.CloseTopeCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.OpenTopeCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SetHoodPositionCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SetIntakeVelocityCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SetTurretPositionCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SpinUpShooterCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import Subsistemas.HoodSubsystem;
import Subsistemas.IntakeSubsystem_Autonomous;
import Subsistemas.ShooterSubsystem;
import Subsistemas.TopeSubsystem;
import Subsistemas.TurretSubsystem_Autonomous;
@Autonomous(name = "Autonomo azul cerca", group = "Autonomous")
@Configurable
public class Autonomo_azul_cerca extends CommandOpMode {
    private Follower follower;
    public static double Velocity1= 1150;
    public static double TurretPosition1 = 198;
    private IntakeSubsystem_Autonomous intake;
    private TurretSubsystem_Autonomous turret;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private TopeSubsystem Tope;
    private final Pose startPose = new Pose(24.223050486238442, 126.05918501060792, Math.toRadians(53.5));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3Part2, Ciclo4,AbrirGate,Ciclo4Part2,Salir;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new IntakeSubsystem_Autonomous(hardwareMap);
        turret = new TurretSubsystem_Autonomous(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);
        Tope = new TopeSubsystem(hardwareMap);
        register(intake, turret, shooter, hood, Tope);

        buildPaths();
        waitForStart();
        if (opModeIsActive()) {
            schedule(createAutonomousSequence());
        }
    }

    public void buildPaths() {
        // Mantengo exactamente tus paths originales
        ciclo1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24.223, 126.059),
                                new Pose(56.425, 82.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(53.5), Math.toRadians(180))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.425, 82.888),
                                new Pose(45.725, 58.763)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(45.725, 58.763),
                                new Pose(21.076, 58.702)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.887, 64.402),
                                new Pose(46.529, 62.179),
                                new Pose(56.425, 82.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        AbrirGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(21.076, 58.702),
                                new Pose(26.276, 64.958),
                                new Pose(16.887, 64.402)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.425, 82.888),
                                new Pose(22.987, 82.522)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3Part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.987, 82.522),
                                new Pose(56.425, 82.888)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.425, 82.888),
                                new Pose(44.889, 35.528)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(44.889, 35.528),
                                new Pose(21.275, 35.192)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo4Part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.275, 35.192),
                                new Pose(56.425, 82.888)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Salir = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.425, 82.888),
                                new Pose(44.478, 76.308)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(

        new ParallelCommandGroup(
                new SpinUpShooterCommand(shooter, Velocity1, 20),
                new SetTurretPositionCommand(turret, TurretPosition1),
                new FollowPathCommand(follower, ciclo1, true, 1),
                new CloseTopeCommand(Tope),
                new SetHoodPositionCommand(hood, 0.265)
        ),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1650),
                        new OpenTopeCommand(Tope)
                ),

                new WaitCommand(1400),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, ciclo2, true, 1),
                        new CloseTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake, 1550)
                ),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, AbrirGate, true, 0.9),
                        new SetIntakeVelocityCommand(intake, 0)

                ),
                new WaitCommand(500),

                new FollowPathCommand(follower, ciclo2part2, true, 1),

                new WaitCommand(200),


                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1650),
                        new OpenTopeCommand(Tope)
                ),

                new WaitCommand(1400),

                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1550),
                        new CloseTopeCommand(Tope),
                        new FollowPathCommand(follower, Ciclo3, true, 1)
                ),
                new WaitCommand(200),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo3Part2, true, 1),
                        new SetIntakeVelocityCommand(intake, 0)
                ),
                new WaitCommand(200),

                new ParallelCommandGroup(
                        new OpenTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake, 1650)
                ),
                new WaitCommand(1400),

                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1550),
                        new FollowPathCommand(follower, Ciclo4, true, 1),
                        new CloseTopeCommand(Tope)
                ),
                new WaitCommand(200),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo4Part2, true, 1),
                        new SetIntakeVelocityCommand(intake, 0)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new OpenTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake, 1650)
                ),
                new WaitCommand(1400),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 0),
                        new SpinUpShooterCommand(shooter, 0, 20),
                        new SetTurretPositionCommand(turret, 0),
                        new CloseTopeCommand(Tope),
                        new FollowPathCommand(follower, Salir, true, 1)
                )








        );
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}