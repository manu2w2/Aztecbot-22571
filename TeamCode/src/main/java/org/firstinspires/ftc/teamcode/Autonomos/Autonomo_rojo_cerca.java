package org.firstinspires.ftc.teamcode.Autonomos;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import Subsistemas.IntakeSubsystem_Autonomous;
import Subsistemas.ServosSubsystem_Autonomous;
import Subsistemas.ShooterSubsystem;
import Subsistemas.TurretSubsystem_Autonomous;
@Autonomous(name = "Autonomo rojo cerca", group = "Autonomous")
@Configurable
public class Autonomo_rojo_cerca extends CommandOpMode {

    private Follower follower;
    public static double Velocity1= 0;
    public static double TurretPosition1 = -191;
    public static double TurretPosition2 = -100;
    private IntakeSubsystem_Autonomous intake;
    private TurretSubsystem_Autonomous turret;
    private ShooterSubsystem shooter;
    private ServosSubsystem_Autonomous servos;
    private final Pose startPose = new Pose(117.30167903462707, 126.0591850106079, Math.toRadians(126.5));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3parte2, Ciclo4,Ciclo4parte2,Ciclo5,CIclo5part2;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
                                new Pose(117.30167903462707, 126.0591850106079),
                                new Pose(87.575, 82.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-126.5), Math.toRadians(0))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.575, 82.888),
                                new Pose(98.276, 58.763)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(98.276, 58.763),
                                new Pose(122.924, 58.702)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(122.924, 58.702),
                                new Pose(95.064, 60.047),
                                new Pose(87.575, 82.888)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.575, 82.888),
                                new Pose(106.748, 55.456),
                                new Pose(128.8790, 58.332)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28))
                .build();

        Ciclo3parte2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.8790, 58.332),
                                new Pose(99.593, 62.236),
                                new Pose(87.574, 82.888)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(28))
                .build();

        Ciclo4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.574, 82.888),
                                new Pose(106.747, 55.456),
                                new Pose(128.8790, 58.332)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(28))
                .build();

        Ciclo4parte2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.8790, 58.332),
                                new Pose(99.593, 62.236),
                                new Pose(87.574, 82.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(0))
                .build();

        Ciclo5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.574, 82.888),
                                new Pose(125.051, 82.302)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        CIclo5part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.051, 82.302),
                                new Pose(87.574, 82.888)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(

                new ParallelCommandGroup(
                        new SpinUpShooterCommand(shooter,Velocity1,20),
                        new SetTurretPositionCommand(turret,TurretPosition1),
                        new FollowPathCommand(follower, ciclo1, true, 0.95),
                        new CloseTopeCommand(servos),
                        new SetHoodPositionCommand(servos,0.2)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake,1750),
                        new OpenTopeCommand(servos)
                ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new SpinUpShooterCommand(shooter,400,20),
                        new FollowPathCommand(follower, ciclo2, true, 1),
                        new CloseTopeCommand(servos)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower,ciclo2part2,true,1),
                        new SpinUpShooterCommand(shooter,Velocity1,20),
                        new SetIntakeVelocityCommand(intake,0)),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake,1750),
                        new OpenTopeCommand(servos)
                ),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new SetTurretPositionCommand(turret,TurretPosition2),
                        new SpinUpShooterCommand(shooter,400,20),
                        new FollowPathCommand(follower, Ciclo3, true, 1),
                        new CloseTopeCommand(servos),
                        new SetIntakeVelocityCommand(intake,1400)
                ),
                new WaitCommand(2000),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo3parte2, true, 1),
                        new SpinUpShooterCommand(shooter,   Velocity1,20),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new SetIntakeVelocityCommand(intake,1750),
                                new OpenTopeCommand(servos)
                        ),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new SetIntakeVelocityCommand(intake,1500),
                                new SpinUpShooterCommand(shooter,400,20),
                                new FollowPathCommand(follower, Ciclo4, true, 1),
                                new CloseTopeCommand(servos)
                        ),
                        new WaitCommand(2500),
                        new ParallelCommandGroup(
                                new SpinUpShooterCommand(shooter,Velocity1,20),
                                new FollowPathCommand(follower, Ciclo4parte2, true, 0.95),
                                new SetIntakeVelocityCommand(intake,0))
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake,1750),
                        new OpenTopeCommand(servos)
                ),
                new WaitCommand(1000),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower,Ciclo5,true,1),
                        new CloseTopeCommand(servos),
                        new SetIntakeVelocityCommand(intake,1500),
                        new SetTurretPositionCommand(turret,TurretPosition1)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower,CIclo5part2,true,1),
                        new SetIntakeVelocityCommand(intake,0)
                ),
                new WaitCommand(200),

                new ParallelCommandGroup(
                        new OpenTopeCommand(servos),
                        new SetIntakeVelocityCommand(intake,1750)
                ),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 0),
                        new SpinUpShooterCommand(shooter,0,20),
                        new SetTurretPositionCommand(turret,0),
                        new CloseTopeCommand(servos)

                )

        );
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}