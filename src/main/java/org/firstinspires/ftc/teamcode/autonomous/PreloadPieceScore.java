package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name = "PreloadSampleScoreNoDiag", group="Linear OpMode")
public class PreloadPieceScore extends OpModeBase {
    private LinearProcessor linProcess = null;

    @Override
    public void init() {
        super.init();
        this.linProcess = new LinearProcessor(
            new ParallelRaceProcessor(
                new ArmStall(17, this.robot),
                new LinearProcessor(
                    new DriveSideways(6.0, this.robot),
                    new DriveForward(7.0, this.robot),
                    new ParallelProcessor(
                        new Rotate(-30.0, false, 0.75, this.robot),
                        new ArmExtension(-4572, this.robot)
                    ),
                    new DriveForward(2.0, this.robot),
                    new DriveSideways(5.0, this.robot),

                    new IntakePiece(this.robot,1.0, false),

                    new DriveForward(-2.0, this.robot),
                    new ArmExtension(0, this.robot)
                )
            ),
            new LinearProcessor(
                    new ParallelProcessor(
                        new ArmRotation(57.0, this.robot),
                        new Rotate(103, true, 0.75, this.robot)
                    ),
                    new ParallelRaceProcessor(
                        new ArmExtension(-3571, this.robot),
                        new IntakePiece(this.robot, 1.0, true)
                    )
            )
        );
        this.linProcess.init();
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }
        linProcess.shutdown();
    }
}
