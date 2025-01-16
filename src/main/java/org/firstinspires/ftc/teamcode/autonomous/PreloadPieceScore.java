package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name = "PreloadSampleScoreNoDiag", group="Linear OpMode")
public class PreloadPieceScore extends OpModeBase {
    private LinearProcessor linProcess = null;

    @Override
    public void init() {
        super.init();
        this.linProcess = new LinearProcessor(new ParallelProcessor(
            new ArmRotation(17.0, this.robot),
            new LinearProcessor(
                new DriveSideways(6.0, this.robot),
                new DriveForward(6.0, this.robot),
                new ParallelProcessor(
                    //new RotateInPlace(17.0, this.robot), //if this was real it would work. Trust. - abdul
                    new ArmExtension(-3570.0, this.robot)
                ),
                new IntakePiece(this.robot,1.0, false)
            )
        ));
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
