package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name="OneSpecimenFromHeadOnRightBound",group="autonomi")
public class OneSpecimenFromHeadOnRightBound extends OpModeBase {
    LinearProcessor linearProcessor;

    public void init() {
        super.init();
        linearProcessor = new LinearProcessor(
                new DriveForward(27.0d, this.robot),
                new ArmRotation(17.0d, this.robot),
                new ParallelRaceProcessor(
                        new ArmExtension(-1800.0,this.robot),
                        new ArmStall(17.0d, this.robot)

                ),
                new ParallelProcessor(
                        new IntakePiece(this.robot, 0.7, false),
                        new ArmExtension(0,this.robot)
                ),
                new DriveForward(-27.0d,this.robot),
                new DriveSideways(52,this.robot)
        );
        linearProcessor.init();
    }

    public void loop() {
        linearProcessor.run();
    }


}
