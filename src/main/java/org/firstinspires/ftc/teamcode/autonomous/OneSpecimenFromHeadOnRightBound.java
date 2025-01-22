package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

//@Disabled
@Autonomous(name="SpecimenObservationZone(RIGHT)",group="autonomi")
public class OneSpecimenFromHeadOnRightBound extends OpModeBase {
    LinearProcessor linearProcessor;

    public void init() {
        super.init();
        linearProcessor = new LinearProcessor(
                new DriveForward(25.0d, this.robot),
                new ArmRotation(17.0d, this.robot),
                new ParallelRaceProcessor(
                        new ArmExtension(-1800.0, 1, this.robot),
                        new ArmStall(17.0d, this.robot)

                ),
                new ParallelProcessor(
                        new ArmRotation(25.0,this.robot),
                        new DriveForward(2.0d,this.robot)),
                new ParallelProcessor(
                        new IntakePiece(this.robot, 0.7, false),
                        new ArmExtension(0, 1, this.robot)
                ),
                new DriveForward(-27.0d,this.robot),
                new DriveSideways(52,this.robot),
                new DriveForward(-3.0d,this.robot)
        );
        linearProcessor.init();
    }

    public void loop() {
        linearProcessor.run();
    }


}
