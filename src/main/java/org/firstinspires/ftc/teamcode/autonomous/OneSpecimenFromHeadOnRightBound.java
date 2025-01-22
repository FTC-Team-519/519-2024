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
                        new ArmExtension(-1700.0, 1, this.robot),
                        new ArmStall(17.0d, this.robot)

                ),
                new ParallelProcessor(
                        new ArmRotation(24.2,this.robot,0.5),
                        new DriveForward(2.0d,this.robot)),
                new Timer(1000),
                new ParallelProcessor(
                        new IntakePiece(this.robot, 0.25, false),
                        new ArmExtension(0, 1, this.robot),
                        new ArmRotation(24.2,this.robot,0.6)
                ),
                new DriveForward(-25.0d,this.robot),
                new DriveSideways(60,this.robot),
                new DriveForward(-4.0d,this.robot)
        );
        linearProcessor.init();
    }

    public void loop() {
        linearProcessor.run();
    }


}
