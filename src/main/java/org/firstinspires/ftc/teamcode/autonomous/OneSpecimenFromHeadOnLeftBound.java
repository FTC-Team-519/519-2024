package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;
import org.firstinspires.ftc.teamcode.util.events.processors.LinearProcessor;
import org.firstinspires.ftc.teamcode.util.events.processors.ParallelProcessor;
import org.firstinspires.ftc.teamcode.util.events.processors.ParallelRaceProcessor;

@Disabled
@Autonomous(name="SpecimenAscentZone(LEFT)",group="autonomi")
public class OneSpecimenFromHeadOnLeftBound extends OpModeBase {
    LinearProcessor linearProcessor;

    public void init() {
        super.init();
        linearProcessor = new LinearProcessor(
                new DriveForward(27.0d, this.robot),
                new ArmRotation(17.0d, this.robot),
                new ParallelRaceProcessor(
                        new ArmStall(17.0,this.robot),
                        new ArmExtension(-1800.0, 1,this.robot)
                ),
                new ParallelProcessor(
                        new IntakePiece(this.robot, 0.7, false),
                        new ArmExtension(0, 1, this.robot)
                ),
                new DriveForward(-5,this.robot),
                new DriveSideways(-34,this.robot),
                new DriveForward(30,this.robot),
                new DriveSideways(10,this.robot)
        );
        linearProcessor.init();
    }

    public void loop() {
        linearProcessor.run();
    }


}

