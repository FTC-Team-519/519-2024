package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name="OneSpecimenFromHeadOnLeftBound",group="autonomi")
public class OneSpecimenFromHeadOnLeftBound extends OpModeBase {
    LinearProcessor linearProcessor;

    public void init() {
        super.init();
        linearProcessor = new LinearProcessor(
                new DriveForward(27.0d, this.robot),
                new ArmRotation(17.0d, this.robot),
                new ArmExtension(-1800.0, this.robot),
                new ParallelProcessor(
                        new IntakePiece(this.robot, 0.7, false),
                        new ArmExtension(0,this.robot)
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

