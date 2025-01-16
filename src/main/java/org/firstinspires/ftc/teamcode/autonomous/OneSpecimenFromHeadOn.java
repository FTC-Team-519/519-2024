package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.*;

@Autonomous(name="OneSpecimenFromHeadOn",group="autonomi")
public class OneSpecimenFromHeadOn extends OpModeBase {
    LinearProcessor linearProcessor;

    public void init() {
        super.init();
        linearProcessor = new LinearProcessor(
                new DriveForward(27, this.robot),
                new ArmRotation(17, this.robot),
                new ArmExtension(-3571, this.robot)
        );
        linearProcessor.init();
    }

    public void loop() {
        linearProcessor.run();
    }

}
