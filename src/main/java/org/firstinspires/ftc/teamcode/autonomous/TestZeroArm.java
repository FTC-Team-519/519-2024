package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;
import org.firstinspires.ftc.teamcode.util.events.ZeroArmAngle;

@Disabled
@TeleOp(name = "TestZeroArm", group="Linear OpMode")
public class TestZeroArm extends OpModeBase {
    private boolean buttonAPressed = false;
    private LinearProcessor linProcess = null;

    @Override
    public void init() {
        super.init();
        this.linProcess = new LinearProcessor(new ZeroArmAngle(this.robot));
        linProcess.init();
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }

        if (!this.buttonAPressed && gamepad1.a){
            return;
        }

        this.buttonAPressed = gamepad1.a;
    }
}
