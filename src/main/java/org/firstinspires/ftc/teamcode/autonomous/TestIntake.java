package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.OpModeBase;
import org.firstinspires.ftc.teamcode.util.events.IntakePiece;
import org.firstinspires.ftc.teamcode.util.events.LinearProcessor;
import org.firstinspires.ftc.teamcode.util.events.ZeroArmAngle;

@TeleOp(name = "TestIntake", group="Linear OpMode")
//@Disabled
public class TestIntake extends OpModeBase {
    private boolean buttonAPressed = false;
    private LinearProcessor linProcess = null;
    private IntakePiece intake ;

    @Override
    public void init() {
        super.init();
        this.intake = new IntakePiece(this.robot,1.0, true);
        this.linProcess = new LinearProcessor(this.intake, new IntakePiece(this.robot,1.0, false));
    }

    @Override
    public void loop() {
        if (!linProcess.isDone()){
            linProcess.run();
        }
        linProcess.shutdown();
        telemetry.addData("samples gained:", intake.curr_samples);
        if (!this.buttonAPressed && gamepad1.a){
            return;
        }

        this.buttonAPressed = gamepad1.a;
        telemetry.update();
    }
}
