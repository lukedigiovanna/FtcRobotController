package org.firstinspires.ftc.teamcode.utils;

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Input {
    public Gamepad inputPad;
    private HashMap buttonInputs;
    private HashMap axisInputs;
    public Input(Gamepad inputPad)  {
        this.inputPad = inputPad;
    }

    public void update()    {

    }
}

