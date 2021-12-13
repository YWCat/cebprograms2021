package org.firstinspires.ftc.teamcode.hardware;


import android.util.Log;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class FakeGamepad {

    class GamepadState {
        public long milliSecondsElapsed;
        public boolean a;
        public boolean y;

        public GamepadState(long milliSecondsElapsed, boolean a, boolean y) {
            this.milliSecondsElapsed = milliSecondsElapsed;
            this.a = a;
            this.y = y;
        }
    }

    private List<FakeGamepad.GamepadState> gamepadStates;
    private int currentStateIndex = 0;
    private long initialTime = -1;

    public FakeGamepad() {
        gamepadStates = new ArrayList<>();
        gamepadStates.add(new GamepadState(0,    false, false));
        gamepadStates.add(new GamepadState(2000, false, true));
        gamepadStates.add(new GamepadState(2010, false, false));
        gamepadStates.add(new GamepadState(3000, false, true));
        gamepadStates.add(new GamepadState(3010, false, false));
    }

    private void updateIndex() {

        boolean indexChanged = false;

        if (initialTime < 0) {
            initialTime = System.currentTimeMillis();
        }

        long currentTime = System.currentTimeMillis();
        if(currentStateIndex < gamepadStates.size()-1) {
            for (int index = currentStateIndex + 1; index < gamepadStates.size(); index++) {
                if (gamepadStates.get(index).milliSecondsElapsed < currentTime - initialTime) {
                    currentStateIndex = index;
                    indexChanged = true;
                } else {
                    break;
                }
            }

            if (indexChanged) {
                Log.i("slide", String.format("Fake gamepad is now using gamepad state at index %d", currentStateIndex));
            }
        }
    }

    public boolean a() {

        updateIndex();
        return gamepadStates.get(currentStateIndex).a;
    }

    public boolean y() {

        updateIndex();
        return gamepadStates.get(currentStateIndex).y;

    }

}
