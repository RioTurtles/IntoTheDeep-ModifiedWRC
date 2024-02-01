package org.firstinspires.ftc.teamcode;

import java.util.Map;

import com.qualcomm.robotcore.hardware.Gamepad;


public class IntuitiveGamepad_v2 {
    Gamepad gamepad;
    Map<Key, Callback> keyMap;
    Map<Key, Double> settingMap;

    public IntuitiveGamepad_v2(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.keyMap.clear();
    }

    public void mapControl(Key button, Callback callback) {keyMap.put(button, callback);}

    public void setHoldDuration(Key button, double duration) {settingMap.put(button, duration);}

    public abstract static class Callback {
        ActionType actionType;
        public Callback(ActionType actionType) {this.actionType = actionType;}
        public abstract void callback();
    }

    public static class Keys {
        public static final Key a = new Key("a");
        public static final Key b = new Key("b");
        public static final Key back = new Key("back");
        public static final Key circle = new Key("circle");
        public static final Key cross = new Key("cross");
        public static final Key dpad_down = new Key("dpad_down");
        public static final Key dpad_left = new Key("dpad_left");
        public static final Key dpad_right = new Key("dpad_right");
        public static final Key dpad_up = new Key("dpad_up");
        public static final Key guide = new Key("guide");
        public static final Key left_bumper = new Key("left_bumper");
        public static final Key left_stick_button = new Key("left_stick_button");
        public static final Key options = new Key("options");
        public static final Key ps = new Key("ps");
        public static final Key right_bumper = new Key("right_bumper");
        public static final Key right_stick_button = new Key("right_stick_button");
        public static final Key share = new Key("share");
        public static final Key square = new Key("square");
        public static final Key start = new Key("start");
        public static final Key touchpad = new Key("touchpad");
        public static final Key touchpad_finger_1 = new Key("touchpad_finger_1");
        public static final Key triangle = new Key("triangle");
        public static final Key x = new Key("x");
        public static final Key y = new Key("y");
    }
    public static class ActionType {
        public static final Action hit = new Action("press");
        public static final Action toggle = new Action("toggle");
        public static final Action hold = new Action("hold");
    }

    public static class Key {String key; public Key(String key) {this.key = key;}}
    public static class Action {String actionType; public Action(String actionType) {this.actionType = actionType;}}
}
