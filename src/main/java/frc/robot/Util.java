package frc.robot;

import com.revrobotics.REVLibError;

public final class Util {
    public static final void handleREVLibErr(REVLibError err) {
        if(err != REVLibError.kOk) {
            // TODO: warn if the error is whatever, throw otherwise
            System.err.printf("REVLibError: %s\n", err);
            // TODO: stacktrace or at leas 1 layer up
        }
    }
}