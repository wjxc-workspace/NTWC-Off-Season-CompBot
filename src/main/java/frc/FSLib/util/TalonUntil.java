package frc.FSLib.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public class TalonUntil {
  public static void assertOk(Supplier<StatusCode> code) {
    StatusCode status = code.get();
    if (status != StatusCode.OK) {
      DriverStation.reportError("can't apply configuration", true);
    }
  }
}
