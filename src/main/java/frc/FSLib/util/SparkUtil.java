package frc.FSLib.util;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkUtil {
  public static void assertOk(Supplier<REVLibError> code) {
    REVLibError error = code.get();
    if (error != REVLibError.kOk) {
      DriverStation.reportError("can't apply configuration", true);
    }
  }
}
