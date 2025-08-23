package frc.robot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;

public class LEDCenter {
  private static LEDCenter m_instance;

  public static LEDCenter getInstance() {
    if (m_instance == null) {
      m_instance = new LEDCenter();
    }
    return m_instance;
  }

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int kCoralIntakeStateCounter = 0;

  public LEDCenter() {
    m_led = new AddressableLED(LEDConstants.kPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setCoralIntake() {
    LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer);
    double speed = 1.5;
    for (int i = kCoralIntakeStateCounter; i < kCoralIntakeStateCounter + 2; i++) {
      m_ledBuffer.setLED(i, Color.kWhite);
      m_ledBuffer.setLED(LEDConstants.kLength - i - 1, Color.kWhite);
    }
    kCoralIntakeStateCounter += speed;
    if (kCoralIntakeStateCounter > LEDConstants.kLength / 3 + 1) {
      kCoralIntakeStateCounter = 0;
    }
    m_led.setData(m_ledBuffer);
  }

  public void setBlink() {
    LEDPattern base = LEDPattern.solid(Color.kMediumPurple);
    LEDPattern pattern = base.blink(Seconds.of(0.1));
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setScrollingRainbow() {
    LEDPattern pattern = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Value.per(Second).of(0.5));
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setBreath() {
    LEDPattern pattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue).breathe(Seconds.of(4));
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setDefault() {
    LEDPattern pattern = LEDPattern.solid(DriverStation.getAlliance().get() == Alliance.Red ? Color.kFirstRed : Color.kFirstBlue);
    pattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setProgress(double value) {
    Color color = DriverStation.getAlliance().get() == Alliance.Red ? Color.kOrangeRed : Color.kCornflowerBlue;
    LEDPattern.solid(Color.kBlack).applyTo(m_ledBuffer);
    double len = value >= 1 ? 12 : value * 12;
    for (int i = 11; i > 11 - len; i--) {
      m_ledBuffer.setLED(i, color);
    }
    for (int i = 24; i < 24 + len; i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setRSLSync() {
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    LEDPattern sycned = base.synchronizedBlink(RobotController::getRSLState);

    // Apply the LED pattern to the data buffer
    sycned.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
  }
}
