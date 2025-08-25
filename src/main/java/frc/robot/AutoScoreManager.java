package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.superstructure.SuperstructureState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class AutoScoreManager {
  @Getter
  @RequiredArgsConstructor
  public enum ScoreLevel {
    kL1(SuperstructureState.kL1),
    kL2(SuperstructureState.kL2),
    kL3(SuperstructureState.kL3);
    // kL4(SuperstructureState.kL4);

    private final SuperstructureState state;
  }

  public interface ScorePosition {
    abstract int getId();
    abstract boolean isRightScore();
    abstract AlgaePosition getAlgaePosition();
  }

  @Getter
  @RequiredArgsConstructor
  public enum ScorePositionBlue implements ScorePosition {
    kA(18, false, AlgaePosition.kAB),
    kB(18, true, AlgaePosition.kAB),
    kC(17, false, AlgaePosition.kCD),
    kD(17, true, AlgaePosition.kCD),
    kE(22, false, AlgaePosition.kEF),
    kF(22, true, AlgaePosition.kEF),
    kG(21, false, AlgaePosition.kGH),
    kH(21, true, AlgaePosition.kGH),
    kI(20, false, AlgaePosition.kIJ),
    kJ(20, true, AlgaePosition.kIJ),
    kK(19, false, AlgaePosition.kKL),
    kL(19, true, AlgaePosition.kGH);

    private final int id;
    private final boolean isRightScore;
    private final AlgaePosition algaePosition;
  }

  @Getter 
  @RequiredArgsConstructor
  public enum ScorePositionRed implements ScorePosition {
    kA(7, false, AlgaePosition.kAB),
    kB(7, true, AlgaePosition.kAB),
    kC(8, false, AlgaePosition.kCD),
    kD(8, true, AlgaePosition.kCD),
    kE(9, false, AlgaePosition.kEF),
    kF(9, true, AlgaePosition.kEF),
    kG(10, false, AlgaePosition.kGH),
    kH(10, true, AlgaePosition.kGH),
    kI(11, false, AlgaePosition.kIJ),
    kJ(11, true, AlgaePosition.kIJ),
    kK(6, false, AlgaePosition.kKL),
    kL(6, true, AlgaePosition.kKL);

    private final int id;
    private final boolean isRightScore;
    private final AlgaePosition algaePosition;
  }

  @Getter
  @RequiredArgsConstructor
  public enum AlgaePosition {
    kAB(2),
    kCD(1),
    kEF(2),
    kGH(1),
    kIJ(2),
    kKL(1);
    
    private final int level;
  }
  
  @Getter
  private boolean isAutoScoreCoal = false;

  @Getter
  private ScoreLevel scoreLevel = null;
  @Getter
  private ScorePosition scorePosition = null;

  private final BooleanPublisher isAutoScoreCoalPublisher = NetworkTableInstance.getDefault()
    .getTable("AutoScore").getBooleanTopic("ScoreCoral").publish();

  private final StringPublisher autoScoreLevelPublisher = NetworkTableInstance.getDefault()
    .getTable("AutoScore").getStringTopic("Level").publish();
  private final StringPublisher autoScorePositionPublisher =  NetworkTableInstance.getDefault()
    .getTable("AutoScore").getStringTopic("Position").publish();

  public AutoScoreManager() {
    isAutoScoreCoalPublisher.set(isAutoScoreCoal);
    autoScoreLevelPublisher.set("");
    autoScorePositionPublisher.set("");

    warmUp();
  }

  // just a warmup method to preload all the score positions, which requrires lots of cpu time (?)
  private void warmUp() {
    ScorePositionBlue.valueOf("kA");
    ScorePositionRed.valueOf("kA");
    ScoreLevel.valueOf("kL1");
  }

  public void setScoreLevel(ScoreLevel newLevel) {
    scoreLevel = newLevel;
    autoScoreLevelPublisher.set(newLevel.name().substring(1));
  }

  public void setScorePosition(String name) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      scorePosition = ScorePositionBlue.valueOf(name);
    } else {
      scorePosition = ScorePositionRed.valueOf(name);
    }
    autoScorePositionPublisher.set(name.substring(1));
  }

  public void toggleEnable() {
    isAutoScoreCoal = !isAutoScoreCoal;
    if (!isAutoScoreCoal) {
      scoreLevel = null;
      scorePosition = null;
      autoScoreLevelPublisher.set("");
      autoScorePositionPublisher.set("");
    }

    isAutoScoreCoalPublisher.set(isAutoScoreCoal);
  }
}
