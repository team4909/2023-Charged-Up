package frc.lib.bioniclib;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Logger {

  private final NetworkTableInstance NT = NetworkTableInstance.getDefault();
  private final NetworkTable logTable = NT.getTable("Robot");
  private BooleanSupplier useNT = () -> !getCachedFMSPresence();
  private boolean seenFMSCache = false;

  ArrayList<ILoggable> m_loggedClasses = new ArrayList<>();

  void register(ILoggable loggable) {
    m_loggedClasses.add(loggable);
  }

  public class LoggedDouble implements DoubleConsumer {

    private DoubleTopic topic;
    private DoublePublisher publisher;
    private DoubleLogEntry logEntry;

    public LoggedDouble(String subTable, String name) {
      topic = logTable.getSubTable(subTable).getDoubleTopic(name);
      publisher = topic.publish();
      logEntry = new DoubleLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
    }

    @Override
    public void accept(double value) {
      if (useNT.getAsBoolean()) {
        publisher.accept(value);
      } else {
        logEntry.append(value);
      }
    }
  }

  private boolean getCachedFMSPresence() {
    if (seenFMSCache)
      return true;
    else if (DriverStation.isFMSAttached()) {
      seenFMSCache = true;
      return true;
    } else {
      return false;
    }
  }
}
