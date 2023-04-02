package frc.lib.bioniclib;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class Logger {

  ArrayList<ILoggable> m_loggedClasses = new ArrayList<>();

  void register(ILoggable loggable) {
    m_loggedClasses.add(loggable);
  }

  public static class LoggedDouble implements DoubleConsumer {

    private DoublePublisher publisher;
    private DoubleLogEntry logEntry;

    @Override
    public void accept(double arg0) {
      publisher.accept(arg0);
    }
  }
}
