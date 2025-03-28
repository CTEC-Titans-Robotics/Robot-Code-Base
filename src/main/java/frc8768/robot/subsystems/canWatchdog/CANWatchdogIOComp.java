package frc8768.robot.subsystems.canWatchdog;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class CANWatchdogIOComp implements CANWatchdogIO {
  CopyOnWriteArrayList<Integer> missingDevices = new CopyOnWriteArrayList<>();
  private final ObjectReader reader =
      new ObjectMapper()
          .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
          .reader();

  private final Thread canWatchdogThread;

  public CANWatchdogIOComp() {
    canWatchdogThread = new Thread(this::threadFn, "CAN Watchdog Thread");
    canWatchdogThread.setDaemon(true);
    canWatchdogThread.start();
  }

  @Override
  public Stream<Integer> getIds(String jsonBody) {
    try {
      return reader.readTree(jsonBody).findPath("DeviceArray").findValues("ID").stream()
          .map(JsonNode::numberValue)
          .map(Number::intValue);
    } catch (IOException e) {
      // e.printStackTrace();
      return Stream.empty();
    }
  }

  @Override
  public int[] missingDevices() {
    return missingDevices.stream().mapToInt(i -> i).toArray();
  }

  /*
   * Called in thread, do not call this directly
   */
  @Override
  public void threadFn() {
    while (!Thread.currentThread().isInterrupted()) {
      sleep(CANWatchdogConstants.SCAN_DELAY_MS);
      final URI uri = URI.create(CANWatchdogConstants.REQUEST);
      final HttpClient client = HttpClient.newHttpClient();
      try {
        // open a tcp socket to phoenix tuner to get the list of devices
        var body =
            client
                .send(
                    HttpRequest.newBuilder().uri(uri).GET().build(),
                    HttpResponse.BodyHandlers.ofString())
                .body();

        if (body == null) continue;

        // parse the json and check if all the ids we expect are present
        Set<Integer> ids = getIds(body).collect(Collectors.toCollection(HashSet::new));
        if (ids.containsAll(CANWatchdogConstants.CAN.getIds())) {
          missingDevices.clear();
        } else {
          CopyOnWriteArrayList<Integer> missingDevicesLocal =
              new CopyOnWriteArrayList<>(
                  CANWatchdogConstants.CAN.getDevices().stream()
                      .filter(device -> !(ids.contains(device.id())))
                      .mapToInt(device -> device.id())
                      .sorted()
                      .boxed()
                      .collect(Collectors.toCollection(ArrayList::new)));
          if (!missingDevicesLocal.equals(missingDevices)) {
            missingDevices.clear(); // FIXME
            missingDevices = missingDevicesLocal;
          }
        }
      } catch (InterruptedException e) {
        // restore the interrupted status
        Thread.currentThread().interrupt();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }
  /**
   * Sleep that handles interrupts and uses an int. Don't do this please?
   *
   * @param millis
   */
  public void sleep(final int millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    }
  }

  public void matchStarting() {
    canWatchdogThread.interrupt();
  }
}
