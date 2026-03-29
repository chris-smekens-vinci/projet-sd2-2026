public class Localisation {

  private final long id;
  private final double lat;
  private final double lon;
  private final String name;
  private final double altitude;

  public Localisation(long id, double lat, double lon, String name, double altitude) {
    this.id = id;
    this.lat = lat;
    this.lon = lon;
    this.name = name;
    this.altitude = altitude;
  }

  public long getId() {
    return id;
  }

  public double getAltitude() {
    return altitude;
  }

  @Override
  public String toString() {
    return "Localisation{" +
        "id=" + id +
        ", name='" + name + '\'' +
        ", altitude=" + altitude +
        '}';
  }
}