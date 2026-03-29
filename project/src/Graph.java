import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class Graph {

  private static class Edge {

    final long targetId;
    final double dist;
    final String streetName;

    Edge(long targetId, double dist, String streetName) {
      this.targetId = targetId;
      this.dist = dist;
      this.streetName = streetName;
    }
  }

  private final Map<Long, Localisation> nodes = new HashMap<>();

  private final Map<Long, List<Edge>> adj = new HashMap<>();

  private final double epsilon = 0.0;

  private static final double K_DEFAULT = 0.05;

  public Graph(String localisations, String roads) {
    loadNodes(localisations);
    loadEdges(roads);
  }

  private void loadNodes(String file) {
    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line = br.readLine();
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          continue;
        }
        String[] parts = line.split(",", 5);
        long id = Long.parseLong(parts[0]);
        String name = parts[1];
        double lat = Double.parseDouble(parts[2]);
        double lon = Double.parseDouble(parts[3]);
        double alt = Double.parseDouble(parts[4]);
        Localisation loc = new Localisation(id, lat, lon, name, alt);
        nodes.put(id, loc);
        adj.putIfAbsent(id, new ArrayList<>());
      }
    } catch (IOException e) {
      throw new RuntimeException("Error loading nodes from " + file, e);
    }
  }

  private void loadEdges(String file) {
    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line = br.readLine();
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          continue;
        }
        String[] parts = line.split(",", 4);
        long source = Long.parseLong(parts[0]);
        long target = Long.parseLong(parts[1]);
        double dist = Double.parseDouble(parts[2]);
        String streetName = parts[3];
        adj.computeIfAbsent(source, k -> new ArrayList<>()).add(new Edge(target, dist, streetName));
      }
    } catch (IOException e) {
      throw new RuntimeException("Error loading edges from " + file, e);
    }
  }

  // Algo 1 : zone inondée
  public Localisation[] determinerZoneInondee(long[] idsOrigin, double epsilon) {
    Set<Long> visited = new HashSet<>();
    Deque<Long> queue = new ArrayDeque<>();
    List<Localisation> order = new ArrayList<>();

    for (long id : idsOrigin) {
      if (!nodes.containsKey(id)) {
        continue;
      }
      visited.add(id);
      queue.add(id);
    }

    while (!queue.isEmpty()) {
      long currentId = queue.poll();
      Localisation current = nodes.get(currentId);
      order.add(current);

      List<Edge> edges = adj.get(currentId);
      if (edges == null) {
        continue;
      }

      for (Edge e : edges) {
        long neighId = e.targetId;
        if (visited.contains(neighId)) {
          continue;
        }
        Localisation neigh = nodes.get(neighId);
        if (neigh == null) {
          continue;
        }
        if (neigh.getAltitude() <= current.getAltitude() + epsilon) {
          visited.add(neighId);
          queue.add(neighId);
        }
      }
    }

    return order.toArray(new Localisation[0]);
  }

  public Localisation[] determinerZoneInondee(long[] idsOrigin) {
    return determinerZoneInondee(idsOrigin, this.epsilon);
  }

  // Algo 2 : Chemin court (rues)
  public Deque<Localisation> trouverCheminLePlusCourtPourContournerLaZoneInondee(long idOrigin,
      long idDestination, Localisation[] floodedZone) {

    Set<Long> flooded = new HashSet<>();
    if (floodedZone != null) {
      for (Localisation l : floodedZone) {
        flooded.add(l.getId());
      }
    }

    if (flooded.contains(idOrigin) || flooded.contains(idDestination)) {
      return new ArrayDeque<>();
    }

    Deque<Long> queue = new ArrayDeque<>();
    Map<Long, Long> parent = new HashMap<>();
    Set<Long> visited = new HashSet<>();

    queue.add(idOrigin);
    visited.add(idOrigin);
    parent.put(idOrigin, null);

    boolean found = false;

    outer:
    while (!queue.isEmpty()) {
      long currentId = queue.poll();
      List<Edge> edges = adj.get(currentId);
      if (edges == null) {
        continue;
      }

      for (Edge e : edges) {
        long neighId = e.targetId;
        if (visited.contains(neighId) || flooded.contains(neighId)) {
          continue;
        }

        visited.add(neighId);
        parent.put(neighId, currentId);
        queue.add(neighId);

        if (neighId == idDestination) {
          found = true;
          break outer;
        }
      }
    }

    Deque<Localisation> path = new ArrayDeque<>();
    if (!found) {
      return path;
    }

    Long current = idDestination;
    while (current != null) {
      Localisation loc = nodes.get(current);
      if (loc != null) {
        path.addFirst(loc);
      }
      current = parent.get(current);
    }

    return path;
  }

  // Algo 3 : Chronologie
  public Map<Localisation, Double> determinerChronologieDeLaCrue(long[] idsOrigin,
      double vWaterInit, double k) {

    Map<Long, Double> dist = new HashMap<>();

    Map<Long, Double> vAtNode = new HashMap<>();

    PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));

    for (long id : idsOrigin) {
      if (!nodes.containsKey(id)) {
        continue;
      }
      if (!dist.containsKey(id) || dist.get(id) > 0.0) {
        dist.put(id, 0.0);
        vAtNode.put(id, vWaterInit);
        pq.offer(new double[]{0.0, id, vWaterInit});
      }
    }

    while (!pq.isEmpty()) {
      double[] top = pq.poll();
      double time = top[0];
      long currentId = (long) top[1];
      double vCurrent = top[2];

      Double best = dist.get(currentId);
      if (best != null && time > best) {
        continue;
      }

      Localisation current = nodes.get(currentId);
      List<Edge> edges = adj.get(currentId);
      if (edges == null) {
        continue;
      }

      for (Edge e : edges) {
        long neighId = e.targetId;
        Localisation neigh = nodes.get(neighId);
        if (neigh == null) {
          continue;
        }

        double slope = (current.getAltitude() - neigh.getAltitude()) / e.dist;

        double vNext = vCurrent + k * slope;

        if (vNext <= 0) {
          continue;
        }

        double travelTime = e.dist / vNext;
        double newTime = time + travelTime;

        Double prevTime = dist.get(neighId);
        if (prevTime == null || newTime < prevTime) {
          dist.put(neighId, newTime);
          vAtNode.put(neighId, vNext);
          pq.offer(new double[]{newTime, neighId, vNext});
        }
      }
    }

    Map<Localisation, Double> result = new LinkedHashMap<>();
    dist.entrySet().stream().sorted(Map.Entry.comparingByValue()).forEach(entry -> {
      Localisation loc = nodes.get(entry.getKey());
      if (loc != null) {
        result.put(loc, entry.getValue());
      }
    });

    return result;
  }

  public Map<Localisation, Double> determinerChronologieDeLaCrue(long[] idsOrigin,
      double vWaterInit) {
    return determinerChronologieDeLaCrue(idsOrigin, vWaterInit, K_DEFAULT);
  }

  // Algo 4 :Évacuation dynamique
  public Deque<Localisation> trouverCheminDEvacuationLePlusCourt(long idDepart, long idEvacuation,
      double vVehicule, Map<Localisation, Double> tFlood) {

    Map<Long, Double> floodTime = new HashMap<>();
    for (Map.Entry<Localisation, Double> entry : tFlood.entrySet()) {
      floodTime.put(entry.getKey().getId(), entry.getValue());
    }

    Map<Long, Double> dist = new HashMap<>();
    Map<Long, Long> parent = new HashMap<>();

    PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));

    dist.put(idDepart, 0.0);
    parent.put(idDepart, null);
    pq.offer(new double[]{0.0, idDepart});

    boolean found = false;

    while (!pq.isEmpty()) {
      double[] top = pq.poll();
      double time = top[0];
      long currentId = (long) top[1];

      if (currentId == idEvacuation) {
        found = true;
        break;
      }

      Double best = dist.get(currentId);
      if (best != null && time > best) {
        continue;
      }

      List<Edge> edges = adj.get(currentId);
      if (edges == null) {
        continue;
      }

      for (Edge e : edges) {
        long neighId = e.targetId;
        if (!nodes.containsKey(neighId)) {
          continue;
        }

        double travelTime = e.dist / vVehicule;
        double arrivalTime = time + travelTime;
        
        Double flood = floodTime.get(neighId);
        if (flood != null && arrivalTime > flood) {
          continue;
        }

        Double prevTime = dist.get(neighId);
        if (prevTime == null || arrivalTime < prevTime) {
          dist.put(neighId, arrivalTime);
          parent.put(neighId, currentId);
          pq.offer(new double[]{arrivalTime, neighId});
        }
      }
    }

    Deque<Localisation> path = new ArrayDeque<>();
    if (!found) {
      return path;
    }

    Long current = idEvacuation;
    while (current != null) {
      Localisation loc = nodes.get(current);
      if (loc != null) {
        path.addFirst(loc);
      }
      current = parent.get(current);
    }

    return path;
  }
}