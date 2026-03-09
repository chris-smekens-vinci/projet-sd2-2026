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

  // Map id -> localisation
  private final Map<Long, Localisation> nodes = new HashMap<>();
  // Adjacency list: id -> outgoing edges
  private final Map<Long, List<Edge>> adj = new HashMap<>();

  // Default epsilon (tolerance) for flood propagation, can be changed or parameterized
  private final double epsilon = 0.0;

  /**
   * localisations: path to nodes CSV (nodes_X.csv) roads: path to edges CSV (edges_X.csv)
   */
  public Graph(String localisations, String roads) {
    loadNodes(localisations);
    loadEdges(roads);
  }

  private void loadNodes(String file) {
    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line = br.readLine(); // header
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
      String line = br.readLine(); // header
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          continue;
        }
        String[] parts = line.split(",", 4);
        long source = Long.parseLong(parts[0]);
        long target = Long.parseLong(parts[1]);
        double dist = Double.parseDouble(parts[2]);
        String streetName = parts[3];
        // assuming edges file already contains both directions if needed
        adj.computeIfAbsent(source, k -> new ArrayList<>())
            .add(new Edge(target, dist, streetName));
      }
    } catch (IOException e) {
      throw new RuntimeException("Error loading edges from " + file, e);
    }
  }

  /**
   * Algorithme 1 : Simulation de la Crue (phase statique). idsOrigin: starting flooded nodes ids.
   * epsilon: tolerance for propagation. Returns an array of Localisation in the order they are
   * flooded (BFS-like).
   */
  public Localisation[] determinerZoneInondee(long[] idsOrigin, double epsilon) {
    // BFS/DFS on directed graph with propagation condition:
    // Alt(Y) <= Alt(X) + epsilon
    Set<Long> visited = new HashSet<>();
    Deque<Long> queue = new ArrayDeque<>();
    List<Localisation> order = new ArrayList<>();

    // initialize queue with origins
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

  // Convenience overload using default epsilon
  public Localisation[] determinerZoneInondee(long[] idsOrigin) {
    return determinerZoneInondee(idsOrigin, this.epsilon);
  }

  /**
   * Algorithme 2 : Navigation de Secours. Find shortest path (in number of streets) from idOrigin
   * to idDestination, avoiding all nodes in floodedZone. Returns a Deque of Localisation from
   * origin to destination.
   */
  public Deque<Localisation> trouverCheminLePlusCourtPourContournerLaZoneInondee(long idOrigin,
      long idDestination,
      Localisation[] floodedZone) {
    // Build set of flooded ids for O(1) checks
    Set<Long> flooded = new HashSet<>();
    if (floodedZone != null) {
      for (Localisation l : floodedZone) {
        flooded.add(l.getId());
      }
    }

    // Standard BFS shortest path in unweighted graph
    Deque<Long> queue = new ArrayDeque<>();
    Map<Long, Long> parent = new HashMap<>();
    Set<Long> visited = new HashSet<>();

    // If origin or destination is flooded, path is impossible
    if (flooded.contains(idOrigin) || flooded.contains(idDestination)) {
      return new ArrayDeque<>();
    }

    queue.add(idOrigin);
    visited.add(idOrigin);
    parent.put(idOrigin, null);

    boolean found = false;

    while (!queue.isEmpty() && !found) {
      long currentId = queue.poll();
      List<Edge> edges = adj.get(currentId);
      if (edges == null) {
        continue;
      }

      for (Edge e : edges) {
        long neighId = e.targetId;
        if (visited.contains(neighId)) {
          continue;
        }
        if (flooded.contains(neighId)) {
          continue;
        }

        visited.add(neighId);
        parent.put(neighId, currentId);
        queue.add(neighId);

        if (neighId == idDestination) {
          found = true;
          break;
        }
      }
    }

    Deque<Localisation> path = new ArrayDeque<>();
    if (!found) {
      return path; // empty if no path
    }

    // Reconstruct path from destination to origin using parent map
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

  // Placeholders for Phase 2 (you can implement later)
  public Map<Localisation, Double> determinerChronologieDeLaCrue(long[] idsOrigin,
      double vWaterInit) {
    throw new UnsupportedOperationException("Phase 2 not implemented yet.");
  }


  public Deque<Localisation> trouverCheminDEvacuationLePlusCourt(long idOrigin,
      long idEvacuation,
      double vVehicule,
      Map<Localisation, Double> tFlood) {
    throw new UnsupportedOperationException("Phase 2 not implemented yet.");
  }
}
