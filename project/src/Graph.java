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

  // Default epsilon (tolerance) for flood propagation
  private final double epsilon = 0.0;
  // Default k factor for water speed update
  private static final double K_DEFAULT = 0.05;

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
        adj.computeIfAbsent(source, k -> new ArrayList<>()).add(new Edge(target, dist, streetName));
      }
    } catch (IOException e) {
      throw new RuntimeException("Error loading edges from " + file, e);
    }
  }

  // -------------------------------------------------------------------------
  // Algorithme 1 : Simulation de la Crue (Phase 1 - static)
  // -------------------------------------------------------------------------

  /**
   * Determines the flooded zone via BFS. Water propagates from X to neighbour Y if Alt(Y) <= Alt(X)
   * + epsilon.
   *
   * @param idsOrigin starting flood node ids
   * @param epsilon   tolerance in metres
   * @return array of flooded Localisations in BFS visitation order
   */
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

  /**
   * Convenience overload using default epsilon.
   */
  public Localisation[] determinerZoneInondee(long[] idsOrigin) {
    return determinerZoneInondee(idsOrigin, this.epsilon);
  }

  // -------------------------------------------------------------------------
  // Algorithme 2 : Navigation de Secours (Phase 1 - static)
  // -------------------------------------------------------------------------

  /**
   * Finds the shortest path (in number of streets) from idOrigin to idDestination, avoiding all
   * nodes in floodedZone. Uses BFS on the unweighted graph.
   *
   * @param idOrigin      start node id
   * @param idDestination destination node id
   * @param floodedZone   nodes to avoid
   * @return Deque of Localisations from origin to destination (empty if no path)
   */
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

  // -------------------------------------------------------------------------
  // Algorithme 3 : Chronologie de la Crue (Phase 2 - temporal)
  // -------------------------------------------------------------------------

  /**
   * Computes the precise flood arrival time tFlood[node] for every reachable node. Uses Dijkstra:
   * edge weight = distance / vWater (travel time). Water speed updates as: vWater(p2) = vWater(p1)
   * + k * slope(p1->p2). Propagation stops on an arc when the resulting speed is <= 0.
   *
   * @param idsOrigin  starting flood node ids (flooded at t=0 with speed vWaterInit)
   * @param vWaterInit initial water speed (m/s)
   * @param k          acceleration factor (default 0.05)
   * @return Map from Localisation to flood time in seconds
   */
  public Map<Localisation, Double> determinerChronologieDeLaCrue(long[] idsOrigin,
      double vWaterInit, double k) {

    // dist[id] = earliest time water reaches node id
    Map<Long, Double> dist = new HashMap<>();
    // vAtNode[id] = water speed when reaching node id (used to propagate further)
    Map<Long, Double> vAtNode = new HashMap<>();

    // Priority queue: (time, nodeId, waterSpeedAtNode)
    // We store speed per entry because the same node might be reached with different speeds
    // but we only care about the fastest (minimum time) arrival.
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

      // Skip if we already found a faster route
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

        // Slope S = (Alt(current) - Alt(neigh)) / distance
        double slope = (current.getAltitude() - neigh.getAltitude()) / e.dist;
        // Water speed at neighbour
        double vNext = vCurrent + k * slope;

        // Water stops if speed is non-positive
        if (vNext <= 0) {
          continue;
        }

        // Travel time uses vNext (speed at destination node)
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

    // Build result map preserving insertion order (ordered by flood time)
    Map<Localisation, Double> result = new LinkedHashMap<>();
    dist.entrySet().stream().sorted(Map.Entry.comparingByValue()).forEach(entry -> {
      Localisation loc = nodes.get(entry.getKey());
      if (loc != null) {
        result.put(loc, entry.getValue());
      }
    });

    return result;
  }

  /**
   * Convenience overload using default k = 0.05. Matches the signature called by the provided test
   * files.
   */
  public Map<Localisation, Double> determinerChronologieDeLaCrue(long[] idsOrigin,
      double vWaterInit) {
    return determinerChronologieDeLaCrue(idsOrigin, vWaterInit, K_DEFAULT);
  }

  // -------------------------------------------------------------------------
  // Algorithme 4 : Évacuation Dynamique (Phase 2 - temporal)
  // -------------------------------------------------------------------------

  /**
   * Finds the fastest evacuation path from idDepart to idEvacuation, avoiding nodes that will be
   * flooded before the vehicle arrives. Uses Dijkstra: edge weight = distance / vVehicule. A node
   * neighbour is forbidden if t_arrival > tFlood[neighbour].
   *
   * @param idDepart     starting node id (vehicle departs at t=0)
   * @param idEvacuation destination node id
   * @param vVehicule    constant vehicle speed (m/s)
   * @param tFlood       flood chronology from Algorithm 3
   * @return Deque of Localisations representing the fastest safe path (empty if none)
   */
  public Deque<Localisation> trouverCheminDEvacuationLePlusCourt(long idDepart, long idEvacuation,
      double vVehicule, Map<Localisation, Double> tFlood) {

    // Build a fast lookup: node id -> flood time (Double.MAX_VALUE if not flooded)
    Map<Long, Double> floodTime = new HashMap<>();
    for (Map.Entry<Localisation, Double> entry : tFlood.entrySet()) {
      floodTime.put(entry.getKey().getId(), entry.getValue());
    }

    // Dijkstra: (time, nodeId)
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

        // Cannot use this node if it will be flooded before we arrive
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