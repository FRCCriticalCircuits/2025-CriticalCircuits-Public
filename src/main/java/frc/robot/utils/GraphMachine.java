package frc.robot.utils;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import edu.wpi.first.math.Pair;

/**
 * a graph class can find a path between two nodes (states).
 */
public class GraphMachine {
    private final Map<String, Node> nodes = new HashMap<>();
    private final Map<String, Set<String>> edges = new HashMap<>();

    public GraphMachine() {}

    public void addNode(String name, Pair<Double, Double> commands) {
        nodes.put(name, new Node(name, commands));
        edges.putIfAbsent(name, new HashSet<>());
    }

    public void addEdge(String originNode, String targetNode) {
        if (nodes.containsKey(originNode) && nodes.containsKey(targetNode)) {
            edges.get(originNode).add(targetNode);
            edges.get(targetNode).add(originNode); // Ensure the edge is bidirectional
        }
    }

    public Pair<String, Pair<Double, Double>> findPath(String originNode, String targetNode) {
        if (!nodes.containsKey(originNode) || !nodes.containsKey(targetNode)) {
            throw new IllegalArgumentException("Node does not exist: " + originNode + "," + targetNode);
        }

        Map<String, String> predecessors = new HashMap<>();
        Queue<String> queue = new LinkedList<>();
        Set<String> visited = new HashSet<>();

        queue.add(originNode);
        visited.add(originNode);
        predecessors.put(originNode, null);

        while (!queue.isEmpty()) {
            String current = queue.poll();
            if (current.equals(targetNode)) break;

            for (String neighbor : edges.getOrDefault(current, Collections.emptySet())) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    predecessors.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        if (!predecessors.containsKey(targetNode)) {
            throw new IllegalArgumentException("No path found, check your edges");
        }

        LinkedList<String> path = new LinkedList<>();
        String current = targetNode;
        while (current != null) {
            path.addFirst(current);
            current = predecessors.get(current);
        }

        if(path.size() > 1) return new Pair<String, Pair<Double, Double>>(path.get(1), nodes.get(path.get(1)).getCommands());
        else return new Pair<String, Pair<Double, Double>>(path.get(0), nodes.get(path.get(0)).getCommands());
    }

    public Node getNode(String name) {
        return nodes.get(name);
    }

    public static class Node {
        private final String name;
        private Pair<Double, Double> commands;

        public Node(String name, Pair<Double, Double> commands) {
            this.name = name;
            this.commands = commands;
        }

        public String getName() {
            return name;
        }

        public Pair<Double, Double> getCommands() {
            return commands;
        }
    }
}