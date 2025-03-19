package frc.robot.subsystems.elevatoreffector;

import edu.wpi.first.math.Pair;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import static frc.robot.subsystems.elevatoreffector.ElevatorEffectorSubsystem.ElevatorState.*;
import static frc.robot.subsystems.elevatoreffector.ElevatorEffectorSubsystem.ElevatorState;
/**
 * a graph class can find a path between two nodes (states).
 */
public class ElevatorStateMachine {
    private final Map<ElevatorState, Node> nodes = new HashMap<>();
    private final Map<ElevatorState, Set<ElevatorState>> edges = new HashMap<>();

    public ElevatorStateMachine() {}

    public void addNode(ElevatorState name, Pair<Double, Double> commands) {
        nodes.put(name, new Node(name, commands));
        edges.putIfAbsent(name, new HashSet<>());
    }

    public void addEdge(ElevatorState originNode, ElevatorState targetNode) {
        if (nodes.containsKey(originNode) && nodes.containsKey(targetNode)) {
            edges.get(originNode).add(targetNode);
            edges.get(targetNode).add(originNode); // Ensure the edge is bidirectional
        }
    }

    public Pair<ElevatorState, Pair<Double, Double>> findPath(ElevatorState originNode, ElevatorState targetNode) {
        if (!nodes.containsKey(originNode) || !nodes.containsKey(targetNode)) {
            throw new IllegalArgumentException("Node does not exist: " + originNode + "," + targetNode);
        }

        Map<ElevatorState, ElevatorState> predecessors = new HashMap<>();
        Queue<ElevatorState> queue = new LinkedList<>();
        Set<ElevatorState> visited = new HashSet<>();

        queue.add(originNode);
        visited.add(originNode);
        predecessors.put(originNode, null);

        while (!queue.isEmpty()) {
            ElevatorState current = queue.poll();
            if (current.equals(targetNode)) break;

            for (ElevatorState neighbor : edges.getOrDefault(current, Collections.emptySet())) {
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

        LinkedList<ElevatorState> path = new LinkedList<>();
        ElevatorState current = targetNode;
        while (current != null) {
            path.addFirst(current);
            current = predecessors.get(current);
        }

        if(path.size() > 1) return new Pair<ElevatorState, Pair<Double, Double>>(path.get(1), nodes.get(path.get(1)).getCommands());
        else return new Pair<ElevatorState, Pair<Double, Double>>(path.get(0), nodes.get(path.get(0)).getCommands());
    }

    public Node getNode(ElevatorState name) {
        return nodes.get(name);
    }

    public static class Node {
        private final ElevatorState name;
        private Pair<Double, Double> commands;

        public Node(ElevatorState name, Pair<Double, Double> commands) {
            this.name = name;
            this.commands = commands;
        }

        public ElevatorState getName() {
            return name;
        }

        public Pair<Double, Double> getCommands() {
            return commands;
        }
    }
}