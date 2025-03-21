package frc.robot.subsystems.elevatoreffector;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import static frc.robot.subsystems.elevatoreffector.ElevatorSubsystem2.ElevatorState;

/**
 * a graph class can find a path between two nodes (states).
 */
public class ElevatorStateMachine {
    private final Map<ElevatorState, Node> nodes = new HashMap<>();
    private final Map<ElevatorState, Set<ElevatorState>> edges = new HashMap<>();

    public ElevatorStateMachine() {
    }

    /**
     * Add a node to the graph
     *
     * @param name      state
     * @param positions distance angle pair
     */
    public void addNode(ElevatorState name, Pair<Distance, Angle> positions) {
        nodes.put(name, new Node(name, positions));
        edges.putIfAbsent(name, new HashSet<>());
    }

    public void addEdge(ElevatorState originNode, ElevatorState targetNode) {
        if (nodes.containsKey(originNode) && nodes.containsKey(targetNode)) {
            edges.get(originNode).add(targetNode);
            edges.get(targetNode).add(originNode); // Ensure the edge is bidirectional
        }
    }

    /**
     * Connect all nodes together such that any node can go to any other node
     *
     * @param states all nodes to be connected
     */
    public void addFullyConnectedEdges(ElevatorState... states) {
        for (int i = 0; i < states.length; i++) {
            for (int j = i + 1; j < states.length; j++) {
                addEdge(states[i], states[j]);
            }
        }
    }

    public Node findPath(ElevatorState originNode, ElevatorState targetNode) {
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

//        if(path.size() > 1) return new Pair<ElevatorState, Pair<Distance, Angle>>(path.get(1), nodes.get(path.get(1)).getCommands());
//        else return new Pair<ElevatorState, Pair<Distance, Angle>>(path.get(0), nodes.get(path.get(0)).getCommands());

        // Return the next node in the path if a path is found
        if (path.size() > 1) return nodes.get(path.get(1));
        // If no path just return itself
        else return nodes.get(path.get(0));
    }

    public Node getNode(ElevatorState name) {
        return nodes.get(name);
    }

    public static class Node {
        private final ElevatorState name;
        private Pair<Distance, Angle> commands;

        public Node(ElevatorState name, Pair<Distance, Angle> commands) {
            this.name = name;
            this.commands = commands;
        }

        public ElevatorState getName() {
            return name;
        }

        public Pair<Distance, Angle> getCommands() {
            return commands;
        }
    }
}