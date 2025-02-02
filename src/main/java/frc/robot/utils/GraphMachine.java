package frc.robot.utils;

import java.util.*;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Adjacent List Based State Machine with pathfinding capabilities
 */
public class GraphMachine {
    private final Map<String, Node> nodes = new HashMap<>();
    private final Map<String, Set<String>> edges = new HashMap<>();

    public GraphMachine() {}

    public void addNode(String name, Command commands, Subsystem... requirements) {
        Command commandToAdd = commands;
        commandToAdd.addRequirements(requirements);
        nodes.put(name, new Node(name, commands));
        edges.putIfAbsent(name, new HashSet<>());
    }

    public void addEdge(String originNode, String targetNode) {
        if (nodes.containsKey(originNode) && nodes.containsKey(targetNode)) {
            edges.get(originNode).add(targetNode);
            edges.get(targetNode).add(originNode);
        }
    }

    public List<Command> findPath(String originNode, String targetNode) {
        if (!nodes.containsKey(originNode) || !nodes.containsKey(targetNode)) {
            return Collections.emptyList();
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
            return Collections.emptyList();
        }

        LinkedList<String> path = new LinkedList<>();
        String current = targetNode;
        while (current != null) {
            path.addFirst(current);
            current = predecessors.get(current);
        }

        return path.stream()
                .map(nodeName -> nodes.get(nodeName).getCommands())
                .collect(Collectors.toList());
    }

    public Node getNode(String name) {
        return nodes.get(name);
    }

    public static class Node {
        private final String name;
        private Command commands;

        public Node(String name, Command commands) {
            this.name = name;
            this.commands = commands;
        }

        public void editCommand(Command commands) {
            this.commands = commands;
        }

        public String getName() {
            return name;
        }

        public Command getCommands() {
            return commands;
        }
    }
}