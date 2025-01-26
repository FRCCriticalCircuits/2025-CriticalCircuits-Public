package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;

public class GraphMachine {
    private Map<String, Node> nodeMap;

    public GraphMachine() {
        nodeMap = new HashMap<>();
    }

    public void addNode(String nodeName, List<Command> commandGroup) {
        if (!nodeMap.containsKey(nodeName)) {
            Node newNode = new Node(nodeName, commandGroup);
            nodeMap.put(nodeName, newNode);
        } else {
            System.out.println("Node '" + nodeName + "' already exists.");
        }
    }

    public void editNode(String nodeName, List<Command> newCommandGroup) {
        Node node = nodeMap.get(nodeName);
        if (node != null) {
            node.setCommands(newCommandGroup);
        } else {
            System.out.println("Node '" + nodeName + "' doesnt exist.");
        }
    }

    public void addEdge(String fromNodeName, String toNodeName) {
        addEdge(fromNodeName, toNodeName, 1);
    }

    public void addEdge(String fromNodeName, String toNodeName, int cost) {
        Node fromNode = nodeMap.get(fromNodeName);
        Node toNode = nodeMap.get(toNodeName);

        if (fromNode != null && toNode != null) {
            fromNode.addNeighbor(toNode, cost);
        } else {
            System.out.println("Node doesn't exist: '" + fromNodeName + "' or '" + toNodeName + "'.");
        }
    }

    private class Node {
        private String name;
        private List<Command> commands;
        private Map<Node, Integer> neighbors;

        public Node(String name, List<Command> commands) {
            this.name = name;
            this.commands = commands;
            this.neighbors = new HashMap<>();
        }

        public void setCommands(List<Command> commands) {
            this.commands = commands;
        }

        public void addNeighbor(Node neighbor, int cost) {
            neighbors.put(neighbor, cost);
        }
    }
}
