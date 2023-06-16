//CÓDIGO JAVA DA AVALIACAO PARCIAL

import java.util.*;

public class Graph {

    private Map<Integer, Map<Integer, Integer>> adjacencyList;
    private Map<Integer, Object> vertexValues;
    private Map<List<Integer>, Object> edgeValues;

    public Graph() {
        adjacencyList = new HashMap<>();
        vertexValues = new HashMap<>();
        edgeValues = new HashMap<>();
    }

    public boolean adjacent(int x, int y) {
        Map<Integer, Integer> neighbors = adjacencyList.get(x);
        if (neighbors == null) {
            return false;
        }
        return neighbors.containsKey(y);
    }

    public List<Integer> neighbors(int x) {
        List<Integer> result = new ArrayList<>();
        Map<Integer, Integer> neighbors = adjacencyList.get(x);
        if (neighbors == null) {
            return result;
        }
        for (int neighbor : neighbors.keySet()) {
            result.add(neighbor);
        }
        return result;
    }

    public void addVertex(int x) {
        if (!adjacencyList.containsKey(x)) {
            adjacencyList.put(x, new HashMap<Integer, Integer>());
        }
    }

    public void removeVertex(int x) {
        adjacencyList.remove(x);
        vertexValues.remove(x);
        List<List<Integer>> toRemove = new ArrayList<>();
        for (List<Integer> edge : edgeValues.keySet()) {
            if (edge.contains(x)) {
                toRemove.add(edge);
            }
        }
        for (List<Integer> edge : toRemove) {
            edgeValues.remove(edge);
        }
    }

    public void addEdge(int x, int y, int z) {
        Map<Integer, Integer> neighbors = adjacencyList.get(x);
        if (neighbors == null) {
            neighbors = new HashMap<>();
            adjacencyList.put(x, neighbors);
        }
        neighbors.put(y, z);
        edgeValues.put(Arrays.asList(x, y), null);
    }

    public void removeEdge(int x, int y) {
        Map<Integer, Integer> neighbors = adjacencyList.get(x);
        if (neighbors != null) {
            neighbors.remove(y);
        }
        edgeValues.remove(Arrays.asList(x, y));
    }

    public Object getVertexValue(int x) {
        return vertexValues.get(x);
    }

    public void setVertexValue(int x, Object v) {
        vertexValues.put(x, v);
    }

    public Object getEdgeValue(int x, int y) {
        return edgeValues.get(Arrays.asList(x, y));
    }

    public void setEdgeValue(int x, int y, Object v) {
        edgeValues.put(Arrays.asList(x, y), v);
    }


    // caminho(a, b) - retorna o caminho, se existir, entre os vértices a e b.
    public List<Integer> caminho(int a, int b) {
        List<Integer> path = new ArrayList<Integer>();
        boolean[] visited = new boolean[numVertices];
        int[] prev = new int[numVertices];
        Queue<Integer> queue = new LinkedList<Integer>();
        queue.add(a);
        visited[a] = true;
        while (!queue.isEmpty()) {
            int current = queue.remove();
            if (current == b) {
                while (current != a) {
                    path.add(0, current);
                    current = prev[current];
                }
                path.add(0, a);
                return path;
            }
            for (int neighbor : getNeighbors(current)) {
                if (!visited[neighbor]) {
                    queue.add(neighbor);
                    visited[neighbor] = true;
                    prev[neighbor] = current;
                }
            }
        }
        return null;
    }

    // uniao(G1, G2) - retorna um grafo resultante da união entre os grafos G1 e G2.
    public Graph uniao(Graph G1, Graph G2) {
        Graph result = new Graph(G1.getNumVertices() + G2.getNumVertices());
        for (int i = 0; i < G1.getNumVertices(); i++) {
            result.addVertex(G1.getVertexValue(i));
            for (int neighbor : G1.getNeighbors(i)) {
                result.addEdge(i, neighbor, G1.getEdgeValue(i, neighbor));
            }
        }
        for (int i = 0; i < G2.getNumVertices(); i++) {
            if (!result.containsVertex(G2.getVertexValue(i))) {
                result.addVertex(G2.getVertexValue(i));
            }
            for (int neighbor : G2.getNeighbors(i)) {
                if (!result.containsEdge(i + G1.getNumVertices(), neighbor + G1.getNumVertices())) {
                    result.addEdge(i + G1.getNumVertices(), neighbor + G1.getNumVertices(), G2.getEdgeValue(i, neighbor));
                }
            }
        }
        return result;
    }

    // interseccao(G1, G2) - retorna um grafo resultante da intersecção entre os grafos G1 e G2.
    public Graph interseccao(Graph G1, Graph G2) {
        Graph result = new Graph(Math.min(G1.getNumVertices(), G2.getNumVertices()));
        for (int i = 0; i < G1.getNumVertices(); i++) {
            if (G2.containsVertex(G1.getVertexValue(i))) {
                result.addVertex(G1.getVertexValue(i));
                for (int neighbor : G1.getNeighbors(i)) {
                    if (G2.containsVertex(G1.getVertexValue(neighbor))) {
                        result.addEdge(i, neighbor, G1.getEdgeValue(i, neighbor));
                    }
                }
            }
        }
        return result;
    }


}


<--------------------------------------------------------------------------------------------------------->
Avaliação Final - Teoria da Computação



import java.util.*;

public class BreadthFirstSearch {

    public static class Graph {
        private Map<Integer, List<Integer>> adjacencyList;

        public Graph() {
            adjacencyList = new HashMap<>();
        }

	
	// Verifica se os vértices x e y são adjacentes no grafo.
        public boolean adjacent(int x, int y) {
            List<Integer> neighbors = adjacencyList.get(x);
            if (neighbors == null) {
                return false;
            } 
            return neighbors.contains(y);
        }

	// Retorna uma lista de vértices adjacentes ao vértice x.
        public List<Integer> neighbors(int x) {
            return adjacencyList.getOrDefault(x, new ArrayList<>());
        }

	// Adiciona um vértice x ao grafo.
        public void addVertex(int x) {
            adjacencyList.putIfAbsent(x, new ArrayList<>());
        }

	// Remove o vértice x do grafo, bem como suas arestas adjacentes.
        public void removeVertex(int x) {
            adjacencyList.remove(x);
            adjacencyList.values().forEach(neighbors -> neighbors.remove(Integer.valueOf(x)));
        }

	// Adiciona uma aresta entre os vértices x e y no grafo.
        public void addEdge(int x, int y) {
            adjacencyList.computeIfAbsent(x, k -> new ArrayList<>()).add(y);
            adjacencyList.computeIfAbsent(y, k -> new ArrayList<>()).add(x);
        }

	// Remove a aresta entre os vértices x e y no grafo.
        public void removeEdge(int x, int y) {
            List<Integer> xNeighbors = adjacencyList.get(x);
            List<Integer> yNeighbors = adjacencyList.get(y);
            if (xNeighbors != null) {
                xNeighbors.remove(Integer.valueOf(y));
            }
            if (yNeighbors != null) {
                yNeighbors.remove(Integer.valueOf(x));
            }
        }

	// Executa a busca em largura (BFS) no grafo a partir do vértice start
	// até o vértice end e retorna o caminho encontrado como uma lista de vértices.
        public List<Integer> bfs(int start, int end) {
            List<Integer> path = new ArrayList<>();
            Map<Integer, Integer> prev = new HashMap<>();
            Queue<Integer> queue = new LinkedList<>();
            Set<Integer> visited = new HashSet<>();

            queue.add(start);
            visited.add(start);

            while (!queue.isEmpty()) {
                int current = queue.poll();

                if (current == end) {
                    // Reconstruct the path
                    int node = current;
                    while (node != start) {
                        path.add(0, node);
                        node = prev.get(node);
                    }
                    path.add(0, start);
                    return path;
                }

                List<Integer> neighbors = adjacencyList.getOrDefault(current, new ArrayList<>());
                for (int neighbor : neighbors) {
                    if (!visited.contains(neighbor)) {
                        queue.add(neighbor);
                        visited.add(neighbor);
                        prev.put(neighbor, current);
                    }
                }
            }

            return path; // No path found
        }
    }

    public static void main(String[] args) {
        Graph graph = new Graph();

        // Adiciona vértices ao grafo
        graph.addVertex(0);
        graph.addVertex(1);
        graph.addVertex(2);
        graph.addVertex(3);

        // Adiciona arestas ao grafo
        graph.addEdge(0, 1);
        graph.addEdge(1, 2);
        graph.addEdge(2, 3);

        // Executa o BFS e obtém o caminho do vértice 0 to vertex 3
        List<Integer> path = graph.bfs(0, 3);

        // Imprime o caminho
        System.out.println("Path: " + path);
    }
}