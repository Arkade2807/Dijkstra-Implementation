// --== CS400 File Header Information ==--
// Name: <your full name>
// Email: <your @wisc.edu email address>
// Group and Team: <your group name: two letters, and team color>
// Group TA: <name of your group's ta>
// Lecturer: <name of your lecturer>
// Notes to Grader: <optional extra notes>

import java.util.PriorityQueue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import java.util.Hashtable;
import java.util.List;
import java.util.LinkedList;
import java.util.NoSuchElementException;

/**
 * This class extends the BaseGraph data structure with additional methods for computing the total
 * cost and list of node data along the shortest path connecting a provided starting to ending
 * nodes. This class makes use of Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number> extends BaseGraph<NodeType, EdgeType>
    implements GraphADT<NodeType, EdgeType> {

  /**
   * While searching for the shortest path between two nodes, a SearchNode contains data about one
   * specific path between the start node and another node in the graph. The final node in this path
   * is stored in it's node field. The total cost of this path is stored in its cost field. And the
   * predecessor SearchNode within this path is referened by the predecessor field (this field is
   * null within the SearchNode containing the starting node in it's node field).
   *
   * SearchNodes are Comparable and are sorted by cost so that the lowest cost SearchNode has the
   * highest priority within a java.util.PriorityQueue.
   */
  protected class SearchNode implements Comparable<SearchNode> {
    public Node node;
    public double cost;
    public SearchNode predecessor;

    public SearchNode(Node node, double cost, SearchNode predecessor) {
      this.node = node;
      this.cost = cost;
      this.predecessor = predecessor;
    }

    public int compareTo(SearchNode other) {
      if (cost > other.cost)
        return +1;
      if (cost < other.cost)
        return -1;
      return 0;
    }
  }

  /**
   * This helper method creates a network of SearchNodes while computing the shortest path between
   * the provided start and end locations. The SearchNode that is returned by this method is
   * represents the end of the shortest path that is found: it's cost is the cost of that shortest
   * path, and the nodes linked together through predecessor references represent all of the nodes
   * along that shortest path (ordered from end to start).
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return SearchNode for the final end node within the shortest path
   * @throws NoSuchElementException when no path from start to end is found or when either start or
   *                                end data do not correspond to a graph node
   */
  protected SearchNode computeShortestPath(NodeType start, NodeType end) {
    // TODO: implement in step 6
    if (start == null || end == null) { // seeing if element exists
      throw new NoSuchElementException("Start or End doesnt exsist");
    }
    if (!this.nodes.containsKey(end) || !this.nodes.containsKey(start)) {
      throw new NoSuchElementException("Start or End doesnt exsist");
    }
    Node startNode = this.nodes.get(start); // getting the Node from hash
    Node endNode = this.nodes.get(end);
    SearchNode startPoint = new SearchNode(startNode, 0, null);
    PriorityQueue<SearchNode> PQ = new PriorityQueue<>(); // priority queue to track all nodes
    Hashtable<NodeType, SearchNode> visitedNodes = new Hashtable();// hashtable to track visited
                                                                   // nodes
    PQ.add(startPoint);
    while (!PQ.isEmpty()) { // while we havent checked all nodes

      SearchNode presentNode = PQ.poll();
      // if(presentNode.node.equals(end)) { // if we reached the end
      // return presentNode;
      // }
      if (!visitedNodes.containsKey(presentNode.node.data)) { // if node is not visited
        visitedNodes.put(presentNode.node.data, presentNode);
        Node present = this.nodes.get(presentNode.node.data);
        for (int i = 0; i < present.edgesLeaving.size(); i++) {
          SearchNode PresentNode = new SearchNode(present.edgesLeaving.get(i).successor,
              present.edgesLeaving.get(i).data.doubleValue() + presentNode.cost, presentNode); // adding
                                                                                               // all
          // connecting nodes to PQ
          PQ.add(PresentNode);

        }
      } else {
                  ////////// if visited checking size form start////////////
        SearchNode pres = visitedNodes.get(presentNode.node.data);
        SearchNode presentNode1 = presentNode;
        double prescost = pres.cost;
        double newcost = presentNode1.cost;
                          /////////////////// if less replace///////
        if (newcost < prescost) { 

          visitedNodes.remove(presentNode.node.data);
          visitedNodes.put(presentNode.node.data, presentNode);

          //
        }
      }
    }
    /// if reached the end and no end node was added throw exception
    if (!visitedNodes.containsKey(end)) {
      throw new NoSuchElementException(visitedNodes.toString() + end.toString());
    }
    return visitedNodes.get(end);
  }

  /**
   * Returns the list of data values from nodes along the shortest path from the node with the
   * provided start value through the node with the provided end value. This list of data values
   * starts with the start value, ends with the end value, and contains intermediary values in the
   * order they are encountered while traversing this shorteset path. This method uses Dijkstra's
   * shortest path algorithm to find this solution.
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return list of data item from node along this shortest path
   */
  public List<NodeType> shortestPathData(NodeType start, NodeType end) {
    // TODO: implement in step 7

    SearchNode present = computeShortestPath(start, end);
    List<NodeType> traverse = new LinkedList<>();
    while (present.node.data != start) {
      traverse.add(0, present.node.data);//traverses from end and adds all elements to 0th element
      // in array till we reach start
      present = present.predecessor;
    }
    traverse.add(0, start); // add start
    return traverse;

  }

  /**
   * Returns the cost of the path (sum over edge weights) of the shortest path freom the node
   * containing the start data to the node containing the end data. This method uses Dijkstra's
   * shortest path algorithm to find this solution.
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return the cost of the shortest path between these nodes
   */
  public double shortestPathCost(NodeType start, NodeType end) {
    SearchNode present = computeShortestPath(start, end);

    return present.cost; // total cost is in the end search node
  }


  @Test
  /**
   * Tests the path traversed in class by hand that is A-> E
   */
  public void testShortestPathData() {
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();

    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("C");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertEdge("A", "B", 4);
    graph.insertEdge("A", "C", 2);
    graph.insertEdge("B", "E", 10);
    graph.insertEdge("B", "D", 1);
    graph.insertEdge("C", "D", 5);
    graph.insertEdge("D", "E", 3);
    graph.insertEdge("D", "F", 0);
    graph.insertEdge("F", "D", 2);
    graph.insertEdge("F", "H", 4);
    graph.insertEdge("G", "H", 4);
    
    LinkedList<String> expectedPath = new LinkedList<>();
    expectedPath.add("A");
    expectedPath.add("B");
    expectedPath.add("D");
    expectedPath.add("E");
    double expectedCost = 8.0;
    double actualCost = graph.shortestPathCost("A", "E");
    assertEquals(expectedCost, actualCost);
    List<String> actualPath = graph.shortestPathData("A", "E");
    assertEquals(expectedPath, actualPath);
  }

  @Test
  /**
   * This is a route not traveresed in class and we check the path it takes and the cost
   */
  public void testShortestPathCost() {
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();

    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("C");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertEdge("A", "B", 4);
    graph.insertEdge("A", "C", 2);
    graph.insertEdge("B", "E", 10);
    graph.insertEdge("B", "D", 1);
    graph.insertEdge("C", "D", 5);
    graph.insertEdge("D", "E", 3);
    graph.insertEdge("D", "F", 0);
    graph.insertEdge("F", "D", 2);
    graph.insertEdge("F", "H", 4);
    graph.insertEdge("G", "H", 4);
    
      double ExpectedCost = 4.0;
      double ActualCost = graph.shortestPathCost("B", "E");
      assertEquals(ExpectedCost, ActualCost);
  }

  @Test
  /*
   * this is  a path with the nodes in the graph but there is no possible path to traverse
   */
  public void testNoSuchElementException() {
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>();

    // Add nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("C");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");

    // Add edges with weights
    graph.insertEdge("A", "B", 4);
    graph.insertEdge("A", "C", 2);
    graph.insertEdge("B", "E", 10);
    graph.insertEdge("B", "D", 1);
    graph.insertEdge("C", "D", 5);
    graph.insertEdge("D", "E", 3);
    graph.insertEdge("D", "F", 0);
    graph.insertEdge("F", "D", 2);
    graph.insertEdge("F", "H", 4);
    graph.insertEdge("G", "H", 4);
    
      assertThrows(NoSuchElementException.class, () -> graph.shortestPathData("F", "G"));
  }
  // TODO: implement 3+ tests in step 8.
 

}
