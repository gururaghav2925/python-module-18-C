# Dijkstra's Single Source Shortest Path Algorithm using Python

# Aim
To implement Dijkstra's algorithm in Python to find the shortest path from a single source vertex to all other vertices in a given weighted and connected graph.

# Procedure
1. Understand that Dijkstra’s algorithm is used to find the shortest path from a source node to all other nodes in a weighted graph with non-negative weights.
2. Represent the graph using an adjacency list or matrix, where each node has a list of its neighbors and associated edge weights.
3. Initialize a priority queue (min-heap) to always expand the closest unvisited node.
4. Set the distance to the source node as 0 and to all other nodes as infinity.
5. While the priority queue is not empty:
   - Extract the node with the smallest tentative distance.
   - For each unvisited neighbor, calculate the new tentative distance through the current node.
   - If the new distance is smaller, update it in the distance table and insert the neighbor into the priority queue.
6. Repeat until all nodes have been visited and the shortest distances are finalized.
7. Display the shortest distances from the source node to all other nodes.
# Program
```python
# Python program for Dijkstra's single source shortest path algorithm. 
# The program is for adjacency matrix representation of the graph

# Library for INT_MAX
import sys

class Graph():

	def __init__(self, vertices):
		self.V = vertices
		self.graph = [[0 for column in range(vertices)]
					for row in range(vertices)]

	def printSolution(self, dist):
		print("Vertex   Distance from Source")
		for node in range(self.V):
			print(node, "           ", dist[node])

	# A utility function to find the vertex with
	# minimum distance value, from the set of vertices
	# not yet included in shortest path tree
	def minDistance(self, dist, sptSet):

		# Initialize minimum distance for next node
		min = sys.maxsize

		# Search not nearest vertex not in the
		# shortest path tree
		for u in range(self.V):
			if dist[u] < min and sptSet[u] == False:
				min = dist[u]
				min_index = u

		return min_index

	# Function that implements Dijkstra's single source
	# shortest path algorithm for a graph represented
	# using adjacency matrix representation
	def dijkstra(self, src):

		dist = [sys.maxsize] * self.V
		dist[src] = 0
		sptSet = [False] * self.V

		for cout in range(self.V):

			# Pick the minimum distance vertex from
			# the set of vertices not yet processed.
			# x is always equal to src in first iteration
			x = self.minDistance(dist, sptSet)

			# Put the minimum distance vertex in the
			# shortest path tree
			sptSet[x] = True

			# Update dist value of the adjacent vertices
			# of the picked vertex only if the current
			# distance is greater than new distance and
			# the vertex in not in the shortest path tree
			for y in range(self.V):
				if self.graph[x][y] > 0 and sptSet[y] == False and 				dist[y] > dist[x] + self.graph[x][y]:
						dist[y] = dist[x] + self.graph[x][y]

		self.printSolution(dist)

# Driver program
g = Graph(9)
g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
		[4, 0, 8, 0, 0, 0, 0, 11, 0],
		[0, 8, 0, 7, 0, 4, 0, 0, 2],
		[0, 0, 7, 0, 9, 14, 0, 0, 0],
		[0, 0, 0, 9, 0, 10, 0, 0, 0],
		[0, 0, 4, 14, 10, 0, 2, 0, 0],
		[0, 0, 0, 0, 0, 2, 0, 1, 6],
		[8, 11, 0, 0, 0, 0, 1, 0, 7],
		[0, 0, 2, 0, 0, 0, 6, 7, 0]
		];

g.dijkstra(0);
```

# Output

![image](https://github.com/user-attachments/assets/ad12f1fc-6582-4952-96ca-15c3e4d94a8c)


# Result
Dijkstra’s algorithm is successfully implemented using Python. The program computes and displays the shortest path distances from the given source vertex to all other vertices in the graph.
