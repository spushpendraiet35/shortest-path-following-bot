/*
*Team Id: 1503
*Author List: Pushpendra Singh
*Filename: NS_Task_1_Sandbox.cpp
*Theme: Nutty_Squirrel – eYRC Specific
*Functions: Task_1_2(void), left_turn_wls(void), right_turn_wls(void), line_following (void), distance(void), movement(int, int ,int), colour_detection(void), 
  minDistance(int,bool), printpath(int,int), printSolution(int, int, int, int, int), dijkstra(int, int, int)
*Global Variables: x, y, z, src, t, g, colour, k, kk, nuts_1[6], red[2], green[2], vector<int> path, graph[V][V]
*Variables: count, left_white_line, center_white_line, right_white_line, min, min_index, v, V, dist[V], sptSet[V], parent[V], flag, flag1, a, b, c, proximity_sensor,
  red_pulse_count, green_pulse_count, filter_clear_pulse_count
*
*/
#include "NS_Task_1_Sandbox.h"
int THRESHOLD = 255;                // reading on black line
#define V 34                        // no. of nodes, including pick up and placing points
int x = 0;                          // to initialise nuts_1[] array
int y = 0;                          // to initialise red[] array
int z = 0;                          // to initialise green[] array
int src = 0;                        // initial position
int t = 0;                          // change values of src
int g;                              // change values of path
string colour;                      // stores the colour of nuts
int k = 0;                          // loop variable
int kk;                             // stores path of running array
int nuts_1[6] = { 3,2,1,4,5,6 };    // stores position at which nuts might be placed
int red[2] = { 7,8 };               // position where red nuts are to be placed
int green[2] = { 9,10 };            // position where green nuts are to be placed
vector<int> path;                   // stores path in vector form
int line_following(void);
int distance(void);
int movement(int a, int b, int c);
//representation of given map in matrix form
int graph[V][V] = {

						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
						{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 8, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0},
						{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 2, 0, 1, 0, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0},
						{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0},
						{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};


// A utility function to find the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree 
int minDistance(int dist[],
	bool sptSet[])
{

	// Initialize min value 
	int min = INT_MAX, min_index;

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false &&
			dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

// Function to print shortest path from source to j using parent array 
void printPath(int parent[], int j)
{

	// Base Case : If j is source 
	if (parent[j] == -1)
		return;

	printPath(parent, parent[j]);
	//stores the path value in path vector

	printf("%d ", j);
	path.push_back(j);
}

// A utility function to print the constructed distance array 
int printSolution(int dist[], int n, int src,
	int parent[], int dest)
{

	//print the source, destination, distance and the path
	printf("Vertex\t Distance\tPath");
	printf("\n%d -> %d \t\t %d\t\t%d ",
		src, dest, dist[dest], src);
	printPath(parent, dest);
	return 0;
}

// Funtion that implements Dijkstra's shortest path algorithm for a graph represented using adjacency matrix representation 
void dijkstra(int graph[V][V], int src, int dest)
{

	// The output array. dist[i] will hold the shortest distance from src to i 
	int dist[V];

	// sptSet[i] will true if vertex i is included in shortest path tree or shortest distance from src to i is finalized 
	bool sptSet[V];

	// Parent array to store shortest path tree 
	int parent[V];

	// Initialize all distances as INFINITE and stpSet[] as false 
	for (int i = 0; i < V; i++)
	{
		parent[src] = -1;
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}

	// Distance of source vertex from itself is always 0 
	dist[src] = 0;

	// Find shortest path for all vertices 
	for (int count = 0; count < V - 1; count++)
	{
		// Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src in first iteration. 
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex as processed 
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the picked vertex. 
		for (int v = 0; v < V; v++)

			// Update dist[v] only if is not in sptSet, there is an edge from u to v, and total weight of path from src to v through u is smaller than current value of dist[v] 
			if (!sptSet[v] && graph[u][v] &&
				dist[u] + graph[u][v] < dist[v])
			{
				parent[v] = u;
				dist[v] = dist[u] + graph[u][v];
			}
	}

	// print the constructed distance array 
	printSolution(dist, V, src, parent, dest);
}

/*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*/
void left_turn_wls(void)
{
	left();
	_delay_ms(200);
	while (ADC_Conversion(2) < 255);
	stop();
}

/*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{
	right();
	_delay_ms(200);
	while (ADC_Conversion(2) < 255);
	stop();
}


/*
* Function Name: movement
* Input: int a, int b, int c
* Output: int flag1
* Logic: Uses the input a, b, c to detect that from where the object is coming, where it is and where to go and makes the movement according to the conditions
* Example Call: movement(0,27,28)  // move forward then take right turn and follow the line
*/

int movement(int a, int b, int c)
{
	int flag1 = 0;
	if ((a == 0 && b == 27 && c == 28) || (a == 11 && b == 12 && c == 19) || (a == 12 && b == 19 && c == 18) || (a == 13 && b == 17 && c == 27) || (a == 14 && b == 15 && c == 20) ||
		(a == 15 && b == 16 && c == 21) || (a == 16 && b == 21 && c == 20) || (a == 17 && b == 27 && c == 26) || (a == 18 && b == 11 && c == 12) || (a == 18 && b == 19 && c == 26) ||
		(a == 19 && b == 12 && c == 13) || (a == 19 && b == 18 && c == 11) || (a == 19 && b == 26 && c == 25) || (a == 20 && b == 15 && c == 16) || (a == 20 && b == 21 && c == 31) ||
		(a == 20 && b == 28 && c == 27) || (a == 21 && b == 31 && c == 30) || (a == 21 && b == 20 && c == 15) || (a == 22 && b == 23 && c == 24) || (a == 23 && b == 18 && c == 19) ||
		(a == 24 && b == 25 && c == 29) || (a == 24 && b == 23 && c == 18) || (a == 25 && b == 29 && c == 30) || (a == 27 && b == 17 && c == 14) || (a == 27 && b == 26 && c == 19) ||
		(a == 28 && b == 20 && c == 21) || (a == 28 && b == 27 && c == 17) || (a == 29 && b == 28 && c == 20) || (a == 30 && b == 31 && c == 32) || (a == 30 && b == 29 && c == 28)|| (a == 26 && b == 25 && c == 24)
		|| (a == 2 && b == 12 && c == 11) || (a == 3 && b == 13 && c == 12) || (a == 4 && b == 14 && c == 17) || (a == 5 && b == 15 && c == 14) ||
		(a == 6 && b == 16 && c == 15) || (a == 9 && b == 30 && c == 31) || (a == 8 && b == 24 && c == 25)|| (a == 26 && b == 27 && c == 0))
	{
		forward();
		_delay_ms(250);
		right_turn_wls();
		flag1=line_following();
	}
	if ((a == 0 && b == 0 && c == 27) || (a == 0 && b == 27 && c == 17) || (a == 11 && b == 12 && c == 13) || (a == 12 && b == 13 && c == 17) || (a == 11 && b == 18 && c == 23) ||
		(a == 12 && b == 19 && c == 26) || (a == 13 && b == 17 && c == 14) || (a == 13 && b == 12 && c == 11) || (a == 14 && b == 17 && c == 13) || (a == 14 && b == 15 && c == 16) ||
		(a == 15 && b == 14 && c == 17) || (a == 15 && b == 20 && c == 28) || (a == 16 && b == 21 && c == 31) || (a == 17 && b == 13 && c == 12) || (a == 17 && b == 14 && c == 15) ||
		(a == 18 && b == 23 && c == 22) || (a == 21 && b == 31 && c == 32) || (a == 22 && b == 23 && c == 18) || (a == 23 && b == 24 && c == 25) || (a == 23 && b == 18 && c == 11) ||
		(a == 25 && b == 26 && c == 27) || (a == 25 && b == 24 && c == 23) || (a == 25 && b == 29 && c == 28) || (a == 26 && b == 27 && c == 28) || (a == 26 && b == 19 && c == 12) ||
		(a == 26 && b == 25 && c == 29) || (a == 27 && b == 26 && c == 25) || (a == 27 && b == 28 && c == 29) || (a == 28 && b == 27 && c == 26) || (a == 28 && b == 29 && c == 25) ||
		(a == 28 && b == 20 && c == 15) || (a == 29 && b == 30 && c == 31) || (a == 29 && b == 28 && c == 27) || (a == 29 && b == 25 && c == 26) || (a == 31 && b == 30 && c == 29) ||
		(a == 31 && b == 21 && c == 16) || (a == 32 && b == 31 && c == 21)|| (a == 16 && b == 15 && c == 14)|| (a == 1 && b == 11 && c == 18) || (a == 2 && b == 12 && c == 19) || 
		(a == 5 && b == 15 && c == 20) || (a == 6 && b == 16 && c == 21) ||(a == 7 && b == 22 && c == 23) || (a == 10 && b == 32 && c == 31)|| (a == 17 && b == 27 && c == 0))
	{
		forward();
		_delay_ms(250);
		flag1=line_following();
	}
	if ((a == 0 && b == 27 && c == 26) || (a == 11 && b == 18 && c == 19) || (a == 12 && b == 11 && c == 18) || (a == 13 && b == 12 && c == 19) || (a == 14 && b == 17 && c == 27) ||
		(a == 15 && b == 20 && c == 21) || (a == 16 && b == 15 && c == 20) || (a == 17 && b == 27 && c == 28) || (a == 18 && b == 19 && c == 12) || (a == 18 && b == 23 && c == 24) ||
		(a == 19 && b == 12 && c == 11) || (a == 19 && b == 18 && c == 23) || (a == 19 && b == 26 && c == 27) || (a == 20 && b == 15 && c == 14) || (a == 20 && b == 21 && c == 16) ||
		(a == 20 && b == 28 && c == 29) || (a == 21 && b == 16 && c == 15) || (a == 21 && b == 20 && c == 28) || (a == 24 && b == 25 && c == 26) || (a == 25 && b == 26 && c == 19) ||
		(a == 26 && b == 19 && c == 18) || (a == 26 && b == 27 && c == 17) || (a == 27 && b == 17 && c == 13) || (a == 27 && b == 28 && c == 20) || (a == 28 && b == 29 && c == 30) ||
		(a == 29 && b == 25 && c == 24) || (a == 30 && b == 29 && c == 25) || (a == 30 && b == 31 && c == 21) || (a == 31 && b == 21 && c == 20) || (a == 32 && b == 31 && c == 30)|| (a == 24 && b == 23 && c == 22)||
		(a == 1 && b == 11 && c == 12) || (a == 2 && b == 12 && c == 13) || (a == 3 && b == 13 && c == 17) || (a == 4 && b == 14 && c == 15) || (a == 5 && b == 15 && c == 16) || (a == 28 && b == 27 && c == 0)||
		(a == 8 && b == 24 && c == 23) || (a == 9 && b == 30 && c == 29)|| (a == 13 && b == 3 && c == 13) || (a == 12 && b == 2 && c == 12) || (a == 11 && b == 1 && c == 11) || (a == 14 && b == 4 && c == 14) || (a == 15 && b == 5 && c == 15) ||
		(a == 16 && b == 6 && c == 16) || (a == 22 && b == 7 && c == 22) || (a == 24 && b == 8 && c == 24) || (a == 30 && b == 9 && c == 30) || (a == 32 && b == 10 && c == 32))
	{
		forward();
		_delay_ms(250);
		left_turn_wls();
		flag1=line_following();
	}
	if (a == 27 && b == 0 && c == 33)
	{
		forward();
		_delay_ms(250);
		flag1=line_following();
		stop();
		_delay_ms(10000);
	}
	if ((a == 18 && b == 11 && c == 1) || (a == 19 && b == 12 && c == 2) || (a == 20 && b == 15 && c == 5) || (a == 21 && b == 16 && c == 6)|| (a == 23 && b == 22 && c == 7) || (a == 31 && b == 32 && c == 10))
	{
		forward();
		_delay_ms(250);
	}
	if ((a == 12 && b == 11 && c == 1) || (a == 23 && b == 24 && c == 8)|| (a == 13 && b == 12 && c == 2) || (a == 17 && b == 13 && c == 3) || (a == 15 && b == 14 && c == 4) || (a == 16 && b == 15 && c == 5) || (a == 29 && b == 30 && c == 9))
	{
		forward();
		_delay_ms(250);
		right_turn_wls();
	}
	if ((a == 11 && b == 12 && c == 2) || (a == 25 && b == 24 && c == 8)|| (a == 12 && b == 13 && c == 3) || (a == 17 && b == 14 && c == 4) || (a == 14 && b == 15 && c == 5) || (a == 15 && b == 16 && c == 6) || (a == 31 && b == 30 && c == 9))
	{
		forward();
		_delay_ms(250);
		left_turn_wls();
	}
	return flag1;
}

/*
* Function Name: colour_detection
* Input: void
* Output: string(colour)
* Logic: Uses ADC_Conversion(FRONT_IR_ADC_CHANNEL) to detect the colour of nut and returns the colour
* Example Call: colour_detection()  // "red"
*/

string colour_detection(void)
{
	unsigned int red_pulse_count, green_pulse_count, filter_clear_pulse_count;
	int proximity_sensor = ADC_Conversion(FRONT_IR_ADC_CHANNEL);
	filter_red();
	red_pulse_count = color_sensor_pulse_count;                         // stores the value of red sensor
	filter_green();
	green_pulse_count = color_sensor_pulse_count;                       // stores the value of green sensor
	filter_clear();
	filter_clear_pulse_count = color_sensor_pulse_count;                // clears sensor value
	if (red_pulse_count > green_pulse_count && red_pulse_count > 3000 && proximity_sensor < 100 && red_pulse_count < 4700)
	{
		cout << red_pulse_count << "red" << endl;
		colour = "red";
		return colour;
	}
	if (green_pulse_count > red_pulse_count && green_pulse_count >3000 && proximity_sensor < 100 && green_pulse_count < 4700)
	{
		cout << green_pulse_count << "green" << endl;
		colour = "green";
		return colour;
	}
	else
	{
		cout << "empty" << endl;
		colour = "empty";
		return colour;
	}
	return colour;
}


/*
* Function Name : Task_1_2()
* Input : void
* Output : void
* Logic : Uses white line sensors, proximity sensor and colour sensors to detect the obstacles, nuts and path.Uses dijkstra algorithm to find the shortest
  path and use it to pick and place the obstacle at the correct position and return back to the initial position in minimum possible time.
* Example Call : Task_1_2() //pick and place the nuts at the correct position and return to the starting point.
*/

void Task_1_2(void)
{
	int i = 0;
	line_following();
	while (i < 4)
	{
		int flag = 0;
		//call dijkstra from starting point to first position of pick up point
		dijkstra(graph, src, nuts_1[x]);
		// move from starting point to first pick up position
		for (int k = 0; k < path.size(); k++)
		{
			kk = nuts_1[x];
			g = path[k];
			flag=movement(t, src, path[k]);
			if (flag == 1)
				break;
			t = src;
			src = path[k];
		}
		path.clear();
		forward();
		_delay_ms(100);
		colour_detection();
		if (colour == "red")
		{
			pick();
			forward();
			_delay_ms(150);
			//call dijkstra from red nut to red nut placing position point
			dijkstra(graph, src, red[y]);
			// move from red pick up point to red nut placing point
			for (int k = 0; k < path.size(); k++)
			{
				kk = red[y];
				g = path[k];
				flag = movement(t, src, path[k]);
				if (flag == 1)
					break;
				t = src;
				src = path[k];
			}

			path.clear();
			place();
			forward();
			_delay_ms(300);
			x += 1;
			y += 1;
			i += 1;
		}
		if (colour == "green")
		{
			pick();
			forward();
			_delay_ms(150);
			//call dijkstra from green nut to green nut placing position point
			dijkstra(graph, src, green[z]);
			// move from green pick up point to green placing point
			for (int k = 0; k < path.size(); k++)
			{
				kk = green[z];
				g = path[k];
				flag = movement(t, src, path[k]);
				if (flag == 1)
					break;
				t = src;
				src = path[k];
			}
			place();
			forward();
			_delay_ms(300);
			path.clear();
			x += 1;
			z += 1;
			i += 1;
		}
		// if there is no nut then move to next pick up point
		if (colour == "empty")
		{
			forward();
			_delay_ms(150);
			x += 1;
		}
	}
	// from last placed nut to the initial position
	dijkstra(graph, src, 33);
	// move back to initial position
	for (int k = 0; k < path.size(); k++)
	{
		movement(t, src, path[k]);
		t = src;
		src = path[k];
	}
}



int line_following(void)
{
	int flag = 0;
	while (1)
	{
		int left_white_line = ADC_Conversion(1);            // value of left white sensor
		int center_white_line = ADC_Conversion(2);          // value of center white sensor
		int right_white_line = ADC_Conversion(3);           // value of right white sensor
		int proximity_sensor = ADC_Conversion(FRONT_IR_ADC_CHANNEL);  // value of proximity sensor
		cout << endl << left_white_line << "    " << center_white_line << "    " << right_white_line << "    " << proximity_sensor << endl;     // prints values of all four sensors on debugging window

		if (left_white_line < THRESHOLD && center_white_line == THRESHOLD && right_white_line < THRESHOLD)          // condition when center sensor detect black line go forward
		{
			forward();
		}
		if (left_white_line < THRESHOLD && center_white_line == THRESHOLD && right_white_line == THRESHOLD)          // condition when center and right sensor detect black line go right
		{
			soft_right();
			velocity(200, 200);
		}
		if (left_white_line < THRESHOLD && center_white_line < THRESHOLD && right_white_line == THRESHOLD)          // condition when right sensor detect black line go right
		{
			soft_right();
			velocity(200, 200);
		}
		if (left_white_line == THRESHOLD && center_white_line == THRESHOLD && right_white_line < THRESHOLD)          // condition when left and center sensor detect black line go left
		{
			soft_left();
			velocity(200, 200);
		}
		if (left_white_line == THRESHOLD && center_white_line < THRESHOLD && right_white_line < THRESHOLD )          // condition when left sensor detect black line go left
		{
			soft_left();
			velocity(200, 200);
		}
		if (left_white_line == THRESHOLD && center_white_line < THRESHOLD && right_white_line == THRESHOLD)          // condition when left and right sensor detect black line go left
		{
			soft_left();
			velocity(200, 200);
		}
		if (left_white_line == THRESHOLD && center_white_line == THRESHOLD && right_white_line == THRESHOLD)          // condition when center sensor detect black line go forward
		{
			stop();
			break;
		}
		if (proximity_sensor > 10 && proximity_sensor < 20)          // condition when the obstacle is detected
		{
			// checks the given distance value with the next value of proximity sensor
			if (distance() == proximity_sensor)
			{
				cout << "obstacle" << endl;
				left_turn_wls();
				line_following();
				// deletes the path where the obstacle is placed
				graph[src][g] = 0;
				graph[g][src] = 0;
				path.clear();
				// finds the next shortest path from the previous node
				dijkstra(graph, src, kk);
				// move through the next shortest path
				for (int l = 0; l < path.size(); l++)
				{
					cout << "                                   " << g << "      " << src << "       " << path[l] << endl;
					movement(g, src, path[l]);
					g = src;
					src = path[l];
				}
				t = g;
				return 1;
			}
		}
	}
	return 0;
}

/*
* Function Name : distance
* Input : void
* Output : int
* Logic : Uses proximity sensor to find the distance and store in length variable
* Example Call : distance()   // returns the length between robot and the obstacle
*/

int distance(void)
{
	int length = ADC_Conversion(FRONT_IR_ADC_CHANNEL);
	cout << length << "    " << "obstacle";
	return length;
}
