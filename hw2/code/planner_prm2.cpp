/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <utility>
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 
#include <queue>
#include <float.h>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;
using std::pair;
using std::make_pair;

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size

/*
output is a double, two intergers:loadmap(takes in filepath)
f is the handle to open the file
if statement to check if the file to load the map has failed

defined two int variables height and width

*/

tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}

	/*
	new double 'map' which is same as height*width
	initialise three doubles cx,cy,cz

	 _____x______
	|
	|
	Y	map
	|
	|
	|
	LOOP
	for each coordinate of map  (integers x and y)
		define a character c
		if that is empty then give an runtime error
		if it is not zero, update map index to 1

	*/
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

/*
split function, reuturns a vector of strings based on a delimiter
*/
// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}

/*
takes in a string and returns a double from the string
uses split function above that returns a vector of string and stores it in a varibles defined as such, 
delimiter here is ,
define a new double called 'ans' which has a size same as split 'str'
convert all the strings to double by looping it through all the strings
return the doubles
*/
double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}
/*
compared two double arrays
up to a given 'size' if any elemement between those arrays differs by more that 0.001 then return false
i.e. the arrays are not the same
in the end, return true. as they are same upto the given 'size'
*/
bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

/*
this is a void, does not return anything
takes in two doubles, two unsigned ints and two ints
define a cellsize
convert x and y to
int * pY
*/
void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}


int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

// compute l2 norm
double l2norm(double* a, double* b, int n) {
	double sum = 0;
	for (int i = 0; i < n; ++i) {
		sum += pow(a[i] - b[i], 2);
	}
	return sqrt(sum);
}

// create a random array of size n 
double* randomArray(int n, double* goal_angles) {
	double* arr = new double[n];
    double r = (double)rand() / (double)RAND_MAX;
    if (r > 0.9) {
        // 10% of the time, returning the goal
        return goal_angles;
    }
	for (int i = 0; i < n; ++i) {
		arr[i] = 2.0*PI*((double)rand() / RAND_MAX);
	}
	return arr;
}

double* randomSam_prm(int n, double* map, int x_size, int y_size) {
	double* arr = new double[n];
	while(1){
		for (int i = 0; i < n; ++i) {
			arr[i] = 2.0*PI*((double)rand() / RAND_MAX);
		}
		if (IsValidArmConfiguration(arr, n, map, x_size, y_size)) {
			return arr;
		}
	}
}

// create a struct of of two vectors of doubles
struct vertex {
	//assign vertex id
	int vertex_id;
	double* angles;
    double cost; 
	vertex* parent; //only for RRTs, not for PRM
	vector<vertex*> neighbors; //only for PRM
}; 
 

vertex* nearestAngle(vector<vertex*> tree, double* sample_q, int n, double* start ) {
	double minDist = 100000000;
	vertex* nearest = NULL;
	for (int i = 0; i < tree.size(); ++i) {
		double dist = l2norm(tree[i]->angles, sample_q, n);
		if (dist < minDist) {
			minDist = dist;
			nearest = tree[i];
		}
	}
    nearest->cost = l2norm(nearest->angles, start, n);
	return nearest;

}

vertex* extend(vertex* nearest_neighbor, double* sample_q, int numofDOFs, double* map, int x_size, int y_size, bool epsilon,double min_angle, double* armstart) {
	double* new_q = new double[numofDOFs];

	vertex* extended_vertex = new vertex;
	extended_vertex->parent = nearest_neighbor;
    extended_vertex->angles = new double[numofDOFs];
    extended_vertex->cost = nearest_neighbor->cost;

    for(int i = 0; i < numofDOFs; i++){
        extended_vertex->angles[i] = nearest_neighbor->angles[i];
    }

    double distance=0.0;
    for (int i=0; i<numofDOFs; i++){
        if (abs(sample_q[i]-nearest_neighbor->angles[i])>distance){
            distance = abs(sample_q[i]-nearest_neighbor->angles[i]);
        }
    }

    int steps=(int)(distance/min_angle);

	for (int i = 1; i <= steps; i++) {
		for (int j =0; j< numofDOFs; j++) {
			new_q[j] = nearest_neighbor->angles[j] + (double)i*(sample_q[j] - nearest_neighbor->angles[j])/steps;
		}
		if ((!IsValidArmConfiguration(new_q, numofDOFs, map, x_size, y_size)) || (l2norm(new_q, sample_q, numofDOFs) > epsilon)) {
            return extended_vertex;
			}
		for (int j = 0; j < numofDOFs; ++j) {
			extended_vertex->angles[j] = new_q[j];
		}
        extended_vertex->cost = nearest_neighbor->cost + l2norm(extended_vertex->angles, nearest_neighbor->angles, numofDOFs);

	}
	return extended_vertex;
}

pair<int,double**> backtrace(vertex* goal, int numofDOFs) {

	//write a sample while loop
	int path_len = 1;
	vertex* current = goal;
	while (current->parent != NULL) {
		++path_len;
		current = current->parent;
	}

	//allocate memory for the plan
	double** plan = new double*[path_len];
	for (int j = 0; j < path_len; ++j) {
		plan[j] = new double[numofDOFs];
	}

	//write a second while loop
	int k = path_len-1;
	current = goal;
	for (int i = k; i >= 0; i--) {
		for (int j = 0; j < numofDOFs; ++j) {
			plan[i][j] = current->angles[j];
		}
		current = current->parent;
	}
	cout<<"path length: "<<path_len<<endl;
	cout<<"Plan Backtrace: "<<endl;
	for (int i = 0; i <= path_len-1; ++i) {
		for (int j = 0; j < numofDOFs; ++j) {
			cout<<plan[i][j]<<" ";
		}
		cout<<endl;
	}
	return make_pair(path_len, plan);
}

bool IsObstacleFree(double* q1,double* q2, int numofDOFs, double* map, int x_size, int y_size, double min_angle) {

    double distance=0.0;

    for (int i=0; i<numofDOFs; i++){
        if (abs(q1[i]-q2[i])>distance){
            distance = abs(q1[i]-q2[i]);
        }
    }

    int steps=(int)(distance/min_angle);

    for (int i=1; i<=steps; i++){
        double* q = new double[numofDOFs];
        for (int j=0; j<numofDOFs; j++){
            q[j] = q1[j] + (q2[j]-q1[j])*(double(i)/steps);
        }
        if (!IsValidArmConfiguration(q, numofDOFs, map, x_size, y_size)){
            return false;
        }
    }
    return true;
}

struct compare_cost {
	bool operator()(pair<double, vertex*> const& a, pair<double, vertex*> const& b) {
		return a.first > b.first;
	}
};

static void findgraph(vector<vertex*> &tree, vertex* q, int numofDOFs, double radius, int N, double* map,  int x_size, int y_size) {
	std::priority_queue<pair<double,vertex*>,vector<pair<double,vertex*>>,compare_cost> pq;

	for (int i = 0; i < tree.size(); ++i) {
		if (l2norm(tree[i]->angles, q->angles, numofDOFs) < radius) {
			if (IsObstacleFree(tree[i]->angles, q->angles, numofDOFs, map, x_size, y_size, 0.1)) {
				pq.push(make_pair(l2norm(tree[i]->angles, q->angles, numofDOFs), tree[i]));
			}
		}
	}
	tree.push_back(q);

	for (int i = 0; i < std::min(N, (int)pq.size()); ++i) {
		pair <double, vertex*> v = pq.top();
		// cout<<"cost: "<<v.first<<endl;
		q->neighbors.push_back(v.second);
		v.second->neighbors.push_back(q);
		pq.pop();
	}
	
}

double planning_cost(double*** plan, int* planlength, int numofDOFs) {
    double total_cost = 0;

    for (int i = 0; i < *planlength-1; i++) {
        double* curr_step = (*plan)[i];
        double* next_step = (*plan)[i + 1];
        total_cost += l2norm(curr_step, next_step, numofDOFs);
    }
    return total_cost;
}

struct compare_dik_cost {
	bool operator()( vertex* a, vertex*  b) {
		return a->cost > b->cost;
	}
};

static void Dij_search(vertex* start, vertex* goal, vector<vertex*> roadmap, double*** plan, int* planlength, int numofDOFs){

//intialise a open list as a priority queue 
std::priority_queue<vertex*,vector<vertex*>,compare_dik_cost> open_list;
//intialise a closed list as a vector
vector<bool> closed_list(roadmap.size(),false);

bool goal_reached = false;

start->cost = 0;
open_list.push(start);

while(!open_list.empty() && !goal_reached){
	vertex* current = open_list.top();
	cout<<"current: "<<current->angles[0]<<" "<<current->angles[1]<<" "<<current->angles[2]<<" "<<current->angles[3]<<" "<<current->angles[4]<<" "<<current->angles[5]<<" "<<current->angles[6]<<endl;
	open_list.pop();
	if(!closed_list[current->vertex_id]){
		closed_list[current->vertex_id] = true;
		for (int i = 0; i < current->neighbors.size(); ++i) {
			vertex* neighbor = current->neighbors[i];
			if(!closed_list[neighbor->vertex_id]){
				double new_cost = current->cost + l2norm(current->angles, neighbor->angles, numofDOFs);
				if(new_cost < neighbor->cost){
					neighbor->cost = new_cost;
					neighbor->parent = current;
				}
				open_list.push(neighbor);
			}
		}
	}


if (closed_list[goal->vertex_id]) {
	goal_reached = true;
	}

if (goal_reached) {
	pair <int, double**> path = backtrace(goal, numofDOFs);
	*planlength = path.first;
	*plan = path.second;
}
}
return;
}

//create a static void function called PRM
static void prm(double* map,
				int x_size,
				int y_size,
				double* armstart_anglesV_rad,
				double* armgoal_anglesV_rad,
				int numofDOFs,
				double*** plan,
				int* planlength,
				int K,
				double radius,
				int N_neighbours){

	vector <vertex*> roadmap;

	vertex* first = new vertex;
	first->angles = randomSam_prm(numofDOFs, map, x_size, y_size);
	first->parent = 0;
	first->vertex_id = 1;
	first->cost = DBL_MAX;
	roadmap.push_back(first);

	cout<<"Roadmap initial size: "<<roadmap.size()<<endl;

	for (int i=2; i<=K; i++){
		double* sample_q = randomSam_prm(numofDOFs, map, x_size, y_size);
		
		vertex* new_vertex = new vertex;
		new_vertex->vertex_id = i;
		new_vertex->parent = 0;
		new_vertex->angles = sample_q;
		new_vertex->cost = DBL_MAX;

		findgraph(roadmap, new_vertex, numofDOFs, radius, N_neighbours, map, x_size, y_size);
	}
	cout<<"Roadmap final size: "<<roadmap.size()<<endl;

// make a new vertex called start_vertex
vertex* start_vertex = new vertex;
start_vertex->cost = 0;
start_vertex->parent = NULL;
start_vertex->angles= armstart_anglesV_rad;
start_vertex->vertex_id = 0;

findgraph(roadmap, start_vertex, numofDOFs, radius, N_neighbours, map, x_size, y_size);

vertex* goal_vertex = new vertex;
goal_vertex->cost = DBL_MAX;
goal_vertex->parent = NULL;
goal_vertex->angles= armgoal_anglesV_rad;
goal_vertex->vertex_id = roadmap.size()+1;

findgraph(roadmap, goal_vertex, numofDOFs, radius, N_neighbours, map, x_size, y_size);

//if number of neighbors for the start and goal vertex are less than 1 then print "No path found"
if (start_vertex->neighbors.size() < 1 || goal_vertex->neighbors.size() < 1) {
	cout<<"No neighbours for start or goal found"<<endl;
	return;
}

//search for the path using Dijkstra's algorithm
Dij_search(start_vertex, goal_vertex, roadmap, plan, planlength, numofDOFs);
}

static void planner(
			int whichplanner,
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength
			)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

    if (whichplanner == 0){
		//print calling rrt
		printf("calling prm\n");
		prm(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength, 1000,3.0,5);
	}
}


/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;


	/*
	tie creates a tuple of references of the same size as the argumestts
	*/

	tie(map, x_size, y_size) = loadMap(argv[1]);

	// print argc using cout
	// cout << "argc: " << argc << endl;
	// print argv using cout
	// cout << "argv: " << argv[0] << endl;

	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
	planner(whichPlanner,map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}

