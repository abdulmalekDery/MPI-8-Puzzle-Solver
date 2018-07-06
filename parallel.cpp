#define _CRT_SECURE_NO_WARNINGS

#include<iostream>
#include<algorithm>
#include<stdio.h>
#include<vector>
#include <mpi.h>
using namespace std;


#define START_BOARD 1
#define GOAL_BOARD 2
#define SOLUTION_DISTANCE 3
#define SOLUTION_RANK 4
#define BOARD_SIZE 5
#define SON_DISTENCE 6

const bool SUCCESS = true;
int boardSize = 3;
int startBoard[100];
int goalBoard[100];

class state {

public:
	int board[10][10];
	int g, f;

	state *came_from;

	state() {
		g = 0;
		f = 0;
		came_from = NULL;
	}

	static int heuristic(state from, state to) {
		int i, j, dist = 0;
		for (i = 0; i < boardSize; i++)
			for (j = 0; j < boardSize; j++)
				if (from.board[i][j] != to.board[i][j])
					dist++;
		return dist;
	}

	bool operator ==(state a) {
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++)
				if (this->board[i][j] != a.board[i][j])
					return false;
		return true;
	}

	void print() {
		for (int i = 0; i < boardSize; i++) {
			for (int j = 0; j < boardSize; j++)
				cout << board[i][j] << " ";
			cout << endl;
		}
		printf("g: %d\n", g);
		printf("f: %d\n\n", f);
	}
};

vector<state>parentPaths;


///* This method checks whether a given problem is solvable or not*/
bool isSolvable(int *board, int dim)
{
	int i, j, zeroPos = 0, blankRow, inversions = 0;
	int size = dim*dim;
	bool solvable = false;

	// Create a copy of the original board
	int *newBoard;
	newBoard = (int*)malloc(sizeof(size));
	memcpy(newBoard, board, size);

	// get the position of the zero element
	for (i = 0; i < size; i++) {
		if (newBoard[i] == 0) {
			zeroPos = i;
		}
	}

	// get the row where zero is located
	blankRow = zeroPos / dim;

	// delete zero element
	for (i = zeroPos; i < size - 1; i++) {
		newBoard[i] = newBoard[i + 1];
	}

	// get num of inversions of the newBoard
	for (i = 0; i < size - 2; i++) {
		for (j = i + 1; j < size - 1; j++) {
			if (newBoard[i] > newBoard[j]) {
				inversions++;
			}
		}
	}

	// check base on inversion rules
	if (dim % 2 == 0) {
		if ((blankRow + inversions) % 2 != 0) {
			solvable = true;
		}
	}
	else {
		if (inversions % 2 == 0) {
			solvable = true;
		}
	}

	return solvable;
}

bool isSOLVABLE(state temp, int n) {
	int i, z, q, k, w, m = 0, j, x;
	bool s = true;
	for (i = 0; i < n; i++)
		for (z = 0; z < n; z++)
		{
			if (z != n - 1) { q = z + 1; w = i; }
			else { q = 0; w = i + 1; }
			for (k = w; k < n; k++)
			{

				for (j = q; j < n; j++)
				{
					q = 0;
					if ((temp.board[i][z] > temp.board[k][j]) && (temp.board[i][z] != 0) && (temp.board[k][j] != 0))
						m++;
				}
			}
		}
	if (n % 2 == 1)			// n is odd?
	{
		if (m % 2 == 1)	//n is odd and and inversions are odd. Puzzle is insolvable.
			s = false;
	}
	else				//n is even
	{
		for (i = 0; i < n; i++)
			for (j = 0; j < n; j++)
				if (temp.board[i][j] == 0) x = i;	//find the row of the space abd put it into the variable x.

		if ((m + x) % 2 == 0)	//inversions + space row is an even number and puzzle is unsilvable.
			s = false;
	}
	return s;
}

bool lowerF(state a, state b) {
	return a.f < b.f;
}

void swap(state &a, int i, int j, int posi, int posj) {
	int temp;
	temp = a.board[i][j];
	a.board[i][j] = a.board[posi][posj];
	a.board[posi][posj] = temp;
}

bool isInSet(state a, vector<state> b) {
	for (int i = 0; i < b.size(); i++)
		if (a == b[i])
			return true;

	return false;
}

void addNeighbor(state current, state goal, int newI, int newJ, int posi, int posj,
	vector<state> &openset, vector<state> closedset) {

	state newstate = current;
	swap(newstate, newI, newJ, posi, posj);
	if (!isInSet(newstate, closedset)) {

		int tentative_g_score = current.g + 1;

		if (!isInSet(newstate, openset) || tentative_g_score < newstate.g) {

			newstate.g = tentative_g_score;
			newstate.f = newstate.g + state::heuristic(newstate, goal);

			state *temp = new state();
			*temp = current;
			newstate.came_from = temp;
			openset.push_back(newstate);
		}
	}
}

void neighbors(state current, state goal, vector<state> &openset, vector<state> closedset) {
	int i, j, posi, posj;

	//Find the position of '0'
	for (i = 0; i < boardSize; i++)
		for (j = 0; j < boardSize; j++)
			if (current.board[i][j] == 0)
			{
				posi = i;
				posj = j;
			}

	i = posi; j = posj;

	if ((i - 1) >= 0) {
		addNeighbor(current, goal, (i - 1), j, posi, posj, openset, closedset);
	}

	if ((i + 1) < boardSize) {
		addNeighbor(current, goal, (i + 1), j, posi, posj, openset, closedset);
	}

	if ((j - 1) >= 0) {
		addNeighbor(current, goal, i, (j - 1), posi, posj, openset, closedset);
	}

	if ((j + 1) < boardSize) {
		addNeighbor(current, goal, i, (j + 1), posi, posj, openset, closedset);
	}
}

void reconstruct_path(state current, vector<state> &came_from) {
	state *temp = &current;
	while (temp != NULL) {
		came_from.push_back(*temp);
		temp = temp->came_from;
	}
}

bool astar(state start, state goal, vector<state> &output) {
	vector<state> closedset;
	vector<state> openset;

	state current;

	start.g = 0;
	start.f = start.g + state::heuristic(start, goal);

	openset.push_back(start);
	int y = 0;
	while (!openset.empty()) {

		sort(openset.begin(), openset.end(), lowerF);

		current = openset[0]; //select the state having lowest fscore value.

		if (current == goal) {
			reconstruct_path(current, output);
			return SUCCESS;
		}

		openset.erase(openset.begin());
		closedset.push_back(current);

		neighbors(current, goal, openset, closedset);
	}

	return !SUCCESS;
}

bool validState(state current, int newI, int newJ, int posi, int posj,
	vector<state> &openset, vector<state> closedset) {

	state newstate = current;
	swap(newstate, newI, newJ, posi, posj);
	if (!isInSet(newstate, closedset)) {

		int tentative_g_score = current.g + 1;

		if (!isInSet(newstate, openset) || tentative_g_score < newstate.g) {
			return true;
		}
	}
	return false;
}

int test(state current, vector<state> openset, vector<state> closedset) {//to find the number of the next available states
	int m = 0;
	//todo count
	int i, j, posi, posj;

	//Find the position of '0'
	for (i = 0; i < boardSize; i++)
		for (j = 0; j < boardSize; j++)
			if (current.board[i][j] == 0)
			{
				posi = i;
				posj = j;
			}

	i = posi; j = posj;

	if ((i - 1) >= 0) {
		if (validState(current, (i - 1), j, posi, posj, openset, closedset))//to test if the next move is not in the close set or the open set
			m++;
	}

	if ((i + 1) < boardSize) {
		if (validState(current, (i + 1), j, posi, posj, openset, closedset))
			m++;
	}

	if ((j - 1) >= 0) {
		if (validState(current, i, (j - 1), posi, posj, openset, closedset))
			m++;
	}

	if ((j + 1) < boardSize) {
		if (validState(current, i, (j + 1), posi, posj, openset, closedset))
			m++;
	}
	return m;
}

int mProgram(state start, state goal, int size) {
	vector<state> closedset;
	vector<state> openset;
	vector<state> output;
	state current;

	start.g = 0;
	start.f = start.g + state::heuristic(start, goal);

	openset.push_back(start);

	while (!openset.empty()) {

		sort(openset.begin(), openset.end(), lowerF);

		current = openset[0]; //select the state having lowest fscore value.

		if (current == goal) {
			reconstruct_path(current, output);
			for (int i = output.size() - 1; i >= 0; i--) {
				output[i].print();
			}
			//todo we should here stop other process
			cout << "\nProcess with rank 0 SUCCESSFUL.\nNumber of moves: " << output.size() - 1 << endl;
			return -1;
		}
		if ((openset.size() - 1 + test(current, openset, closedset)) <= size - 1)
		{
			openset.erase(openset.begin());
			closedset.push_back(current);
			neighbors(current, goal, openset, closedset);
		}
		else
		{
			cout << "number of slaves is " << size - 1 << " number of states in openset is " << openset.size() << "\n";
			fflush(stdout);
			break;
		}
	}

	for (int q = 1; q <= openset.size(); q++)
	{
		int h = 0;
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++) {
				startBoard[h] = openset[q - 1].board[i][j];
				h++;
			}
		int g = openset[q - 1].g;
		MPI_Send(&boardSize, 1, MPI_INT, q, BOARD_SIZE, MPI_COMM_WORLD);
		MPI_Send(&g, 1, MPI_INT, q, SON_DISTENCE, MPI_COMM_WORLD);
		MPI_Send(startBoard, boardSize*boardSize, MPI_INT, q, START_BOARD, MPI_COMM_WORLD);
		MPI_Send(goalBoard, boardSize*boardSize, MPI_INT, q, GOAL_BOARD, MPI_COMM_WORLD);
	}
	for (int q = openset.size(); q <size; q++)
	{
		int dummy = -1;
		MPI_Send(&dummy, 1, MPI_INT, q, BOARD_SIZE, MPI_COMM_WORLD);
	}
	return openset.size();
}


void SProgarm(int rank, int size) {
	MPI_Status status;

	MPI_Recv(&boardSize, 1, MPI_INT, 0, BOARD_SIZE, MPI_COMM_WORLD, &status);
	if (boardSize != -1) {
		int g = 1;
		MPI_Recv(&g, 1, MPI_INT, 0, SON_DISTENCE, MPI_COMM_WORLD, &status);
		MPI_Recv(startBoard, boardSize*boardSize, MPI_INT, 0, START_BOARD, MPI_COMM_WORLD, &status);
		MPI_Recv(goalBoard, boardSize*boardSize, MPI_INT, 0, GOAL_BOARD, MPI_COMM_WORLD, &status);
		int h = 0;
		state start, goal;
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++) {
				start.board[i][j] = startBoard[h];
				goal.board[i][j] = goalBoard[h];
				h++;
			}
		vector<state>outputT;
		if (astar(start, goal, outputT) == SUCCESS) {
			for (int i = outputT.size() - 1; i >= 0; i--) {
				outputT[i].print();
			}
			cout << "\nSUCCESSFUL.\nNumber of moves: " << outputT.size() - 1 << endl;
			cout << "The Rank Is " << rank << endl;
			int a = outputT.size() - 1 + g;
			MPI_Send(&a, 1, MPI_INT, 0, SOLUTION_DISTANCE, MPI_COMM_WORLD);
			MPI_Send(&rank, 1, MPI_INT, 0, SOLUTION_RANK, MPI_COMM_WORLD);
		}
		else
			cout << "\nUNSUCCESSFUL.\n";
	}


}

int main(int argc, char *argv[]) {
	int rank, size;

	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	if (rank == 0)
	{
	
		
		startBoard[0] = 8;
		startBoard[1] = 1;
		startBoard[2] = 3;
		startBoard[3] = 4;
		startBoard[4] = 0;
		startBoard[5] = 2;
		startBoard[6] = 7;
		startBoard[7] = 6;
		startBoard[8] = 5;
       

	   
	   /*startBoard[0] = 1;
	   startBoard[1] = 2;
	   startBoard[2] = 3;
	   startBoard[3] = 4;
	   startBoard[4] = 8;
	   startBoard[5] = 5;
	   startBoard[6] = 7;
	   startBoard[7] = 6;
	   startBoard[8] = 0;*/
	   
		
		/*startBoard[0] = 0;
		startBoard[1] = 1;
		startBoard[2] = 3;
		startBoard[3] = 4;
		startBoard[4] = 2;
		startBoard[5] = 5;
		startBoard[6] = 7;
		startBoard[7] = 8;
		startBoard[8] = 6;*/
		
	   //Unsolvable
	  /* startBoard[0] = 1;
	   startBoard[1] = 2;
	   startBoard[2] = 3;
	   startBoard[3] = 4;
	   startBoard[4] = 5;
	   startBoard[5] = 6;
	   startBoard[6] = 8;
	   startBoard[7] = 7;
	   startBoard[8] = 0;*/

		goalBoard[0] = 1;
		goalBoard[1] = 2;
		goalBoard[2] = 3;
		goalBoard[3] = 4;
		goalBoard[4] = 5;
		goalBoard[5] = 6;
		goalBoard[6] = 7;
		goalBoard[7] = 8;
		goalBoard[8] = 0;

		/*
		startBoard[0] = 1;
		startBoard[1] = 2;
		startBoard[2] = 3;
		startBoard[3] = 4;
		startBoard[4] = 5;
		startBoard[5] = 6;
		startBoard[6] = 0;
		startBoard[7] = 8;
		startBoard[8] = 9;
		startBoard[9] = 10;
		startBoard[10] = 7;
		startBoard[11] = 11;
		startBoard[12] = 13;
		startBoard[13] = 14;
		startBoard[14] = 15;
		startBoard[15] = 12;
		*/
		/*
		goalBoard[0] = 1;
		goalBoard[1] = 2;
		goalBoard[2] = 3;
		goalBoard[3] = 4;
		goalBoard[4] = 5;
		goalBoard[5] = 6;
		goalBoard[6] = 7;
		goalBoard[7] = 8;
		goalBoard[8] = 9;
		goalBoard[9] = 10;
		goalBoard[10] = 11;
		goalBoard[11] = 12;
		goalBoard[12] = 13;
		goalBoard[13] = 14;
		goalBoard[14] = 15;
		goalBoard[15] = 0;
		*/
		boardSize = 3;

		int h = 0;
		state start, goal;
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++) {
				start.board[i][j] = startBoard[h];
				goal.board[i][j] = goalBoard[h];
				h++;
			}

		if (isSOLVABLE(start, boardSize))
		{

			int c = mProgram(start, goal, size);
			if (c == -1)
			{
				cout << " the master solve the problem ";
				for (int i = 1; i < size; i++)
				{
					int dummy = -1;
					MPI_Send(&dummy, 1, MPI_INT, i, BOARD_SIZE, MPI_COMM_WORLD);
				}
			}
			else {
				//after slaves finishing their job
				int recivedStates[100][2];
				for (int i = 1; i <= c; i++) {
					int min = 0, pRank = 0;
					MPI_Status status;
					MPI_Recv(&min, 1, MPI_INT, i, SOLUTION_DISTANCE, MPI_COMM_WORLD, &status);
					MPI_Recv(&pRank, 1, MPI_INT, i, SOLUTION_RANK, MPI_COMM_WORLD, &status);
					recivedStates[i - 1][0] = pRank;
					recivedStates[i - 1][1] = min;
				}
				//if (recivedStates[0][0] != -1)
					for (int i = 0; i < c; i++) {
						cout << "Process Rank is " << recivedStates[i][0] << "\n";
						cout << "Process distance is " << recivedStates[i][1] << "\n";
					}
				/*else {
					cout << "Job Aborted";
				}*/
			}
		}
		else
		{
			cout << "This Board Is UnSolveable";
			for (int i = 1; i < size; i++)
			{
				int dummy = -1;
				MPI_Send(&dummy, 1, MPI_INT, i, BOARD_SIZE, MPI_COMM_WORLD);
			}
		}

	}
	else
	{
		SProgarm(rank, size);
	}

	MPI_Finalize();
	return 0;
}
