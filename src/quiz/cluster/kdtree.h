/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#define X 	0
#define Y 	1
#define Z	2
#define XYZ 3
#define BOX	6
#define square(a) (a*a)
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id){

		if( *node == NULL){
			*node = new Node(point,id);
		}else{
			uint cd = depth % XYZ;

			if( point[cd] < ((*node)->point[cd])  ){
				insertHelper(&((*node)->left),depth+1,point,id);
			}else{
				insertHelper(&((*node)->right),depth+1,point,id);
			}
		}

	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);
	}

	float getDistance( Node *node,std::vector<float> target, float distanceTol){
		float targetBox[BOX],distance[XYZ];
		targetBox[X] 	= target[X] - distanceTol; 	targetBox[X+XYZ]	= target[X] + distanceTol;
		targetBox[Y] 	= target[Y] - distanceTol; 	targetBox[Y+XYZ]	= target[Y] + distanceTol;
		targetBox[Z] 	= target[Z] - distanceTol; 	targetBox[Z+XYZ]	= target[Z] + distanceTol;

		distance[X] 	= node->point[X]-target[X]; 
		distance[Y]		= node->point[Y]-target[Y];
		distance[Z]		= node->point[Z]-target[Z];
		if(( targetBox[X] <= node->point[X] && node->point[X]  <= targetBox[X+XYZ] )
		  && 	( targetBox[Y] <= node->point[Y] && node->point[Y]  <= targetBox[Y+XYZ] )
		  &&	( targetBox[Z] <= node->point[Z] && node->point[Z]  <= targetBox[Z+XYZ] ))
			return sqrt(distance[X]*distance[X] 
					+ distance[Y]*distance[Y]
					+ distance[Z]*distance[Z]);
		else	
			return -1;
		
	}

	void searchSideKick(Node *node,  std::vector<float> target,int depth,  float distanceTol, std::vector<int> &search_results ){

		if(node != NULL){
			float distance = getDistance(node,target,distanceTol);
			//std::cout <<"distance" <<distance<< std::endl;
			if( -1 < distance  && distance <= distanceTol)
				search_results.push_back(node->id); 

			if((target[depth % XYZ] - distanceTol) < node->point[depth % XYZ])
				searchSideKick(node->left, target, depth+1, distanceTol, search_results);
			if((target[depth % XYZ] + distanceTol) > node->point[depth % XYZ])
				searchSideKick(node->right, target, depth+1, distanceTol, search_results);
		}
		 

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		Node *start = root;
		searchSideKick(start, target, X, distanceTol,ids);
		return ids;
	}
	

};




