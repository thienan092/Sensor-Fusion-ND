/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
  
    void insertHelper(Node*& node, uint level, std::vector<float> point, int id)
    {
        uint cd = level % 3;
        
        if(node == NULL)
        {
            node = new Node(point, id);
        }
        else if(point[cd] < node->point[cd])
        {
            insertHelper(node->left, level+1, point, id);
        }
        else
        {
            insertHelper(node->right, level+1, point, id);
        }
     }


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(root, 0, point, id);
	}
  
    void searchHelper(std::vector<float> target, Node* node, uint level, float distanceTol, std::vector<int> &ids) const 
    {
        uint cd = level % 3;
        
        if(node != NULL)
        {
            float dx = node->point[0] - target[0];
            float dy = node->point[1] - target[1];
            float dz = node->point[2] - target[2];
            float d = sqrt(dx*dx + dy*dy + dz*dz);

            if (d <= distanceTol)
            {
                ids.push_back(node->id);
            }
          
            if(target[cd] < (node->point[cd] + distanceTol))
            {
                searchHelper(target, node->left, level+1, distanceTol, ids);
            }
          
            if(target[cd] > (node->point[cd] - distanceTol))
            {
                searchHelper(target, node->right, level+1, distanceTol, ids);
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) const 
	{
		std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};