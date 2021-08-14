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

	void insertHelper(Node** node, int depth, std::vector<float> point, int id) {

		if (*node == nullptr) {
			*node = new Node(point, id);
			return;
		}

		int cd = depth & 1;

		if (point[cd] < ((*node)->point[cd])) {
			insertHelper(&((*node)->left), depth + 1, point, id);
		}
		else {
			insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	inline float get_distance(std::vector<float> a, std::vector<float> b) {
		float x = a[0] - b[0];
		float y = a[1] - b[1];

		return std::sqrtf(x * x + y * y);
	}

	inline bool is_in_box_of_distance(std::vector<float> v, std::vector<float> target, float dist) {
		return (target[0] + dist) >= v[0] && (target[1] + dist) >= v[1] && (target[0] - dist) <= v[0] && (target[1] - dist) <= v[1];
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
		if (node == nullptr) {
			return;
		}

		if (is_in_box_of_distance(node->point, target, distanceTol)) {
			float distance = get_distance(node->point, target);
			if (distance <= distanceTol) {
				ids.push_back(node->id);
			}
		}

		int cd = depth & 1;
		if ((target[cd] - distanceTol) < node->point[cd]) {
			searchHelper(target, node->left, depth + 1, distanceTol, ids);
		}

		if ((target[cd] + distanceTol) >= node->point[cd]) {
			searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}

	}

	inline void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	inline std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




