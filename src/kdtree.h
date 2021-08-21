#pragma once

// Structure to represent node of kd tree
struct Node
{
	float * point;
	int id;
	Node* left;
	Node* right;

	Node(float * arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template <typename PointT, int Depth>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)	{
	
	}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node** node, int depth, float * point, int id) {

		if (*node == nullptr) {
			*node = new Node(point, id);
			return;
		}

		int cd = depth % Depth;

		if (point[cd] < ((*node)->point[cd])) {
			insertHelper(&((*node)->left), depth + 1, point, id);
		}
		else {
			insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	inline float get_distance(float * a, float * b) {

		float res = 0;
		for (int i = 0; i < Depth; i++) {
			res += (a[i] - b[i]) * (a[i] - b[i]);
		}

		return std::sqrtf(res);
	}

	inline bool is_in_box_of_distance(float * v, float * target, float dist) {

		bool res = true;
		for (int i = 0; i < Depth; i++) {
			if((target[i] + dist) < v[i] || (target[i] - dist) > v[i]) {
				res = false;
				break;
			}
		}
		//return (target[0] + dist) >= v[0] && (target[1] + dist) >= v[1] && (target[0] - dist) <= v[0] && (target[1] - dist) <= v[1];
		return res;
	}

	void searchHelper(float * target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
		if (node == nullptr) {
			return;
		}

		if (is_in_box_of_distance(node->point, target, distanceTol)) {
			float distance = get_distance(node->point, target);
			if (distance <= distanceTol) {
				ids.push_back(node->id);
			}
		}

		int cd = depth % Depth;
		if ((target[cd] - distanceTol) < node->point[cd]) {
			searchHelper(target, node->left, depth + 1, distanceTol, ids);
		}

		if ((target[cd] + distanceTol) >= node->point[cd]) {
			searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}

	}

	inline void insert(PointT point, int id)
	{
		insertHelper(&root, 0, point.data, id);
	}

	// return a list of point ids in the tree that are within distance of target
	inline std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target.data, root, 0, distanceTol, ids);
		return ids;
	}
};




