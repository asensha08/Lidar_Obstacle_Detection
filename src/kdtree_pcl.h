#ifndef KDTREE_PCL_H
#define KDTREE_PCL_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node *left;
    Node *right;

    Node(PointT p, int setId)
	:	point(p), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT, typename PointIndice>
struct KdTree
{
    Node<PointT> *root;
    KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

    void insertHelper(Node<PointT> *&node, uint depth, PointT point, int id)
    {
        if(NULL == node)
        {
            node = new Node<PointT>(point, id);
        }
        else
        {
            int cd = depth %3;
            if(cd ==0)
            {
                if(point.x <(node->point.x))
                {
                    insertHelper(node->left, depth+1, point, id);
                }
                else
                {
                    insertHelper(node->right, depth+1, point, id);
                }
            }
            else if(cd ==1)
            {
               if(point.y <(node->point.y))
                {
                    insertHelper(node->left, depth+1, point, id);
                }
                else
                {
                    insertHelper(node->right, depth+1, point, id);
                } 
            }
            else
            {
                if(point.z <(node->point.z))
                {
                    insertHelper(node->left, depth+1, point, id);
                }
                else
                {
                    insertHelper(node->right, depth+1, point, id);
                } 
            }
        }
    }

    void insert(PointT point, int id)
    {
        insertHelper(root, 0, point, id);
    }

    void searchHelper(PointT target, Node<PointT> *node, int depth, float distanceTol, std::vector<PointIndice> &indices)
    {
        if(node != NULL)
        {
            if( (node->point.x >=(target.x - distanceTol) &&
               node->point.x <=(target.x + distanceTol))&&
               (node->point.y >=(target.y - distanceTol) &&
               node->point.y <=(target.y + distanceTol)) &&
               (node->point.z >=(target.z - distanceTol) &&
               node->point.z <=(target.z + distanceTol)))
               {
                   float x_dist_sq = (node->point.x - target.x) *  (node->point.x - target.x);
                   float y_dist_sq = (node->point.y - target.y) *  (node->point.y - target.y);
                   float z_dist_sq = (node->point.z - target.z) *  (node->point.z - target.z);
                   float distance = sqrtf(x_dist_sq * y_dist_sq * z_dist_sq);
                   if(distance <=distanceTol)
                   {
                       indices.push_back(node->id);
                   }
               }
               uint index = depth % 3;
               float value, node_value;
               switch(index)
               {
                   case 0: value = target.x; node_value = node->point.x;
                   case 1: value = target.y; node_value = node->point.y;
                   case 2: value = target.z; node_value = node->point.z;

               }
               if(value - distanceTol < node_value)
               {
                   searchHelper(target, node->left, depth+1, distanceTol, indices);
               }
               if(value - distanceTol > node_value)
               {
                   searchHelper(target, node->right, depth+1, distanceTol, indices);

               }

        }
    }

    std::vector<PointIndice> search(PointT target, float distanceTol)
    {
        std::vector<PointIndice> indices;
        searchHelper(target,root, 0, distanceTol, indices);

        return indices;
    }
};
#endif