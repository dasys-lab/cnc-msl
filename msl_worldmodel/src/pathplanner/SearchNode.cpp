/*
 * SearchNode.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/SearchNode.h"

namespace msl
{

	SearchNode::SearchNode()
	{
		vertex = nullptr;
		cost = std::numeric_limits<double>::max();
		predecessor = nullptr;

	}

	SearchNode::SearchNode(shared_ptr<CNPoint2D> vertex, double cost,
							shared_ptr<SearchNode> predecessor)
	{
		this->vertex = vertex;
		this->cost = cost;
		this->predecessor = predecessor;
	}

	SearchNode::~SearchNode()
	{
	}

	/**
	 * gets the cost
	 * @return double
	 */
	double SearchNode::getCost()
	{
		return cost;
	}

	/**
	 * sets the cost
	 * @param cost double
	 */
	void SearchNode::setCost(double cost)
	{
		this->cost = cost;
	}

	/**
	 * gets the predecessor node
	 * @return shared_ptr<SearchNode>
	 */
	shared_ptr<SearchNode> SearchNode::getPredecessor()
	{
		return predecessor;
	}

	/**
	 * sets the predecessor node
	 * @param predecessor shared_ptr<SearchNode>
	 */
	void SearchNode::setPredecessor(shared_ptr<SearchNode> predecessor)
	{
		this->predecessor = predecessor;
	}

	/**
	 * gets the vertex
	 * @return shared_ptr<VoronoiDiagram::Vertex>
	 */
	shared_ptr<CNPoint2D> SearchNode::getVertex()
	{
		return vertex;
	}

	/**
	 * sets the vertex
	 * @param vertex shared_ptr<VoronoiDiagram::Vertex>
	 */
	void SearchNode::setVertex(shared_ptr<CNPoint2D> vertex)
	{
		this->vertex = vertex;
	}

	/**
	 * compares two SearchNodes, true if first has lower cost
	 * @param first shared_ptr<SearchNode>
	 * @param second shared_ptr<SearchNode>
	 * @return bool
	 */
	bool SearchNode::compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second)
	{
		return (first->cost < second->cost);
	}

} /* namespace alicaTests */
