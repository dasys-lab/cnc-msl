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

	SearchNode::SearchNode(shared_ptr<VoronoiDiagram::Vertex> vertex, double cost,
							shared_ptr<SearchNode> predecessor)
	{
		this->vertex = vertex;
		this->cost = cost;
		this->predecessor = predecessor;
	}

	SearchNode::~SearchNode()
	{
	}

	double SearchNode::getCost()
	{
		return cost;
	}

	void SearchNode::setCost(double cost)
	{
		this->cost = cost;
	}

	const shared_ptr<SearchNode> SearchNode::getPredecessor()
	{
		return predecessor;
	}

	void SearchNode::setPredecessor(shared_ptr<SearchNode> predecessor)
	{
		this->predecessor = predecessor;
	}

	shared_ptr<VoronoiDiagram::Vertex> SearchNode::getVertex()
	{
		return vertex;
	}

	void SearchNode::setVertex(shared_ptr<VoronoiDiagram::Vertex> vertex)
	{
		this->vertex = vertex;
	}

	bool SearchNode::compare(shared_ptr<SearchNode> first, shared_ptr<SearchNode> second)
	{
		return (first->cost < second->cost);
	}

} /* namespace alicaTests */
