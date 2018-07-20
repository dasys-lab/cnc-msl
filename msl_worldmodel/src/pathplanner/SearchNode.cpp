/*
 * SearchNode.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/SearchNode.h"

namespace msl
{

//	SearchNode::SearchNode()
//	{
//		edge = nullptr;
//		cost = std::numeric_limits<double>::max();
//		heuristic = std::numeric_limits<double>::max();
//		predecessor = nullptr;
//
//	}

SearchNode::SearchNode(VoronoiDiagram::Halfedge_around_vertex_circulator edge, double cost, double heuristic, shared_ptr<SearchNode> predecessor)
{
    this->initialEdge = false;
    this->edge = edge;
    this->cost = cost;
    this->heuristic = heuristic;
    this->predecessor = predecessor;
}

SearchNode::SearchNode(shared_ptr<Vertex> vertex, double cost, double heuristic, shared_ptr<SearchNode> predecessor)
{
    this->initialEdge = true;
    this->vertex = vertex;
    this->cost = cost;
    this->heuristic = heuristic;
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
 * gets the heuristic
 * @return double
 */
double SearchNode::getHeuristic()
{
    return heuristic;
}

/**
 * sets the heuristic
 * @param heuristic double
 */
void SearchNode::setHeuristic(double heuristic)
{
    this->heuristic = heuristic;
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
 * gets the edge
 * @return VoronoiDiagram::Halfedge_around_vertex_circulator
 */
VoronoiDiagram::Halfedge_around_vertex_circulator SearchNode::getEdge()
{
    return edge;
}

bool SearchNode::matches(shared_ptr<Vertex> vertex)
{
    if (initialEdge)
    {
        return (*this->vertex) == (*vertex);
    }
    else
    {
        return (*this->edge->source()) == (*vertex);
    }
}

/**
 * sets the edge
 * @param VoronoiDiagram::Halfedge_around_vertex_circulator
 */
void SearchNode::setEdge(VoronoiDiagram::Halfedge_around_vertex_circulator edge)
{
    this->edge = edge;
}

VoronoiDiagram::Halfedge_around_vertex_circulator SearchNode::getIncidentEdges()
{
    if (this->initialEdge)
    {
        return this->vertex->incident_halfedges();
    }
    else
    {
        return this->edge->source()->incident_halfedges();
    }
}

shared_ptr<geometry::CNPoint2D> SearchNode::getPoint()
{
    if (this->initialEdge)
    {
        return make_shared<geometry::CNPoint2D>(this->vertex->point().x(), this->vertex->point().y());
    }
    else
    {
        return make_shared<geometry::CNPoint2D>(this->edge->source()->point().x(), this->edge->source()->point().y());
    }
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
