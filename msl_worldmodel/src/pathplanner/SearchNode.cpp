/*
 * SearchNode.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: Stefan Jakob
 */

#include "pathplanner/SearchNode.h"

using std::shared_ptr;

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

SearchNode::SearchNode(Vertex vertex, double cost, double heuristic, shared_ptr<SearchNode> predecessor)
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

double SearchNode::getCost() const
{
    return cost;
}

void SearchNode::setCost(double cost)
{
    this->cost = cost;
}

double SearchNode::getHeuristic() const
{
    return heuristic;
}

void SearchNode::setHeuristic(double heuristic)
{
    this->heuristic = heuristic;
}

shared_ptr<const SearchNode> SearchNode::getPredecessor() const
{
    return predecessor;
}

shared_ptr<SearchNode> SearchNode::getPredecessor()
{
    return predecessor;
}

void SearchNode::setPredecessor(shared_ptr<SearchNode> predecessor)
{
    this->predecessor = predecessor;
}

VoronoiDiagram::Halfedge_around_vertex_circulator SearchNode::getEdge() const
{
    return edge;
}

bool SearchNode::matches(const Vertex &vertex)
{
    if (initialEdge)
    {
        return (*this->vertex) == vertex;
    }
    else
    {
        return (*this->edge->source()) == vertex;
    }
}

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

geometry::CNPointAllo SearchNode::getPoint()
{
    if (this->initialEdge)
    {
        return geometry::CNPointAllo(this->vertex->point().x(), this->vertex->point().y());
    }
    else
    {
        return geometry::CNPointAllo(this->edge->source()->point().x(), this->edge->source()->point().y());
    }
}

bool SearchNode::compare(shared_ptr<const SearchNode> first, shared_ptr<const SearchNode> second)
{
    return (first->cost < second->cost);
}

} /* namespace alicaTests */
