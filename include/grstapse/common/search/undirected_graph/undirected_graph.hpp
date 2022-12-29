/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

// Global
#include <memory>
#include <tuple>
#include <vector>
// External
#include <fmt/format.h>
#include <robin_hood/robin_hood.hpp>
// Local
#include "grstapse/common/utilities/custom_hashings.hpp"
#include "grstapse/common/utilities/error.hpp"

namespace grstapse
{
    struct DummyPayload
    {};

    /**!
     * An undirected graph
     */
    template <typename VertexPayload = DummyPayload>
    class UndirectedGraph
    {
       public:
        class Edge;

        /**!
         * A vertex from the undirected graph
         */
        class Vertex
        {
           public:
            //! Constructor
            Vertex(unsigned int id, const std::shared_ptr<VertexPayload>& payload = nullptr)
                : m_id(id)
                , m_payload(payload)
            {}

            //! \returns The identifier for this vertex
            [[nodiscard]] inline unsigned int id() const
            {
                return m_id;
            }

            //! \returns The number of edges this vertex is a part of
            [[nodiscard]] inline unsigned int edgeDegree() const
            {
                return m_edges.size();
            }

            //! \returns A list of edges this vertex is a part of
            [[nodiscard]] inline const std::vector<std::shared_ptr<Edge>>& edges() const
            {
                return m_edges;
            }

            //! \returns The payload associated with this vertex
            const std::shared_ptr<VertexPayload>& payload() const
            {
                return m_payload;
            }

            // Adds an edge that this vertex is a part of
            void addEdge(const std::shared_ptr<Edge>& edge)
            {
                m_edges.push_back(edge);
            }

           private:
            unsigned int m_id;
            std::vector<std::shared_ptr<Edge>> m_edges;
            std::shared_ptr<VertexPayload> m_payload;
        };

        /**!
         * An edge from the undirected graph
         */
        class Edge
        {
           public:
            //! Constructor
            Edge(const std::shared_ptr<Vertex>& a, const std::shared_ptr<Vertex>& b, float cost = 1.0f)
                : m_a(a)
                , m_b(b)
                , m_cost(cost)
            {}

            //! \returns One of vertices at one end of the edge
            [[nodiscard]] inline const std::shared_ptr<Vertex>& nodeA() const
            {
                return m_a;
            }

            //! \returns The vertex at the other end of the edge
            [[nodiscard]] inline const std::shared_ptr<Vertex>& nodeB() const
            {
                return m_b;
            }

            //! \returns Whether \p node is one of the vertices that are part of this edge
            [[nodiscard]] inline bool contains(const std::shared_ptr<Vertex>& node) const
            {
                return node == m_a || node == m_b;
            }

            //! \returns The other node that is part of this edge
            [[nodiscard]] const std::shared_ptr<Vertex>& other(const std::shared_ptr<Vertex>& node) const
            {
                if(node == m_a)
                {
                    return m_b;
                }
                if(node == m_b)
                {
                    return m_a;
                }
                throw createLogicError(fmt::format("Vertex '{0:d}' is not part of this edge", node->id()));
            }

            //! \returns The cost to traverse this edge
            [[nodiscard]] inline float cost() const
            {
                return m_cost;
            }

           private:
            std::shared_ptr<Vertex> m_a;
            std::shared_ptr<Vertex> m_b;
            float m_cost;
        };

        //! Constructor
        UndirectedGraph() = default;

        //! Adds a vertex to the graph
        const std::shared_ptr<Vertex>& addVertex(unsigned int id,
                                                 const std::shared_ptr<VertexPayload>& payload = nullptr)
        {
            if(m_vertices.contains(id))
            {
                throw createLogicError(fmt::format("Vertex with id '{0:d}' already exists", id));
            }
            auto vertex    = std::make_shared<Vertex>(id, payload);
            m_vertices[id] = vertex;
            return m_vertices[id];
        }

        //! \returns A map of vertices in this graph
        [[nodiscard]] inline const robin_hood::unordered_map<unsigned int, std::shared_ptr<Vertex>>& vertices() const
        {
            return m_vertices;
        }

        //! \returns The number of vertices in this graph
        [[nodiscard]] inline unsigned int numVertices() const
        {
            return m_vertices.size();
        }

        //! Adds an edge to the graph
        const std::shared_ptr<Edge>& addEdge(unsigned int a, unsigned int b, float cost = 1.0f)
        {
            if(!m_vertices.contains(a))
            {
                throw createLogicError(fmt::format("Vertex with id '{0:d}' does not exist", a));
            }
            if(!m_vertices.contains(b))
            {
                throw createLogicError(fmt::format("Vertex with id '{0:d}' does not exist", b));
            }

            return addEdge(m_vertices[a], m_vertices[b], cost);
        }

        //! Adds an edge to the graph
        const std::shared_ptr<Edge>& addEdge(const std::shared_ptr<Vertex>& a,
                                             const std::shared_ptr<Vertex>& b,
                                             float cost = 1.0f)
        {
            auto edge = std::make_shared<Edge>(a, b, cost);
            a->addEdge(edge);
            b->addEdge(edge);
            m_edges[std::pair(a->id(), b->id())] = edge;
            return m_edges[std::pair(a->id(), b->id())];
        }

        //! \returns A map of edges in this graph
        [[nodiscard]] inline const robin_hood::unordered_map<std::pair<unsigned int, unsigned int>,
                                                             std::shared_ptr<Edge>>&
        edges() const
        {
            return m_edges;
        }

        //! \returns The number of edges in this graph
        [[nodiscard]] inline unsigned int numEdges() const
        {
            return m_edges.size();
        }

       protected:
        robin_hood::unordered_map<unsigned int, std::shared_ptr<Vertex>> m_vertices;
        robin_hood::unordered_map<std::pair<unsigned int, unsigned int>, std::shared_ptr<Edge>> m_edges;
    };

}  // namespace grstapse