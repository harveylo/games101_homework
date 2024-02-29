#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    : masses(), springs()
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        auto start_mass = new Mass(start, node_mass, false);
        auto end_mass = new Mass(end, node_mass, false);
        
        auto step = (end - start) / (num_nodes - 1);

        masses.emplace_back(start_mass);
        for (int i = 0; i < num_nodes-2; i++) {
            auto position = start + step * (i+1);
            auto mass = new Mass(position, node_mass, false);
            auto spring = new Spring(masses[i],mass,k);
            masses.emplace_back(mass);
            springs.emplace_back(spring);
        }
        springs.emplace_back(new Spring(*(masses.end()-1),end_mass,k));
        masses.emplace_back(end_mass);

    //    Comment-in this part when you implement the constructor
        // pinned_nodes indicates the nodes that should be pinned in place
        // which means they are not movable
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto force = s->k * ((s->m2->position - s->m1->position).norm() -s->rest_length);
            s->m1->forces = force * (s->m2->position - s->m1->position).unit();
            s->m2->forces = -s->m1->forces;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                auto a = m->forces / m->mass;
                auto v = m->velocity + a * delta_t;
                m->position += v * delta_t;

                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
