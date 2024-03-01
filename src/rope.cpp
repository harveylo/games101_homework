#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

// #define EXPLICIT 0

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
        constexpr double k_d_global = 0.0068;
        constexpr double k_d_inner = 0.1;
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto ab = s->m2->position - s->m1->position;
            auto force = s->k * (ab.norm() -s->rest_length);
            auto direction_from_m1_to_m2 = ab.unit();
            auto direction_from_m2_to_m1 = -direction_from_m1_to_m2;
            s->m1->forces += force * direction_from_m1_to_m2;
            s->m2->forces += force * direction_from_m2_to_m1;

            // Damping forces added here
            // Inner Damping forces
            s->m1->forces += -k_d_inner * dot(direction_from_m1_to_m2,s->m1->velocity - s->m2->velocity) * direction_from_m1_to_m2;
            s->m2->forces += -k_d_inner * dot(direction_from_m2_to_m1,s->m2->velocity - s->m1->velocity) * direction_from_m2_to_m1;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                
                // TODO (Part 2): Add global damping
                m->forces += -k_d_global * m->velocity;
                
                auto a = m->forces / m->mass;
                auto delta_v = delta_t * a;

                #ifdef EXPLICIT
                    // ! Explicit Euler method reuqires about 10240 steps per frame to 
                    // ! make the rope stable.
                    m->position += delta_t * m->velocity;
                    m->velocity += delta_v;
                #else 
                    // semi-implicit
                    m->velocity += delta_v;
                    m->position += delta_t * m->velocity;
                #endif

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
            auto ab = s->m2->position - s->m1->position;
            auto force = s->k * (ab.norm() -s->rest_length);
            auto direction_from_m1_to_m2 = ab.unit();
            auto direction_from_m2_to_m1 = -direction_from_m1_to_m2;
            s->m1->forces += force * direction_from_m1_to_m2;
            s->m2->forces += force * direction_from_m2_to_m1;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                auto a = m->forces / m->mass + gravity;

                auto delta_position =  (m->position - m->last_position) +  a * delta_t * delta_t;
                
                // TODO (Part 4): Add global Verlet damping
                constexpr double global_verlet_damping = 0.00010;

                m->position = m->position + (1-global_verlet_damping) * delta_position;

                m->last_position = temp_position;
            }
            // ! Important, don't forget to reset the forces
            m->forces = Vector2D(0, 0);
        }
    }
}
