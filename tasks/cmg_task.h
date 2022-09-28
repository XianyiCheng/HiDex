
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../mechanics/utilities/utilities.h"
#include "../mechanics/utilities/sample.h"

void enumerate_cs_modes(const ContactConstraints &cons, const std::vector<ContactPoint> &envs, std::vector<VectorXi> *modes)
{
    // contacting-separating mode enumeration
    MatrixXd A;
    VectorXd b;
    MatrixXd D;
    VectorXd d;
    cons.NormalVelocityConstraints(envs, &A, &b);
    cons.TangentVelocityConstraints(envs, &D, &d);

    if (envs.size() == 0)
    {
        VectorXi m(0);
        modes->push_back(m);
    }
    else
    {
        cs_mode_enumeration(A, modes);
    }
}

void enumerate_ss_modes(const ContactConstraints &cons, const std::vector<ContactPoint> &envs, const VectorXi &cs_mode, std::vector<VectorXi> *ss_modes)
{

    MatrixXd A;
    VectorXd b;
    MatrixXd D;
    VectorXd d;
    cons.NormalVelocityConstraints(envs, &A, &b);
    cons.TangentVelocityConstraints(envs, &D, &d);
    ss_mode_enumeration(A, D, cs_mode, ss_modes);
}

class RRTTree
{
public:
    struct Node
    {
        Vector7d config; // x, y, z, qx, qy, qz, qw
        int parent = -1;
        int edge = -1; // index of edge from parent to this
        bool is_extended_to_goal = false;
        std::vector<ContactPoint> envs;
        std::vector<VectorXi> modes;
        Node(Vector7d data)
        {
            config = data;
            manipulator_config = mnp_config;
        }
    };

    struct Edge
    {
        VectorXi mode;
        Edge(VectorXi m) : mode(m) {}
    };

    std::vector<Node> nodes;
    std::vector<Edge> edges;
    double angle_weight;
    double translation_weight;
    Tree(const double &a_weight, const double &p_weight) : angle_weight(a_weight), translation_weight(p_weight) {}
    double dist(const Vector7d &q1, const Vector7d &q2)
    {
        Vector3d p1(q1[0], q1[1], q1[2]);
        Vector3d p2(q2[0], q2[1], q2[2]);
        Quaterniond quat1(q1[6], q1[3], q1[4], q1[5]); // Quaterniond: w, x, y, z
        Quaterniond quat2(q2[6], q2[3], q2[4], q2[5]);
        Vector3d dp = p1 - p2;
        double dtrans = dp.norm();
        double angle = angBTquat(quat1, quat2);

        double d = this->translation_weight * dtrans + this->angle_weight * angle;
        return d;
    }

    int nearest_neighbor(const Vector7d &q)
    {
        int near_idx;
        double min_d = DBL_MAX;
        for (int i = 0; i < nodes.size(); i++)
        {
            double d = this->dist(nodes[i].config, q);
            if (d < min_d)
            {
                near_idx = i;
                min_d = d;
            }
        }
        return near_idx;
    }
    int nearest_unextended_to_goal(const Vector7d &q)
    {
        int near_idx;
        double min_d = DBL_MAX;
        for (int i = 0; i < nodes.size(); i++)
        {
            double d = this->dist(nodes[i].config, q);
            if ((d < min_d) && (!nodes[i].is_extended_to_goal))
            {
                near_idx = i;
                min_d = d;
            }
        }
        if (min_d == DBL_MAX)
        {
            return -1;
        }

        return near_idx;
    }
    void backtrack(int last_node_idx, std::vector<int> *node_path)
    {
        int cur_node = last_node_idx;
        node_path->push_back(cur_node);
        bool is_start_node = nodes[cur_node].parent != -1;
        while (is_start_node)
        {
            cur_node = nodes[cur_node].parent;
            node_path->push_back(cur_node);
            is_start_node = nodes[cur_node].parent != -1;
        }
        return;
    }
    void add_node(Node *n, int parent, Edge *e)
    {
        int node_idx = nodes.size();
        int edge_idx = edges.size();
        n->parent = parent_idx;
        n->edge = edge_idx;

        nodes.push_back(*n);
        edges.push_back(*e);
        return;
    }

    void initial_node(Node *n)
    {
        n->parent = -1;
        nodes.push_back(*n);
        return;
    }
};

class CMGTASK
{

public:
    struct State
    {
        Vector7d m_pose;
        std::vector<ContactPoint> envs;
        int m_mode_idx = -1; // the mode chosen for this state, to the next state
        std::vector<Eigen::VectorXi> modes;

        State() {}

        State(Vector7d pose, const std::vector<ContactPoint> &envs, int mode_idx, const std::vector<Eigen::VectorXi> &modes_) : m_pose(pose), envs(envs_), m_mode_idx(mode_idx), modes(modes_) {}

        State(const State &state_)
        {
            // copy constructor
            m_pose = state_.m_pose;
            m_mode_idx = state_.m_mode_idx;
            modes = state_.modes;
        }

        void do_action(int action) { m_mode_idx = action; }

        State &operator=(const State &state_)
        {
            this->m_pose = state_.m_pose;
            this->m_mode_idx = state_.m_mode_idx;
            this->modes = state_.modes;
            return *this;
        }
    };

    struct SearchOptions
    {
        // the search options for search_a_new_path using RRT
        Eigen::Vector3d x_lb;
        Eigen::Vector3d x_ub;
        double eps_translation; // the maximum translation in a RRT extend
        double eps_angle;       // the maximum rotation in a RRT extend
        int sampleSO3 = true;
        double goal_biased_prob = 0.8;
        int max_samples = 100;
        Vector3d sample_rotation_axis;

        SearchOptions() {}

        SearchOptions(const SearchOptions &opts)
        {
            this->x_lb = opts.x_lb;
            this->x_ub = opts.x_ub;
            this->sampleSO3 = opts.sampleSO3;
            this->goal_biased_prob = opts.goal_biased_prob;
            this->max_samples = opts.max_samples;
            if (!opts.sampleSO3)
                this->sample_rotation_axis = opts.sample_rotation_axis;
        }
        SearchOptions &operator=(const SearchOptions &opts)
        {
            this->x_lb = opts.x_lb;
            this->x_ub = opts.x_ub;
            this->sampleSO3 = opts.sampleSO3;
            this->goal_biased_prob = opts.goal_biased_prob;
            this->max_samples = opts.max_samples;
            if (!opts.sampleSO3)
                this->sample_rotation_axis = opts.sample_rotation_axis;
            return *this;
        }
    };

    CMGTASK()
    {
        this->cons = std::make_unique<ContactConstraints>(2);
    }

    void initialize(const Vector7d &start_object_pose, const Vector7d &goal_object_pose, double goal_thr,
                    double wa, double wt, double charac_len double mu_env, double mu_mnp,
                    Matrix6d object_inertia,
                    const SearchOptions &options)
    {
        this->start_object_pose = start_object_pose;
        this->goal_object_pose = goal_object_pose;
        this->goal_thr = goal_thr;
        this->wa = wa;
        this->wt = wt;
        this->charac_len = charac_len;
        this->mu_env = mu_env;
        this->mu_mnp = mu_mnp;
        this->search_options = options;
        this->m_initialized = true;
    }

    State get_start_state() const { return generate_state(start_object_pose); }

    State generate_state(const Vector7d &object_pose) const
    {
        State state_;
        state_.m_pose = object_pose;
        this->m_world->getObjectContacts(&state_.envs, object_pose);
        enumerate_cs_modes(*this->cons.get(), state_.envs, &state_.modes);

        return state_;
    }

    std::vector<State> search_a_new_path(const State &start_state)
    {
        // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

        // during the search, it figure out the constriants (modes) for the states
        // for now just return linear interpolation towards the sampled state

        // every state need to be associated with m_mode_idx (mode to this state)

        // int action_idx = start_state.m_mode_idx;

        // initialize a tree
        RRTTree rrt_tree(this->wa, this->charac_len * this->wt);
        RRTTree::Node start_node(start_state.m_pose);
        start_node.modes.push_back(start_state.modes[start_state.m_mode_idx])
        start_node.envs = state_state.envs;
        rrt_tree.initial_node(start_node);

        set_rand_seed();

        int goal_idx = -1;

        // -------

        for (int kk = 0; kk < this->search_options->max_samples; kk++)
        {
            std::cout << "rrt iter: " << kk << std::endl;

            // bias sample toward the goal
            Vector7d x_rand;
            int near_idx;
            if ((randd() > this->search_options.goal_biased_prob) && (kk >= 1))
            {
                Vector3d p_rand;
                Quaterniond q_rand;
                p_rand = sample_position(this->search_options.x_ub, this->search_options.x_lb);

                q_rand = (this->search_options.sampleSO3) ? generate_unit_quaternion() | sample_rotation(this->search_options.sample_rotation_axis);

                x_rand << p_rand[0], p_rand[1], p_rand[2],
                    q_rand.x(), q_rand.y(), q_rand.z(), q_rand.w();
                near_idx = rrt_tree.nearest_neighbor(x_rand);
            }
            else
            {
                x_rand = this->goal_object_pose;
                near_idx = rrt_tree.nearest_unextended_to_goal(this->goal_object_pose);
                if (near_idx != 0)
                {
                    rrt_tree.nodes[near_idx].is_extended_to_goal = true;
                }
            }

            // -----------------------------------
            // extend all cs modes, best ss modes

            // steer goal

            // move
            x_rand = steer_config(rrt_tree.nodes[near_idx].config, x_rand, this->eps_trans, this->eps_angle);

            // move
            Vector6d v_star = compute_rbvel_body(rrt_tree.nodes[near_idx].config, x_rand);

            // move
            Vector6d f_o = weight_w2o(rrt_tree.nodes[near_idx].config, this->f_w);

            // contact mode enumeration
            if (rrt_tree.nodes[near_idx].modes.size() == 0)
            {
                this->m_world->getObjectContacts(&(rrt_tree.nodes[near_idx].envs), rrt_tree.nodes[near_idx].config);
                enumerate_cs_modes(*this->cons.get(), rrt_tree.nodes[near_idx].envs, &rrt_tree.nodes[near_idx].modes)
            }

            // for every mode do forward integration
            Vector6d v_zero = Vector6d::Zero();
            // move
            double d_zero = dist_vel(v_zero, v_star, rrt_tree.translation_weight, rrt_tree.angle_weight);
            for (const auto &cs_mode : rrt_tree.nodes[near_idx].modes)
            {

                std::cout << "cs mode " << cs_mode.transpose() << std::endl;

                std::vector<VectorXi> modes;

                enumerate_ss_modes(*this->cons.get(), rrt_tree.nodes[near_idx].envs, cs_mode, &modes);

                double dv_best = d_zero;

                VectorXi mode_best;

                std::vector<VectorXi> mode_to_extend;
                {
                    VectorXi all_sticking_mode(3 * cs_mode.size());
                    all_sticking_mode.setZero();
                    all_sticking_mode.block(0, 0, cs_mode.size(), 1) = cs_mode;
                    // move
                    Vector6d v = this->pw->EnvironmentConstrainedVelocity(v_star, rrt_tree.nodes[near_idx].envs, all_sticking_mode);
                    
                    if (dist_vel(v, v_star, rrt_tree.translation_weight, rrt_tree.angle_weight) < d_zero)
                    {
                        mode_to_extend.push_back(all_sticking_mode);
                    }
                }
                for (const auto &mode : modes)
                {
                    // move
                    Vector6d v = this->pw->EnvironmentConstrainedVelocity(v_star, rrt_tree.nodes[near_idx].envs, mode);
                    // std::cout << "mode: " << mode.transpose() <<  " velocity: " << v.transpose() << std::endl;
                    if (dist_vel(v, v_star, rrt_tree.translation_weight, rrt_tree.angle_weight) < dv_best)
                    {
                        dv_best = dist_vel(v, v_star, rrt_tree.translation_weight, rrt_tree.angle_weight);
                        mode_best = mode;
                    }
                }
                std::cout << "best mode " << mode_best.transpose() << std::endl;

                if (dv_best < d_zero - 1e-4)
                {
                    mode_to_extend.push_back(mode_best);
                }

                /// choose sliding mode end

                for (const auto &mode : mode_to_extend)
                {

                    std::cout << "Extend mode: " << mode.transpose() << std::endl;
                                    
                    std::vector<Vector7d> path;

                    // move
                    ifmanipulatorcollide = this->pw->ForwardIntegration(
                        rrt_tree.nodes[near_idx].config, x_rand, mnp_config, rrt_tree.nodes[near_idx].envs, mode, this->f_w,
                        rrt_tree.translation_weight, rrt_tree.angle_weight, &path);

                    // if integration is successful
                    if (path.size() > 1 )
                    {

                        Node new_node(path.back());
                        Edge new_edge(mode);

                        rrt_tree.add_node(&new_node, near_idx, &new_edge);
                    }
                }
            }
            //----------------------------------

            int goal_near_idx = rrt_tree.nearest_neighbor(x_goal);
            if (rrt_tree.dist(rrt_tree.nodes[goal_near_idx].config, x_goal) <= goal_thr)
            {
                printf("Found goal node in %d samples. \n", kk + 1);
                std::cout << rrt_tree.nodes[goal_near_idx].manipulator_config.transpose() << std::endl;
                goal_idx = goal_near_idx;
                break;
            }
        }

        if (goal_idx != -1)
        {
            ifsuccess = true;
            printf("GOAL REACHED! \n");
        }
        else
        {
            ifsuccess = false;
            goal_idx = rrt_tree.nearest_neighbor(x_goal);
            std::cout << "GOAL NOT REACHED. Dist: " << rrt_tree.dist(rrt_tree.nodes[goal_idx].config, x_goal)
                      << ". Closest config: " << rrt_tree.nodes[goal_idx].config.transpose() << std::endl;
        }

        /// end of search

        std::vector<State> path_;

        if (ifsuccess)
        {
            std::vector<int> node_path;
            rrt_tree.backtrack(goal_idx, &node_path);
            std::reverse(node_path.begin(), node_path.end());
            for (int k = 1; k < node_path.size() - 1; k++)
            {
                int kn = node_path[k];
                VectorXi mode = rrt_tree.edges[rrt_tree.nodes[kn + 1].edge].mode;
                int mode_idx = -1;
                for (int idx = 0; idx < rrt_tree.nodes[kn].modes.size(); ++idx)
                {
                    if ((mode - rrt_tree.nodes[kn].modes[idx]).norm() == 0)
                    {
                        mode_idx = idx;
                        break;
                    }
                }
                State new_state(rrt_tree.nodes[kn].config, rrt_tree.nodes[kn].envs, mode_idx, rrt_tree.nodes[kn].modes);
                path_.push_back(new_state);
            }
            State new_state(rrt_tree.nodes[node_path.back()].config, rrt_tree.nodes[node_path.back()].envs, -1, rrt_tree.nodes[node_path.back()].modes);
            path_.push_back(new_state);
        }

        return path_;
    }

    double evaluate_path(const std::vector<State> &path)
    {
        // return the REWARD of the path: larger reward -> better path

        // TODO: define reward
        double reward = 1 / double(path.size());

        return reward;
    }

private:
    bool m_initialized = false;
    Vector7d start_object_pose;
    Vector7d goal_object_pose;
    double goal_thr;
    double wa; // weigh the importance of angle
    double wt; // weigh the importance of translation

    double charac_len; // characteristic length

    double mu_mnp = 0.8;
    double mu_env = 0.5;

    Matrix6d object_inertia;

    std::vector<ContactPoint> object_surface_pts;

    std::shared_ptr<WorldTemplate> m_world; // save the object, environment, do collision detections, ...

    SearchOptions search_options;

    std::unique_ptr<ContactConstraints> cons;
};