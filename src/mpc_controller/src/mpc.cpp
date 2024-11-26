#include <mpc_controller/mpc.h>

namespace mpc_controller
{
    MPC::MPC()
    {
        t_end = 0;
        steps = 0;

        max_vel = 0;
        max_acc = 0;
        max_delta = 0;
        max_delta_rate = 0;
        L = 0;
        kcte = 0;
        kepsi = 0;
        kev = 0;
        ksteer_cost = 0;
    }

    void MPC::init(int steps,
             double dt,
             double max_vel,
             double max_acc,
             double max_delta,
             double max_delta_rate,
             double L,
             double kcte,
             double kepsi,
             double kv,
             double ksteer_cost)
    {
        this->t_end = steps * dt;
        this->t_end = steps * dt;
        this->steps = steps;
        this->max_vel = max_vel;
        this->max_acc = max_acc;
        this->max_delta = max_delta;
        this->max_delta_rate = max_delta_rate;
        this->L = L;
        this->kcte = kcte;
        this->kepsi = kepsi;
        this->kev = kv;
        this->ksteer_cost = ksteer_cost;

        // Initialization of the differential equation
        f << dot(x) == vel * cos(fi);
        f << dot(y) == vel * sin(fi);
        f << dot(fi) == vel * tan(delta) / L;
        f << dot(delta) == delta_rate;
        f << dot(vel) == acc;
    }

    void MPC::solve(double v0,
                    double delta0,
                    std::vector<double> &traj_coef,
                    double &rate_u,
                    double &acc_u,
                    std::vector<double> &res_x,
                    std::vector<double> &res_y)
    {
        assert(std::abs(delta0) <= max_delta);
        ACADO::OCP ocp(t_start, t_end, steps);
        // constrains
        ocp.subjectTo(f);
        ocp.subjectTo(ACADO::AT_START, x == 0);
        ocp.subjectTo(ACADO::AT_START, y == 0);
        ocp.subjectTo(ACADO::AT_START, fi == 0);
        ocp.subjectTo(-max_acc <= acc <= max_acc);
        ocp.subjectTo(-max_delta_rate <= delta_rate <= max_delta_rate);
        ocp.subjectTo(-max_delta <= delta <= max_delta);
        ocp.subjectTo(ACADO::AT_START, vel == v0);
        ocp.subjectTo(ACADO::AT_START, delta == delta0);

        assert(traj_coef.size() == 4);
        double &a0 = traj_coef[0];
        double &a1 = traj_coef[1];
        double &a2 = traj_coef[2];
        double &a3 = traj_coef[3];

        // minimization objectives
        ACADO::Expression cte = pow(y - a0 - a1 * x - a2 * x * x - a3 * x * x * x, 2);
        ACADO::Expression epsi = pow(fi - atan(a1 + 2 * a2 * x + 3 * a3 * x * x), 2);
        ACADO::Expression verr = pow(vel - max_vel, 2);
        ACADO::Expression steer_cost = pow(delta_rate, 2);

        ocp.minimizeMayerTerm(kcte * cte + kepsi * epsi + kev * verr + ksteer_cost * steer_cost);

        ACADO::OptimizationAlgorithm alg(ocp);
        alg.set(ACADO::PRINTLEVEL, ACADO::NONE);
        alg.solve();
        ACADO::VariablesGrid controls;
        alg.getControls(controls);
        ACADO::VariablesGrid states;
        alg.getDifferentialStates(states);

        rate_u = controls(0, 0);
        acc_u = controls(0, 1);

        res_x.resize(states.getNumPoints());
        res_y.resize(res_x.size());
        for (std::size_t i = 0; i < res_x.size(); ++i)
        {
            res_x[i] = states(i, 0);
            res_y[i] = states(i, 1);
        }
    }

    MPC::~MPC()
    {

    }
}