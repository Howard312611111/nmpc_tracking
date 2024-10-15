#include <iostream>
#include <vector>
#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

int main()
{
    cout << "casadi_test" << endl;

    // This is another way to define a nonlinear solver. Opti is new

    /*
    *       min x1*x4*(x1 + x2 + x3) + x3
    *       s.t. x1*x2*x3*x4 >= 25
    *            x1^2 + x2^2 + x3^2 + x4^2 = 40
    *            1 <= x1, x2, x3, x4 <= 5
    */

    // Optimization variables
    SX x = SX::sym("x", 4);
    std::cout << "x:" << x << std::endl;

    // Objective
    SX f = x(0)*x(3)*(x(0) + x(1) + x(2)) + x(2);
    // SX f = x(0)*x(3)*(x(0) + x(1) + x(2)) + x(2);
    std::cout << "f:" << f << std::endl;

    // Constraints
    // SX g = vertcat(6 - x(0)*3 + x(1) * 2 * x(2) - p(0), p(1) * x(0) + x(1) - x(2) - 1);
    SX g = vertcat(x(0)*x(1)*x(2)*x(3), pow(x(0),2) + pow(x(1),2) + pow(x(2),2) + pow(x(3),2));
    std::cout << "g:" << g << std::endl;

    // Initial guess and bounds for the optimization variables
    vector<double> x0 = {0.0, 0.0, 0.0, 0.0};
    vector<double> lbx = {1, 1, 1, 1};
    vector<double> ubx = {5, 5, 5, 5};

    // Nonlinear bounds
    vector<double> lbg = {25, 40};
    vector<double> ubg = {inf, 40};

    // NLP
    SXDict nlp = {{"x", x}, {"f", f}, {"g", g}};

    // Create NLP solver and buffers
    Function solver = nlpsol("solver", "ipopt", nlp);
    std::map<std::string, DM> arg, res;

    // Solve the NLP
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = x0;
    res = solver(arg);

    // Print the solution
    cout << "--------------------------------" << endl;
    // std::cout << res << std::endl;
    cout << "objective: " << res.at("f") << endl;
    cout << "solution: " << res.at("x") << endl;

    return 0;
}