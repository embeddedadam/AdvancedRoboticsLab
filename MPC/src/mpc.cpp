#include <Eigen/Dense>
#include <osqp.h>
#include <iostream>

class MPCController {
public:
    MPCController(int N, double dt, double ref) 
        : N(N), A(2, 2), B(2, 1), ref(ref) 
    {
        A << 1, dt, 0, 1;
        B << 0.5 * dt * dt, dt;
    }

    double control(const Eigen::Vector2d& x0) {
        // Setting up QP matrices
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2*N, 2*N);
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(N, N);
        // Here you can weigh the state and control costs differently
        for(int i = 0; i < N; i++) {
            Q(2*i, 2*i) = 1;
            Q(2*i+1, 2*i+1) = 1;
        }

        Eigen::MatrixXd Aeq = buildEqualityMatrix();
        Eigen::VectorXd leq = Eigen::VectorXd::Zero(2*N);
        leq.head(2) = x0;
        Eigen::VectorXd ueq = leq;

        Eigen::VectorXd l = Eigen::VectorXd::Ones(N) * -1.0;  // constraints on u, lower bound
        Eigen::VectorXd u = Eigen::VectorXd::Ones(N) * 1.0;   // constraints on u, upper bound

        // Setup OSQP data
        OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
        OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

        data->n = N;
        data->m = 2*N;
        data->P = csc_matrix(data->n, data->n, Q.nonZeros(), Q.valuePtr(), Q.outerIndexPtr(), Q.innerIndexPtr());
        data->q = -Q * buildReferenceVector();
        data->A = csc_matrix(data->m, data->n, Aeq.nonZeros(), Aeq.valuePtr(), Aeq.outerIndexPtr(), Aeq.innerIndexPtr());
        data->l = leq.data();
        data->u = ueq.data();

        osqp_setup(&work, data, settings);

        // Solve
        osqp_solve(work);

        double u0 = work->solution->x[0];  // Take the first control action

        // Cleanup
        osqp_cleanup(work);
        c_free(data->A);
        c_free(data->P);
        c_free(data);
        c_free(settings);

        return u0;
    }

private:
    Eigen::MatrixXd buildEqualityMatrix() {
        // This builds the equality constraint matrix
        Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(2*N, N);
        for(int i = 0; i < N; i++) {
            if(i == 0) {
                Aeq.block<2,1>(2*i, i) = B;
            } else {
                Aeq.block<2,1>(2*i, i) = A * Aeq.block<2,1>(2*(i-1), i-1) + B;
            }
        }
        return Aeq;
    }

    Eigen::VectorXd buildReferenceVector() {
        // This builds the reference trajectory
        Eigen::VectorXd r(2*N);
        for(int i = 0; i < N; i++) {
            r(2*i) = ref;
            r(2*i+1) = 0.0;  // Zero velocity reference
        }
        return r;
    }

    int N;  // Horizon
    Eigen::MatrixXd A, B;
    double ref;
    OSQPWorkspace* work;
};