//
// Auto-generated file by OptimizationEngine
// See https://alphaville.github.io/optimization-engine/
//
//


use optimization_engine::{constraints::*, panoc::*, alm::*, *};

// ---Private Constants----------------------------------------------------------------------------------

/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = 0.0001;

/// Initial tolerance
const INITIAL_EPSILON_TOLERANCE: f64 = 0.0001;

/// Update factor for inner tolerance
const EPSILON_TOLERANCE_UPDATE_FACTOR: f64 = 0.1;

/// Delta tolerance
const DELTA_TOLERANCE: f64 = 0.0001;

/// LBFGS memory
const LBFGS_MEMORY: usize = 10;

/// Maximum number of iterations of the inner solver
const MAX_INNER_ITERATIONS: usize = 500;

/// Maximum number of outer iterations
const MAX_OUTER_ITERATIONS: usize = 10;

/// Maximum execution duration in microseconds
const MAX_DURATION_MICROS: u64 = 5000000;

/// Penalty update factor
const PENALTY_UPDATE_FACTOR: f64 = 5.0;

/// Initial penalty
const INITIAL_PENALTY_PARAMETER: Option<f64> = None;

/// Sufficient decrease coefficient
const SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT: f64 = 0.1;

/// Whether preconditioning should be applied
const DO_PRECONDITIONING: bool = false;

// ---Public Constants-----------------------------------------------------------------------------------

/// Number of decision variables
pub const CONTROLLER_SOLVER_NUM_DECISION_VARIABLES: usize = 65;

/// Number of parameters
pub const CONTROLLER_SOLVER_NUM_PARAMETERS: usize = 68;

/// Number of parameters associated with augmented Lagrangian
pub const CONTROLLER_SOLVER_N1: usize = 0;

/// Number of penalty constraints
pub const CONTROLLER_SOLVER_N2: usize = 0;



// ---Parameters of the constraints----------------------------------------------------------------------

const CONSTRAINTS_XMIN :Option<&[f64]> = Some(&[-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,-270.0,]);
const CONSTRAINTS_XMAX :Option<&[f64]> = Some(&[370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,370.0,]);










// ---Internal private helper functions------------------------------------------------------------------

/// Make constraints U
fn make_constraints() -> impl Constraint {
    // - Rectangle:
    Rectangle::new(CONSTRAINTS_XMIN, CONSTRAINTS_XMAX)
    }





// ---Main public API functions--------------------------------------------------------------------------


/// Initialisation of the solver
pub fn initialize_solver() -> AlmCache {
    let panoc_cache = PANOCCache::new(CONTROLLER_SOLVER_NUM_DECISION_VARIABLES, EPSILON_TOLERANCE, LBFGS_MEMORY);
    AlmCache::new(panoc_cache, CONTROLLER_SOLVER_N1, CONTROLLER_SOLVER_N2)
}

/// If preconditioning has been applied, then at the end (after a solution has been obtained)
/// we need to undo the scaling and update the cost function
fn unscale_result(solver_status: &mut Result<AlmOptimizerStatus, SolverError>) {
    if let Ok(sss) = solver_status {
        let w_cost : f64 = icasadi_controller_solver::get_w_cost();
        sss.update_cost(sss.cost() / w_cost);
    }
}

/// Solver interface
///
/// ## Arguments
/// - `p`: static parameter vector of the optimization problem
/// - `alm_cache`: Instance of AlmCache
/// - `u`: Initial guess
/// - `y0` (optional) initial vector of Lagrange multipliers
/// - `c0` (optional) initial penalty
///
/// ## Returns
/// This function returns either an instance of AlmOptimizerStatus with information about the
/// solution, or a SolverError object if something goes wrong
pub fn solve(
    p: &[f64],
    alm_cache: &mut AlmCache,
    u: &mut [f64],
    y0: &Option<Vec<f64>>,
    c0: &Option<f64>,
) -> Result<AlmOptimizerStatus, SolverError> {

    assert_eq!(p.len(), CONTROLLER_SOLVER_NUM_PARAMETERS, "Wrong number of parameters (p)");
    assert_eq!(u.len(), CONTROLLER_SOLVER_NUM_DECISION_VARIABLES, "Wrong number of decision variables (u)");

    // Start by initialising the optimiser interface (e.g., set w=1)
    icasadi_controller_solver::init_controller_solver();

    let mut rho_init : f64 = 1.0;
    if DO_PRECONDITIONING {
        // Compute the preconditioning parameters (w's)
        // The scaling parameters will be stored internally in `interface.c`
        icasadi_controller_solver::precondition(u, p);

        // Compute initial penalty
        icasadi_controller_solver::initial_penalty(u, p, & mut rho_init);
    }

    let psi = |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi_controller_solver::cost(u, xi, p, cost);
        Ok(())
    };
    let grad_psi = |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        icasadi_controller_solver::grad(u, xi, p, grad);
        Ok(())
    };
    
    let bounds = make_constraints();

    let alm_problem = AlmProblem::new(
        bounds,
        NO_SET,
        NO_SET,
        psi,
        grad_psi,
        NO_MAPPING,
        NO_MAPPING,
        CONTROLLER_SOLVER_N1,
        CONTROLLER_SOLVER_N2,
    );

    let mut alm_optimizer = AlmOptimizer::new(alm_cache, alm_problem)
        .with_delta_tolerance(DELTA_TOLERANCE)
        .with_epsilon_tolerance(EPSILON_TOLERANCE)
        .with_initial_inner_tolerance(INITIAL_EPSILON_TOLERANCE)
        .with_inner_tolerance_update_factor(EPSILON_TOLERANCE_UPDATE_FACTOR)
        .with_max_duration(std::time::Duration::from_micros(MAX_DURATION_MICROS))
        .with_max_outer_iterations(MAX_OUTER_ITERATIONS)
        .with_max_inner_iterations(MAX_INNER_ITERATIONS)
        .with_initial_penalty(c0.unwrap_or(INITIAL_PENALTY_PARAMETER.unwrap_or(rho_init)))
        .with_penalty_update_factor(PENALTY_UPDATE_FACTOR)
        .with_sufficient_decrease_coefficient(SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT);

    // solve the problem using `u`, the initial condition `u`, and
    // initial vector of Lagrange multipliers, if provided;
    // returns the problem status (instance of `AlmOptimizerStatus`)
    if let Some(y0_) = y0 {
        let mut alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(y0_);
        let mut solution_status = alm_optimizer.solve(u);
        unscale_result(&mut solution_status);
        solution_status
    } else {
        let mut solution_status = alm_optimizer.solve(u);
        unscale_result(&mut solution_status);
        solution_status
    }

}