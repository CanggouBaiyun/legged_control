#include "go2_control_cpp/wbc.hpp"
#include <osqp.h>
#include <Eigen/SparseCore>
#include <dlfcn.h>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace go2 {
struct OsqpSolver::Impl {
  std::shared_ptr<void> library;
  template<class T> T symbol(const char* name) {
    auto value=dlsym(library.get(),name);
    if (!value) throw std::runtime_error(std::string("Missing OSQP symbol: ")+name);
    return reinterpret_cast<T>(value);
  }
  decltype(&osqp_setup) setup;
  decltype(&osqp_cleanup) cleanup;
  decltype(&osqp_solve) run;
  decltype(&osqp_set_default_settings) defaults;
  decltype(&osqp_update_data_mat) update_matrix;
  decltype(&osqp_update_data_vec) update_vectors;
  decltype(&osqp_update_rho) update_rho;
  decltype(&OSQPCscMatrix_set_data) csc;
  decltype(&osqp_version) get_version;
  OSQPSolver* solver=nullptr;
  std::size_t setups=0, updates=0;
  std::vector<OSQPInt> p_outer,p_inner,a_outer,a_inner;
  Eigen::ArrayXX<bool> p_pattern,a_pattern;
  Eigen::Index previous_n=-1, previous_m=-1;
  OSQPFloat initial_rho=0;
  explicit Impl(const std::string& path)
      : library(dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL),[](void* p){ if(p) dlclose(p); }) {
    static_assert(std::is_same_v<OSQPFloat,double>,"WBC requires double-precision OSQP");
    if (!library) throw std::runtime_error(std::string("Cannot load OSQP: ")+dlerror());
    setup=symbol<decltype(setup)>("osqp_setup");
    cleanup=symbol<decltype(cleanup)>("osqp_cleanup");
    run=symbol<decltype(run)>("osqp_solve");
    defaults=symbol<decltype(defaults)>("osqp_set_default_settings");
    update_matrix=symbol<decltype(update_matrix)>("osqp_update_data_mat");
    update_vectors=symbol<decltype(update_vectors)>("osqp_update_data_vec");
    update_rho=symbol<decltype(update_rho)>("osqp_update_rho");
    csc=symbol<decltype(csc)>("OSQPCscMatrix_set_data");
    get_version=symbol<decltype(get_version)>("osqp_version");
    if (std::string(get_version()).rfind("1.",0)!=0)
      throw std::runtime_error("Expected OSQP C API 1.x with matching headers");
  }
  ~Impl() { if(solver) cleanup(solver); }
  void clear() { if(solver) cleanup(solver); solver=nullptr; }
};
OsqpSolver::OsqpSolver(const std::string& library):impl_(std::make_unique<Impl>(library)){}
OsqpSolver::~OsqpSolver()=default;
std::size_t OsqpSolver::setupCount() const { return impl_->setups; }
std::size_t OsqpSolver::updateCount() const { return impl_->updates; }
std::string OsqpSolver::version() const { return impl_->get_version(); }

QpSolution OsqpSolver::solve(const WbcQp& qp) {
  const Eigen::Index n=qp.g.size(), ne=qp.equality_target.size(), ni=qp.lower_bound.size(), m=ne+ni;
  if (!n || qp.H.rows()!=n || qp.H.cols()!=n || qp.equality_matrix.rows()!=ne ||
      qp.equality_matrix.cols()!=n || qp.inequality_matrix.rows()!=ni || qp.inequality_matrix.cols()!=n ||
      qp.upper_bound.size()!=ni || !qp.H.allFinite() || !qp.g.allFinite() ||
      !qp.equality_matrix.allFinite() || !qp.equality_target.allFinite() ||
      !qp.inequality_matrix.allFinite() || (qp.lower_bound.array().isNaN()).any() ||
      (qp.upper_bound.array().isNaN()).any() || (qp.lower_bound.array()>qp.upper_bound.array()).any())
    throw std::runtime_error("Invalid QP data");
  Eigen::MatrixXd dense_a(m,n);
  dense_a << qp.equality_matrix,qp.inequality_matrix;
  using SparseMatrix=Eigen::SparseMatrix<double,Eigen::ColMajor,OSQPInt>;
  using Triplet=Eigen::Triplet<double,OSQPInt>;
  std::vector<Triplet> p_entries,a_entries;
  p_entries.reserve(n*(n+1)/2);
  a_entries.reserve(m*n);
  auto& impl=*impl_;
  if(impl.p_pattern.rows()!=n || impl.a_pattern.rows()!=m || impl.a_pattern.cols()!=n) {
    impl.p_pattern=Eigen::ArrayXX<bool>::Constant(n,n,false);
    impl.a_pattern=Eigen::ArrayXX<bool>::Constant(m,n,false);
  }
  // Keep the union of observed positions. Never remove a position when its
  // value becomes zero; expand once when a new nonzero is first encountered.
  // This avoids both numerical-zero churn and dense symbolic factorization.
  for(Eigen::Index col=0;col<n;++col) {
    for(Eigen::Index row=0;row<=col;++row) {
      if(qp.H(row,col)!=0) impl.p_pattern(row,col)=true;
      if(impl.p_pattern(row,col))
        p_entries.emplace_back(row,col,qp.H(row,col));
    }
    for(Eigen::Index row=0;row<m;++row) {
      if(dense_a(row,col)!=0) impl.a_pattern(row,col)=true;
      if(impl.a_pattern(row,col))
        a_entries.emplace_back(row,col,dense_a(row,col));
    }
  }
  SparseMatrix P(n,n),A(m,n);
  P.setFromTriplets(p_entries.begin(),p_entries.end());
  A.setFromTriplets(a_entries.begin(),a_entries.end());
  P.makeCompressed();
  A.makeCompressed();
  const std::vector<OSQPInt> po(P.outerIndexPtr(),P.outerIndexPtr()+n+1), pi(P.innerIndexPtr(),P.innerIndexPtr()+P.nonZeros());
  const std::vector<OSQPInt> ao(A.outerIndexPtr(),A.outerIndexPtr()+n+1), ai(A.innerIndexPtr(),A.innerIndexPtr()+A.nonZeros());
  Eigen::VectorXd lower(m),upper(m);
  lower << qp.equality_target,qp.lower_bound;
  upper << qp.equality_target,qp.upper_bound;
  lower=lower.cwiseMax(-OSQP_INFTY); upper=upper.cwiseMin(OSQP_INFTY);
  if (!impl.solver || n!=impl.previous_n || m!=impl.previous_m ||
      po!=impl.p_outer || pi!=impl.p_inner || ao!=impl.a_outer || ai!=impl.a_inner) {
    impl.clear();
    OSQPSettings settings;
    impl.defaults(&settings);
    settings.verbose=0; settings.polishing=1; settings.max_iter=20000; settings.warm_starting=0;
    // Default relative tolerances can accept force-bound errors above the
    // controller's absolute 1e-3 constraint gate, especially if polishing fails.
    settings.eps_abs=1e-6; settings.eps_rel=1e-6;
    impl.initial_rho=settings.rho;
    OSQPCscMatrix pc{},ac{};
    impl.csc(&pc,n,n,P.nonZeros(),P.valuePtr(),P.innerIndexPtr(),P.outerIndexPtr());
    impl.csc(&ac,m,n,A.nonZeros(),A.valuePtr(),A.innerIndexPtr(),A.outerIndexPtr());
    if (impl.setup(&impl.solver,&pc,qp.g.data(),&ac,lower.data(),upper.data(),m,n,&settings)!=0) {
      impl.clear(); throw std::runtime_error("OSQP setup failed");
    }
    impl.previous_n=n; impl.previous_m=m;
    impl.p_outer=po; impl.p_inner=pi; impl.a_outer=ao; impl.a_inner=ai;
    ++impl.setups;
  } else {
    // Matrix updates recompute scaling using the current cost/bounds, so load
    // the new vectors first (the same ordering as OSQP's Python interface).
    if (impl.update_vectors(impl.solver,qp.g.data(),lower.data(),upper.data())!=0 ||
        impl.update_matrix(impl.solver,P.valuePtr(),nullptr,P.nonZeros(),A.valuePtr(),nullptr,A.nonZeros())!=0 ||
        // Cold-start x/y is insufficient: adaptive rho also persists across
        // solves. Restore its initial value to match a fresh cold solve.
        impl.update_rho(impl.solver,impl.initial_rho)!=0) {
      impl.clear(); throw std::runtime_error("OSQP data update failed");
    }
    ++impl.updates;
  }
  if (impl.run(impl.solver)!=0 || !impl.solver->info || impl.solver->info->status_val!=OSQP_SOLVED ||
      !impl.solver->solution || !impl.solver->solution->x)
    throw std::runtime_error("OSQP solve failed or did not reach solved status");
  QpSolution result;
  result.x=Eigen::Map<Eigen::VectorXd>(impl.solver->solution->x,n);
  result.status=impl.solver->info->status;
  result.iterations=impl.solver->info->iter;
  result.primal_residual=impl.solver->info->prim_res;
  result.dual_residual=impl.solver->info->dual_res;
  if (!result.x.allFinite()) throw std::runtime_error("Non-finite OSQP solution");
  return result;
}
}  // namespace go2
