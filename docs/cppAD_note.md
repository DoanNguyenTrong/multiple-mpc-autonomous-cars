



## Simple vector
- std::vector
- std::valarray
- cppAD::vector

1. Default constructor `SimpleVector<Scalar> x;` - empty, `x.size()` = 0
1. Sizing constructor `size_t n;` -> `SimpleVector<Scalar> x (n);`
1. Copy constructor `SimpleVector<Scalar> y(x);` - **Deep** copy, `x` is passed by reference, can be `const`
1. Assignement `y = x` - **Deep** assignment.
1. Size `size_t n = size_t (x.size())`
1. Resize `x.resize(n)` - previously stored data in `x` are lost
1. value_type `Vector::value_type` // `SimpleVector<Scalar>::value_type = Scalar`
1. Conversion operation `static_cast<Scalar> (x[i])` - convert `elementType` to `Scalar`


## cppAD Test Vector Template Class

1. Syntax `CPPAD_TESTVECTOR(Scalar)`
2. Choice **during the install procedure**, which template class to use (specific simple vector prefered - e.g. `CppAD::vector <Type>`) to form `CPPAD_TESTVECTOR(Type)`
3. `CppAD::vector` - cmake command specify `cppad_testvector` to be cppad, `CPPAD_CPPADVECTOR` - True
```c
# if CPPAD_CPPADVECTOR
# define CPPAD_TESTVECTOR(Scalar) CppAD::vector< Scalar >
# endif
```
4. `std::vector` - cmake command specify `cppad_testvector` to be std, `CPPAD_STDVECTOR` - True
```c
# if CPPAD_STDVECTOR
# include <vector>
# define CPPAD_TESTVECTOR(Scalar) std::vector< Scalar >
# endif
```
5. `boost::numeric::ublast::vector` - cmake command specify `cppad_testvector` to be boost, `CPPAD_BOOSTVECTOR` - True
```c
# if CPPAD_BOOSTVECTOR
# include <boost/numeric/ublas/vector.hpp>
# define CPPAD_TESTVECTOR(Scalar) boost::numeric::ublas::vector< Scalar >
# endif
```
6. `CppAD::eigen_vector` - cmake command specify `cppad_testvector` to be eigen, `CPPAD_EIGENVECTOR` - True
```c
# if CPPAD_EIGENVECTOR
# include <cppad/example/cppad_eigen.hpp>
# define CPPAD_TESTVECTOR(Scalar) CppAD::eigen_vector< Scalar >
# endif
```

## Poly
Evaluate a polynomial or its derivative

1. Syntax
```c
#include <cppad/utility/poly.hpp>
p = Poly(k, a , z)
```
2. Description - compute k-th derivative of the polynomial $P(z) = a_0 + a_1 z^1 + ... + a_d z^d$ \
   If $k=0$ -> return $P(z)$
3. Include `cppad/utility/poly.hpp` is included by `cppad/cppad.h`, but it can also be included separately
4. `size_t k` - the order of the derivative to calculate
5. `const Vector &a` - argument of the polynominal
6. `const Type &z` - point at which to evaluate the polynomial
7. `Type p` - k-th derivative of $P(z)$\
   If $k > d$, `p = Type(0)`
8. Operations: $x$ and $y$ are objects of type `Type`, and $i$-`int` \
   `x = i` \
   `x = y` \
   `x *= y` \
   `x += y`

9. Vector: the type `Vector` must be a `SimpleVector` class with element_of_type `Type`. The routine `CheckSimpleVector` will generate error message if this is not the case

## AD - Algorithmic Differentiation
Efficient method for calculating its derivatives at arbitrary order and analytic in nature.

1. Forward mode: computes the partial derivative of all the dependent variables  w.r.t one independent variable (or independent variable direction)
2. Reverse mode: compures the partial derivative of one dependent variable (or one dependent variable direction) w.r.t all the independent variables
3. Operation count: # of floating point operations for either a forward ore reverse mode is a small multiple of the number required to evaluate the original function. \
   Using reverse mode, you can evaluate the derivative of a scalar valued function w.r.t thousands of variables in a small multiple of the work to evaluate the original function
4. Efficiency: AD automatically takes advantages of the speed of your algorithmic representation of a function. \
   E.g., if you calculate a determinant using LU factorizationn, AD will use the LU representation for the derivative of the determinant (faster than using the definition of the determinant).



# Use Ipopt to Solve a Nonlinear Programming Problem

Problem:

$$
   minimize: f(x) \\
   subject \ to: gl \leq g(x) \leq gu \\
   xl \leq x \leq xu
$$


Syntax:
```cpp
# include <cppad/ipopt/solve.hpp>
ipopt::solve(
   options, xi, xl, xu, gl, gu, fg_eval, solution
) 
```
Params:
- `const Vector& xi` , with `size() == nx`. Initial point where Ipopt starts the optimization process. 
- `const Vector& xl`, with `size() == nx`. Lower limits of $x$
- `const Vector& xu`, with `size() == nx`. Upper limits of $x$
- `const Vector& gl`, with `size() == nx`. Lower limits of $g(x)$
- `const Vector& gu`, with `size() == nx`. Upper limits of $g(x)$
- `FG_eval fg_eval` - class `FG_eval` is unspecified except for the fact that it supports the syntax
```cpp
   FG_eval::ADvector
   fg_eval(fg, x)
```
The type `ADvector` and the arguments to fg , x have the following meaning:
   
   - `FG_eval::ADvector` must be a `SimpleVector` class with elements of type `AD<double>`. 
   - `const ADvector& x` where `nx = x.size()`.
   - `ADvector& fg` where `1 + ng = fg.size()`. The input value of the elements of fg does not matter. Upon return from `fg_eval` , 
      - fg[0] = $f(x)$
      - for i=0,…,ng−1, \
         fg[1 + i] = $g_i(x)$

Solution: `ipopt::solve_result<Dvector>& solution` contains
- `ipopt::solve_result<Dvector>::status_type solution.status` - final Ipopt status for the optimizer
   - `not_defined` - did not return a final status for the problem.
   - `unknown` - not defined status for `finalize_solution`.
   - `maxiter_exceeded`
   - `stop_at_tiny_step` - slow progress -> stop!
   - `stop_at_acceptable_point` - converged to acceptable (not desired) tolerances
   - `local_infeasibility` - converged to a non-feasible point (no solution possibility)
   - `user_requested_stop`
   - `diverging_iterates` - the iterates are diverging
   - `restoration_failure` - Restoration phase failed, algorithm doesn't know how to proceed.
   - `error_in_step_computation` - An unrecoverable error occurred while Ipopt tried to compute the search direction.
   - `invalid_number_detected` - Algorithm received an invalid number (such as `nan` or `inf`) from the users function `fg_info.eval` or from the CppAD evaluations of its derivatives
   - `internal_error` - unknown Ipopt internal error
- `Vector solution.x` with `size()== nx`. Final $x$ value
- `Vector solution.zl` with `size()== nx`. Final Lagrange multipliers for the lower bounds on $x$.
- `Vector solution.zu` with `size()== nx`. Final Lagrange multipliers for the upper bounds on $x$.
- `Vector solution.g` with `size()== ng`. Final value for the constraint function $g(x)$.
- `Vector solution.lambda` with `size()== ng`. Final value for the Lagrange multipliers corresponding to the constraint function.
- `double solution.obj_value` - Final value of the objective function $f(x)$


Configures:
- `Bvector` must be a `SimpleVector` with elements of type == `bool`
- `Dvector` must be a `SimpleVector` with elements of type == `double`
- `const std::string options`, each option is terminated by "\n". Each line consists of two or three tokens separated by one or more spaces. 
- `Retape value` - the retape flag.
   - `value = true` -> `ipopt::solve` with retape the operation sequence for each new value of $x$.
   - `value = false` -> `ipopt::solve` with tape the operation sequence at the value of $x_i$ and use that sequence for the entire optimization process.
   - The default value is `false`.
- `Sparse value direction` - the sparse Jacobian and Hessian flag
   - `value = true` -> `ipopt::solve` will use a sparse matrix representation for the computation of Jacobians and Hessians
   - `value = false` -> `ipopt::solve` will use a full matrix representation for these calculations.
   - The default value is `false`.
   - If sparse is true, retape must be false.
   - It is unclear if `sparse_jacobian` would be faster user `forward` or `reverse` mode so you are able to choose the direction.
      - If `value == true && direction == forward`
the Jacobians will be calculated using SparseJacobianForward.
      - If `value == true && direction == reverse`
the Jacobians will be calculated using SparseJacobianReverse.
- `String name value` - any Ipopt string option. Here `name` is any valid Ipopt string option and `value` is its setting.
- `Numeric name value` - any Ipopt numeric option. Here `name` is any valid Ipopt numeric option and `value` is its setting. 
- `Integer name value` - any Ipopt integer option. Here `name` is any valid Ipopt integer option and `value` is its setting.


