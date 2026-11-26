# DSL for CLF/CBF

This package contains the DSL used by `CLF` and `CBF` classes to declare mathematical constraints without
constructing Gurobi objects directly inside individual control functions.

The core idea is:

1. the formula is built only once when the QP model is installed;
2. the structure of the Gurobi variables remains fixed;
3. at every iteration, only numeric coefficients and RHS values are updated using the current state.

This makes it possible to write readable formulas, for example:

```kotlin
override fun ControlFunctionScope.formula(): ConstraintFormula {
    val distance = self.position - other.position
    val minDistance = max(self.safeMargin, other.safeMargin)
    val h = squaredNorm(distance) - squared(minDistance)
    return 2.0 * dot(distance, self.u - other.u) + slack greaterThanOrEqualTo
        -(eta / timeStep) * h
}
```

without rebuilding the Gurobi model at every step, and **without importing anything from the DSL
packages**: the whole formula vocabulary is exposed either as members of the expression types or as
members of `ControlFunctionScope`.

## Main files

- `ControlFunctionScope.kt`: the single entry point for formulas. It exposes the endpoints (`self`,
  `other`), `timeStep`, `slack`, the lifting functions `scalar { ... }` / `vector { ... }`, and the
  whole formula vocabulary: `dot`, `squaredNorm`, `squared`, `max`, numeric multipliers
  (`2.0 * ...`, `eta / timeStep`), and the infix constraint builders `lessThanOrEqualTo` /
  `greaterThanOrEqualTo`.
- `ConstraintFormula.kt`: defines the linear/quadratic formula types produced by the constraint
  builders.
- `FormulaInstaller.kt`: compiles a formula into a reusable Gurobi constraint and produces an
  `InstalledControlConstraint`. It also owns the slack rules (see below).
- `FormulaRuntime.kt`: contains the values available while formulas are updated: local device,
  optional neighbor, QP settings, and `deltaTime`.
- `expressions/`: contains the small symbolic types used by the DSL (`RuntimeScalar`,
  `VectorExpression`, `DecisionExpression`, `AffineExpression`, `QuadraticExpression`). Their
  arithmetic operators (`+`, `-`, `*`, `/`, unary `-`) are member functions.

## How it works

A `CLF` or `CBF` implements:

```kotlin
protected abstract fun ControlFunctionScope.formula(): ConstraintFormula
```

The formula is evaluated when the model is installed, but dynamic values are not read immediately.
Expressions such as `self.position`, `self.maxSpeed`, `timeStep`, or `vector { ... }` build objects
that are evaluated later, inside `InstalledControlConstraint.update`.

For linear constraints:

- the left-hand side is an `AffineExpression`;
- the Gurobi variables present on the left-hand side are discovered during installation, and the
  grouping of runtime coefficients by variable is precomputed once;
- at runtime, the RHS is updated and `model.chgCoeff(...)` is called on each variable with the sum
  of its re-evaluated coefficients.

For quadratic constraints:

- the left-hand side is a `QuadraticExpression`;
- the quadratic structure is fixed during installation;
- at runtime, only the RHS is updated.

This distinction is important: the DSL currently supports dynamic coefficients in linear
constraints, but not dynamic quadratic coefficients.

## What `ControlFunctionScope` exposes

Inside `formula()`, the following values are available:

- `self`: local endpoint.
- `other`: neighbor endpoint; use it only in pairwise constraints.
- `self.u` / `other.u`: Gurobi decision vectors.
- `self.position` / `other.position`: runtime positions.
- `self.safeMargin` / `other.safeMargin`: runtime margins.
- `self.maxSpeed` / `other.maxSpeed`: runtime maximum speeds.
- `timeStep`: control step duration.
- `slack`: slack variable as an affine expression; it is empty if no `slackWeight` is declared.
- `scalar { ... }`: lifts a dynamic `Double` value into the DSL.
- `vector { ... }`: lifts a dynamic `DoubleArray` value into the DSL.
- `dot(coefficients, decision)`: affine dot product between a runtime vector and decision variables.
- `squaredNorm(...)`: squared norm of a runtime vector (scalar) or of a decision expression
  (quadratic).
- `squared(...)`, `max(...)`: scalar helpers.
- `Number * RuntimeScalar`, `Number * AffineExpression`, `Number / RuntimeScalar`: numeric literal
  multipliers.
- `lessThanOrEqualTo` / `greaterThanOrEqualTo`: infix constraint builders.

`other` is lazy: a local formula can ignore it; a pairwise formula fails early if it is installed
without a decision vector or without a neighbor device.

## Defining a new CBF

For a new CBF:

1. extend `CBF`;
2. choose `name`, `eta`, and `slackWeight`;
3. implement `ControlFunctionScope.formula()`;
4. use only dynamic values expressed through the DSL, not values read directly during installation.

Example of a local linear constraint:

```kotlin
class MyCBF(
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
) : CBF() {
    override val name: String = "my_cbf"

    override fun ControlFunctionScope.formula(): ConstraintFormula {
        val distance = self.position - vector { /* current reference position */ }
        val h = squaredNorm(distance) - squared(self.safeMargin)
        return 2.0 * dot(distance, self.u) + slack greaterThanOrEqualTo
            -(eta / timeStep) * h
    }
}
```

If the CBF depends on external data that may change, store it through providers and lift it with
`scalar { ... }` or `vector { ... }`, as `ObstacleAvoidanceCBF` does.

If the CBF is pairwise, use `other.position`, `other.u`, `other.safeMargin`, and similar values. In
that case, the solver must install it in the pairwise model.

## Defining a new CLF

For a new CLF:

1. extend `CLF`;
2. define `convergenceRate` and `slackWeight`;
3. implement `ControlFunctionScope.formula()`;
4. remember that CLFs always require a finite positive `slackWeight`.

Example:

```kotlin
class MyCLF(
    override val convergenceRate: Double = 1.0,
    override val slackWeight: Double? = 1.0,
    private var targetProvider: () -> Target,
) : CLF() {
    override val name: String = "my_clf"

    override fun syncFrom(other: ControlFunction) {
        if (other is MyCLF) {
            targetProvider = other.targetProvider
        }
    }

    override fun ControlFunctionScope.formula(): ConstraintFormula {
        val target = vector { targetProvider().position.toDoubleArray() }
        val distance = self.position - target
        return 2.0 * timeStep * dot(distance, self.u) - slack lessThanOrEqualTo
            -convergenceRate * squaredNorm(distance)
    }
}
```

When a CLF or CBF contains dynamic providers, implement `syncFrom` if the instance installed in the
solver needs to be updated with providers coming from a new application-level instance.

## Slack

Slack is derived directly from `slackWeight` — there is no separate policy type:

- `slackWeight == null`: no slack variable; `slack` in the formula is an empty affine expression
  and the constraint is hard;
- `slackWeight != null`: a slack variable is created and penalized in the objective with the given
  weight; the weight must be finite and positive;
- CLFs additionally *require* a non-null `slackWeight` (enforced at installation);
- quadratic constraints reject slack: `GRBQConstr` cannot host a slack term on the LHS.

`FormulaInstaller` validates all of this and fails early at installation time.

## Available expressions

The DSL exposes only the operators needed by the current formulas:

- runtime scalars (members of `RuntimeScalar`): `+`, `-`, unary `-`, `*`, `/`, and
  `RuntimeScalar * AffineExpression`;
- runtime vectors (member of `VectorExpression`): subtraction;
- decision vectors (members of `DecisionExpression`): `+`, `-`, unary `-` (e.g. `self.u - other.u`);
- affine expressions (members of `AffineExpression`): `+`, `-`, unary `-`;
- scope members: `dot`, `squaredNorm`, `squared`, `max`, `Number * ...`, `Number / ...`, and the
  infix constraint builders.

If a new operator is needed, add it only when a real formula requires it. The package is
intentionally small to avoid unused symmetric overloads.

## Extending the DSL

Before extending the DSL, identify which expression type should be produced:

- dynamic scalar value: add members on `RuntimeScalar` or helpers on `ControlFunctionScope`;
- dynamic vector value: add members on `VectorExpression`;
- linear coefficient of Gurobi variables: use or extend `AffineExpression`;
- fixed quadratic structure: use or extend `QuadraticExpression`.

Prefer adding vocabulary as members of `ControlFunctionScope` (or of the expression types): this
keeps formulas import-free and makes the whole vocabulary discoverable from the scope.

Rule of thumb: a DSL function must not read dynamic values during installation. Instead, it should
store a function that reads from `FormulaRuntime` during the update. `FormulaRuntime` contains the
current formula snapshot: local device, optional neighbor, QP settings, and control step.

To extend linear constraints:

- create or combine `AffineExpression` values;
- make sure the set of Gurobi variables is known during installation;
- let coefficients be `RuntimeScalar` values.

To extend quadratic constraints:

- keep the quadratic structure fixed;
- update only the RHS, unless `FormulaInstaller` is explicitly changed as well.

To support a new class of constraints, for example quadratic constraints with dynamic coefficients,
adding operators is not enough: the installation/update protocol in `FormulaInstaller` must also be
extended.

## Checklist for a new formula

- Does the formula use `self` and `other` only in the correct context?
- Are time-varying values lifted with `scalar { ... }` or `vector { ... }`?
- Is the left-hand side affine or quadratic according to what Gurobi needs to install?
- Is `slackWeight` consistent with the constraint kind (required for CLFs, forbidden on quadratic)?
- If there are dynamic providers, does `syncFrom` update the installed instance?
