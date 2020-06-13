package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._

object Simplification {

  // simplifies expression, includes combining constants and eliminating zero terms
  // also checks vector properties
  // tree must be in expanded form
  def basicSimplify(e: ScalarExpr): ScalarExpr = e match {
    // Add the Numerals
    case Add(NumScalar(number_1), NumScalar(number_2)) => NumScalar(number_1 + number_2)

    // Remove Zero Numerals
    case Add(scalar_expr, NumScalar(number)) =>
      if (number == 0) basicSimplify((scalar_expr)) // Ignore Zero Numeric
      else basicSimplify(Add(NumScalar(number), scalar_expr)) // Change the Add Tree Order
    case Add(NumScalar(number), scalar_expr) =>
      if (number == 0) basicSimplify(scalar_expr) // Ignore Zero Numeric
      else Add(NumScalar(number), basicSimplify(scalar_expr))
    case Add(u, v) => Add(basicSimplify(u), basicSimplify(v))

    // Gather numeric constants in various term structures.
    // In the expression NumScalar(d*e), d*e relies on scala's built in math package.
    case Mul(u, v) => if (findZeros(Mul(u, v))) {
      NumScalar(0)
    } else {
      val coeff = combineScalars(Mul(u, v))
      val remaining_terms = terms(Mul(u, v))
      val tmp = Mul(NumScalar(coeff), remaining_terms)
      return  tmp
    }
    case Dot(a, b) => Dot(a, b)
    case u: ScalarExpr => u
    case _ => e
  }

  // combine numberic constants
  def combineScalars(e: ScalarExpr): Double = e match {
    case Mul(u, v) => combineScalars(u) * combineScalars(v)
    case NumScalar(d) => return d
    case u: ScalarExpr => return 1
  }

  // determines if remaining elements in tree are all numbers
  def terms(e: ScalarExpr): ScalarExpr = e match {
    case Mul(NumScalar(d), NumScalar(e)) => NumScalar(1.0)
    case Mul(NumScalar(d), v) => terms(v)
    case Mul(v, NumScalar(d)) => terms(v)
    case Mul(u, v) =>
      if (anyTerms(u) && anyTerms(v)) terms(u) * terms(v)
      else if (anyTerms(u)) terms(u)
      else terms(v)
    case v: ScalarExpr => v
//      else if (anyTerms(v)) terms(v)
//      else NumScalar(1.0)
//    case _ => e
  }

  def anyTerms(e: ScalarExpr): Boolean = e match {
    case Mul(u, v) => anyTerms(u) || anyTerms(v)
    case NumScalar(d) => false
    case e: ScalarExpr => true
  }

  // find zero terms for elimination
  def findZeros(e: ScalarExpr): Boolean = e match {
    case Mul(u, v) => findZeros(u) || findZeros(v)
    case NumScalar(d) =>
      if (d == 0) true
      else false
    case e: ScalarExpr => false
  }


}
