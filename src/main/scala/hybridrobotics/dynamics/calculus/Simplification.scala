package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
import Expansion._
import TreeProperties._

object Simplification {

  // Main Simplification Function
  //  def simplify(expr: Any): Any = {
  //    expr match {
  //      case expr: ScalarExpr => simplifyScalarExpr(expr)
  ////      case expr: VectorExpr => simplifyVectorExpr(expr)
  ////      case expr: MatrixExpr => simplifyMatrixExpr(expr)
  //    }
  //  }


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
      return tmp
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

  def vectorRules(e: ScalarExpr): ScalarExpr = e match {
    case Add(u, v) => vectorRules(u) + vectorRules(v)
    case Mul(u, v) => vectorRules(u) * vectorRules(v)

    // Gather numeric constants nested in vector expressions (scalar multiples)
    case Dot(Cross(b, c), a) => basicSimplify(Dot(a, Cross(b, c)))
    case Dot(a, Cross(b, c)) => if ((a == b) || (a == c) || (b == c)) {
      NumScalar(0)
    } else {
      Dot(a, Cross(b, c))
    }
    case Dot(a: VectorExpr, b: VectorExpr) =>
      if ((a.isInstanceOf[UnitVector]) && (b.isInstanceOf[UnitVector]) && (a == b)) {
        NumScalar(1)
      }
      else {
        Dot(a, b)
      }

    case u: ScalarExpr => u
  }

  // removes the zero terms from the tree.
  def removeZeros(e: ScalarExpr): ScalarExpr = e match {
    case Add(NumScalar(d), v) => if (d == 0) {
      removeZeros(v)
    } else {
      Add(NumScalar(d), removeZeros(v))
    }
    case Add(u, NumScalar(d)) => if (d == 0) {
      removeZeros(u)
    } else Add(removeZeros(u), NumScalar(d))
    case Add(u, v) =>
      Add(removeZeros(u), removeZeros(v))
    case Mul(u, v) => Mul(u, v)
    case Dot(a, b) => Dot(a, b)
    case NumScalar(d) => NumScalar(d)
    case VarScalar(s) => VarScalar(s)
    case ConstScalar(s) => ConstScalar(s)
  }


  // pull scalar components out of vector and matrix expressions
  def pullM(m: MatrixExpr): MatrixExpr = m match {
    case MAdd(u, v) => pullM(u) + pullM(v)

    case MMul(m, SMMul(u, v)) => SMMul(MMul(m, u), v)
    case MMul(SMMul(u, v), m) => SMMul(MMul(m, u), v)
    case MMul(m, u) =>
      if (nestedScalarsM(MMul(m, u))) {
        pullM(MMul(pullM(m), pullM(u)))
      }
      else {
        MMul(m, u)
      }

    case SMMul(SMMul(a, b), v) => pullM(SMMul(a, b * v))
    case SMMul(u, v) => if (nestedScalarsM(u)) {
      pullM(SMMul(pullM(u), v))
    } else {
      SMMul(u, v)
    }
    case m: MatrixExpr => m
  }

  def pullV(v: VectorExpr): VectorExpr = v match {

    case MVMul(SMMul(u, v), w) => pullV(SMul(MVMul(u, w), v))
    case MVMul(m, SMul(u, v)) => pullV(SMul(MVMul(m, u), v))
    case MVMul(u, v) => if (nestedScalarsV(MVMul(u, v))) {
      pullV(MVMul(pullM(u), pullV(v)))
    } else {
      MVMul(u, v)
    }

    case SMul(SMul(u, v), w) => pullV(SMul(u, v * w))
    case SMul(u, v) => if (nestedScalarsV(u)) {
      pullV(SMul(pullV(u), v))
    } else SMul(u, v)

    case Cross(SMul(u, v), SMul(w, x)) => pullV(SMul(u x w, v * x))
    case Cross(a, SMul(u, v)) => pullV(SMul(a x u, v))
    case Cross(SMul(u, v), a) => pullV(SMul(u x a, v))
    case Cross(a, b) => if (nestedScalarsV(Cross(a, b))) {
      pullV(Cross(pullV(a), pullV(b)))
    } else {
      Cross(a, b)
    }

    case u: VectorExpr => u
  }

  def pull(e: ScalarExpr): ScalarExpr = e match {
    case Add(u, v) => pull(u) + pull(v)
    case Mul(u, v) => pull(u) * pull(v)
    case Dot(SMul(u, v), w) => pull(v * Dot(u, w))
    case Dot(w, SMul(u, v)) => pull(v * Dot(u, w))
    case Dot(u, v) => if (nestedScalars(Dot(u, v))) {
      pull(Dot(pullV(u), pullV(v)))
    } else {
      Dot(u, v)
    }
    case u: ScalarExpr => u
  }


  def simplifyScalarExpr(e: ScalarExpr): ScalarExpr = {
    var s = expansion(e)
    s = pull(s)
    s = vectorRules(s)
    s = basicSimplify(s)
    return s
  }

}
