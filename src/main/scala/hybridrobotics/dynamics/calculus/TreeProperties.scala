package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._

object TreeProperties {

  // is node element a literal
  def isLeaf(e: ScalarExpr): Boolean = e match {
    case Add(u, v) => return false
    case Mul(u, v) => return false
    case Dot(a, b) => return false
    case u: ScalarExpr => return true
  }

  // is a scalar element z in expression e
  def isMember(e: ScalarExpr, z: ScalarExpr): Boolean = e match {
    case Add(u, v) => isMember(u, z) && isMember(v, z)
    case Mul(u, v) => isMember(u, z) && isMember(v, z)
    case u: ScalarExpr =>
      if (u == z) return true
      else return false
  }

  // is a vector element  in expression vectorExpr
  def isVectorAMember(expr: Any, element: VectorExpr): Boolean = {
    // this function verifies if a given vector is part of a expression
    expr match {
        // Scalar Expr
      case Add(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case Mul(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case Dot(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
        // Vector Expr
      case VAdd(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case Cross(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case SMul(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case TransposeVector(v) => isVectorAMember(v, element)
      case MVMul(m, v) => isVectorAMember(m, element) || isVectorAMember(v, element)
        // Base case for vectors
      case expr: VectorExpr =>
        if (expr == element)
          return true
        else
          return false
        // Matrix Expr
      case MAdd(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case MMul(u, v) => isVectorAMember(u, element) || isVectorAMember(v, element)
      case SMMul(m, s) => isVectorAMember(m, element) || isVectorAMember(s, element)
      case VVMul(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
      case TransposeMatrix(m) => isVectorAMember(m, element)
      case CrossMap(v) =>
        if (v == element)
          return true
        else
          return false
      case _ => false
    }
  }

  //
  //  def isVectorAMember(expr: Any, element: VectorExpr): Boolean = {
  //    // this function verifies if a given vector is part of a expression
  //    expr match {
  //      case expr: ScalarExpr => expr match {
  //        case Dot(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
  //        case _ => false
  //      }
  //      case expr: VectorExpr => expr match {
  //        case VAdd(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
  //        case Cross(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
  //        case SMul(a, b) => isVectorAMember(a, element)
  //        case TransposeVector(v) => isVectorAMember(v, element)
  //        case MVMul(m, v) => isVectorAMember(m, element) || isVectorAMember(v, element)
  //        case expr: VectorExpr =>
  //          if (expr == element)
  //            return true
  //          else
  //            return false
  //      }
  //      case expr: MatrixExpr => expr match {
  //        case VVMul(a, b) => isVectorAMember(a, element) || isVectorAMember(b, element)
  //        case MMul(u, v) => isVectorAMember(u, element) || isVectorAMember(v, element)
  //        case TransposeMatrix(m) => isVectorAMember(m, element)
  //        case CrossMap(v) =>
  //          if (v==element)
  //            return true
  //          else
  //            return false
  //        case _ => false
  //      }
  //
  //    }
  //
  //  }

  // find violations of the expanded structure
  def nestedAdds(e: ScalarExpr): Boolean = e match {
    case Add(u, v) => return true
    case Mul(u, v) => return nestedAdds(u) || nestedAdds(v)
    case Dot(a, b) => return nestedAddsV(a) || nestedAddsV(b)
    case u: ScalarExpr => return false
  }

  def nestedAddsV(v: VectorExpr): Boolean = v match {
    case VAdd(u, v) => return true
    case MVMul(m, v) => return nestedAddsM(m) || nestedAddsV(v)
    case SMul(v, u) => return nestedAdds(u) || nestedAddsV(v)
    case Cross(a, b) => return nestedAddsV(a) || nestedAddsV(b)
    case v: VectorExpr => return false
  }


  def nestedAddsM(m: MatrixExpr): Boolean = m match {
    case MAdd(u, v) => return true
    case MMul(m, p) => return nestedAddsM(m) || nestedAddsM(p)
    case SMMul(m, u) => return nestedAdds(u) || nestedAddsM(m)
    case m: MatrixExpr => return false
  }


  // nested scalars searched to combine constant coefficiencts
  def nestedScalars(e: ScalarExpr): Boolean = e match {
    case Add(u, v) => return nestedScalars(u) || nestedScalars(v)
    case Mul(u, v) => return nestedScalars(u) || nestedScalars(v)
    case Dot(a, b) => return nestedScalarsV(a) || nestedScalarsV(b)
    case u: ScalarExpr => return false
  }

  def nestedScalarsV(v: VectorExpr): Boolean = v match {
    case VAdd(u, v) => return nestedScalarsV(u) || nestedScalarsV(v)
    case MVMul(m, v) => return nestedScalarsM(m) || nestedScalarsV(v)
    case SMul(v, u) => return true
    case Cross(a, b) => return nestedAddsV(a) || nestedAddsV(b)
    case v: VectorExpr => return false
  }

  def nestedScalarsM(m: MatrixExpr): Boolean = m match {
    case MAdd(m, p) => return nestedScalarsM(m) || nestedScalarsM(p)
    case MMul(m, p) => return nestedScalarsM(m) || nestedScalarsM(p)
    case SMMul(m, u) => return true
    case m: MatrixExpr => return false
  }
}
