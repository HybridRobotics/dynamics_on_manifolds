package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.calculus.TreeProperties._
import hybridrobotics.dynamics.data_types._

object Expansion {

  // expands complete abstract syntax tree
  // expands complete abstract syntax tree
  def expansion(e: ScalarExpr): ScalarExpr = {
    var A = expandScalarExpr(e)
    return A
  }

  // expandScalarExpr expression
  def expandMatrixExpr(m: MatrixExpr): MatrixExpr = m match {
    case MAdd(u, v) => MAdd(expandMatrixExpr(u), expandMatrixExpr(v))
    case MMul(m, MAdd(u, v)) => MAdd(MMul(m, expandMatrixExpr(u)), MMul(m, expandMatrixExpr(v)))
    case MMul(MAdd(u, v), m) => MAdd(MMul(m, expandMatrixExpr(u)), MMul(m, expandMatrixExpr(v)))
    case MMul(m, u) =>
      if (nestedAddsM(MMul(m, u))) {
        expandMatrixExpr(MMul(expandMatrixExpr(m), expandMatrixExpr(u)))
      }
      else {
        MMul(m, u)
      }
    case SMMul(MAdd(a, b), v) => expandMatrixExpr(MAdd(SMMul(a, v), SMMul(b, v)))
    case SMMul(v, Add(a, b)) => expandMatrixExpr(MAdd(SMMul(v, a), SMMul(v, b)))
    case SMMul(u, v) => if (expandMatrixExpr(u) != u) {
      expandMatrixExpr(SMMul(expandMatrixExpr(u), v))
    } else if (expandScalarExpr(v) != v) {
      expandMatrixExpr(SMMul(u, expandScalarExpr(v)))
    } else {
      SMMul(u, v)
    }
    case m: MatrixExpr => m
  }


  def expandVectorExpr(v: VectorExpr): VectorExpr = v match {
    case VAdd(u, v) => expandVectorExpr(u) + expandVectorExpr(v)
    case MVMul(MAdd(u, v), w) => VAdd(expandVectorExpr(MVMul(u, w)), expandVectorExpr(MVMul(v, w)))
    case MVMul(m, VAdd(u, v)) => VAdd(expandVectorExpr(MVMul(m, u)), expandVectorExpr(MVMul(m, v)))
    case MVMul(m, u) => if (nestedAddsV(MVMul(m, u))) {
      expandVectorExpr(MVMul(expandMatrixExpr(m), expandVectorExpr(u)))
    } else {
      MVMul(m, u)
    }

    case SMul(VAdd(a, b), v) => expandVectorExpr(SMul(a, v) + SMul(b, v))
    case SMul(u, v) => if (nestedAddsV(SMul(u, v))) {
      expandVectorExpr(SMul(expandVectorExpr(u), expandScalarExpr(v)))
    } else {
      SMul(u, v)
    }

    case Cross(VAdd(a, b), VAdd(c, d)) => VAdd(expandVectorExpr(Cross(a, c)), VAdd(expandVectorExpr(Cross(a, d)), VAdd(expandVectorExpr(Cross(b, c)), expandVectorExpr(Cross(b, d)))))
    case Cross(VAdd(a, b), c) => VAdd(expandVectorExpr(Cross(a, c)), expandVectorExpr(Cross(b, c)))
    case Cross(c, VAdd(a, b)) => VAdd(expandVectorExpr(Cross(a, c)), expandVectorExpr(Cross(b, c)))
    case Cross(a, b) =>
      if (nestedAddsV(Cross(a, b))) {
        expandVectorExpr(Cross(expandVectorExpr(a), expandVectorExpr(b)))
      } else {
        Cross(a, b)
      }


    case v: VectorExpr => v
  }

  def expandScalarExpr(e: ScalarExpr): ScalarExpr = e match {
    case Add(u, v) => Add(expandScalarExpr(u), expandScalarExpr(v))

    case Mul(Add(u, v), Add(w, x)) => Add(expandScalarExpr(Mul(u, w)), Add(expandScalarExpr(Mul(v, w)), Add(expandScalarExpr(Mul(u, x)), expandScalarExpr(Mul(v, x)))))
    case Mul(Add(u, v), w) => Add(expandScalarExpr(Mul(u, w)), expandScalarExpr(Mul(v, w)))
    case Mul(w, Add(u, v)) => Add(expandScalarExpr(Mul(u, w)), expandScalarExpr(Mul(v, w)))
    case Mul(u, v) =>
      if (nestedAdds(Mul(u, v))) {
        expandScalarExpr(Mul(expandScalarExpr(u), expandScalarExpr(v)))
      } else {
        Mul(u, v)
      }

    case Dot(VAdd(a, b), VAdd(c, d)) => Add(expandScalarExpr(Dot(a, c)), Add(expandScalarExpr(Dot(a, d)), Add(expandScalarExpr(Dot(b, c)), expandScalarExpr(Dot(b, d)))))
    case Dot(VAdd(a, b), c) => Add(expandScalarExpr(Dot(a, c)), expandScalarExpr(Dot(b, c)))
    case Dot(c, VAdd(a, b)) => Add(expandScalarExpr(Dot(a, c)), expandScalarExpr(Dot(b, c)))
    case Dot(a, b) =>
      if (nestedAdds(Dot(a, b))) {
        expandScalarExpr(Dot(expandVectorExpr(a), expandVectorExpr(b)))
      } else {
        Dot(a, b)
      }

    case u: ScalarExpr => u
  }

}
