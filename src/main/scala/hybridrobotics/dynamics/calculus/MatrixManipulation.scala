package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.TreeProperties._

object MatrixManipulation {


  def extractVariationCoefficients(equation: VectorExpr, variables: List[VectorExpr]): Map[VectorExpr, MatrixExpr] = {
    // TODO make it generalized to scalar and matrices, currently only vectors
    var coefficientMap: Map[VectorExpr, MatrixExpr] = Map()

    for (vector_ <- variables) {
      if (!isVectorAMember(equation, vector_))
        coefficientMap += vector_ -> ZeroMatrix()
      else {
        coefficientMap += vector_ -> getVariationCoefficient(equation, vector_)
      }
    }
    return coefficientMap
  }

  def getVariationCoefficient(expr: Any, focusVector: VectorExpr): MatrixExpr = expr match {
    case VAdd(u, v) => {
      val is_in_u = isVectorAMember(u, focusVector)
      val is_in_v = isVectorAMember(v, focusVector)

      if (is_in_u && is_in_v) {
        val tmp = MAdd(getVariationCoefficient(u, focusVector), getVariationCoefficient(v, focusVector))
        return tmp
      }
      else if (is_in_u) {
        return getVariationCoefficient(u, focusVector)
      }
      else if (is_in_v) {
        return getVariationCoefficient(v, focusVector)
      }
      else {
        ZeroMatrix()
      }
    }

    case MVMul(u, v) => {
      if (v == focusVector) {
        return u
      }
      else {
        val is_in_u = isVectorAMember(u, focusVector)
        val is_in_v = isVectorAMember(v, focusVector)
        if (is_in_u && is_in_v) println("This should be a linear equation")
        if (is_in_u) {
          return Matrix("fix this") // TODO
        }
        else if (is_in_v) {
          return MMul(u, getVariationCoefficient(v, focusVector))
        }
        else {
          return ZeroMatrix()
        }
      }
    }

    case Cross(u, v) => {
      if (u == v) return ZeroMatrix()
      else if (v == focusVector) return CrossMap(u)
      else if (u == focusVector) return SMMul(CrossMap(v), NumScalar(-1.0))
      else {
        val is_in_u = isVectorAMember(u, focusVector)
        val is_in_v = isVectorAMember(v, focusVector)
        if (is_in_u && is_in_v) println("This should be a linear equation")
        if (is_in_u) return MMul(SMMul(CrossMap(v), NumScalar(-1.0)), getVariationCoefficient(u, focusVector))
        else if (is_in_v) return MMul(CrossMap(u), getVariationCoefficient(v, focusVector))
        else return ZeroMatrix()
      }
    }

    case SMul(v, s) => SMMul(getVariationCoefficient(v, focusVector), s)

    case expr: VectorExpr =>
      if (expr == focusVector) IdentityMatrix()
      else if (!isVectorAMember(expr, focusVector)) ZeroMatrix()
      else getVariationCoefficient(expr, focusVector)

    case _ => IdentityMatrix()
  }

}
