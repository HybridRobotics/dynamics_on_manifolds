package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.TreeProperties._

object MatrixManipulation {


  def extractVariationCoefficients(equation: VectorExpr, variables: List[VectorExpr]): Map[VectorExpr, MatrixExpr] = {
    // TODO make it generalized to scalar and matrices, currently only vectors
    var coefficientMap: Map[VectorExpr, MatrixExpr] = Map()

    var eqn = equation.basicSimplify()

    for (vector_ <- variables) {
      val ismember = isVectorAMember(eqn, vector_)
      if (!isVectorAMember(eqn, vector_))
        coefficientMap += vector_ -> ZeroMatrix()
      else {
        var tmp =  getVariationCoefficient(eqn, vector_)
        tmp = tmp.basicSimplify() // TODO remove the temporary variable once debug is complete
        coefficientMap += vector_ -> tmp
      }
    }
    return coefficientMap
  }

  def getVariationCoefficient(expr: Any, focusVector: VectorExpr): MatrixExpr = expr match {
    // Scalar Expr
    // TODO write case for Dot(a.b)c

    // VectorExpr
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

    case Cross(u, v) => {
      if (u == v) return ZeroMatrix()
      else if (v == focusVector) return CrossMap(u)
      else if (u == focusVector) return SMMul(CrossMap(v), NumScalar(-1.0))
      else {
        val is_in_u = isVectorAMember(u, focusVector)
        val is_in_v = isVectorAMember(v, focusVector)
        if (is_in_u && is_in_v) throw new Exception("Cross: both vectors cannot have the focusVector")
        if (is_in_u) return MMul(SMMul(CrossMap(v), NumScalar(-1.0)), getVariationCoefficient(u, focusVector))
        else if (is_in_v) return MMul(CrossMap(u), getVariationCoefficient(v, focusVector))
        else return ZeroMatrix()
      }
    }

    // MatrixExpr
    case MVMul(mat, vec) => {
      val is_in_mat = isVectorAMember(mat, focusVector)
      val is_in_vec = isVectorAMember(vec, focusVector)
      if (is_in_vec) {
        if (vec == focusVector)
          return mat
        else
          return MMul(mat, getVariationCoefficient(vec, focusVector))
      }
      else if (is_in_mat) {
        mat match {
          case MMul(lmat, rmat) => {
            val is_in_lmat = isVectorAMember(lmat, focusVector)
            val is_in_rmat = isVectorAMember(rmat, focusVector)
            if (is_in_lmat) return getVariationCoefficient(MVMul(lmat, MVMul(rmat, vec)), focusVector)
            else if (is_in_rmat) return MMul(lmat, getVariationCoefficient(MVMul(rmat, vec), focusVector))
            else ZeroMatrix()
          }
          case SMMul(sub_mat, sclr) => getVariationCoefficient(MVMul(sub_mat, SMul(vec, sclr)), focusVector)
          case CrossMap(y) => {
            if (y == focusVector)
              return SMMul(CrossMap(vec), NumScalar(-1.0))
            else
              return MMul(SMMul(CrossMap(vec), NumScalar(-1.0)), getVariationCoefficient(y, focusVector))
          }
        }
      }
      else
        return ZeroMatrix()
    }

    case SMul(v, s) => {
      val is_in_vec = isVectorAMember(v, focusVector)
      val is_in_sclr = isVectorAMember(s, focusVector)

      if (is_in_vec && is_in_sclr) throw new Exception("SMul: both vector and scalar cannot have the focusVector")
      if (is_in_vec) {
        return SMMul(getVariationCoefficient(v, focusVector), s)
      }
      else if (is_in_sclr) {
        s match {
          case Add(a, b) => {
            val is_in_a = isVectorAMember(a, focusVector)
            val is_in_b = isVectorAMember(b, focusVector)
            if (is_in_a && is_in_b) {
              MAdd(getVariationCoefficient(SMul(v,a), focusVector), getVariationCoefficient(SMul(v, b), focusVector))
            }
            else if (is_in_a) {
              getVariationCoefficient(SMul(v, a), focusVector)
            }
            else if (is_in_b) {
              getVariationCoefficient(SMul(v, b), focusVector)
            }
            else ZeroMatrix()
            //            MVMul(MAdd(getVariationCoefficient(a, focusVector), getVariationCoefficient(b, focusVector)), v)
          }

          case Mul(a, b) => {
            val is_in_a = isVectorAMember(a, focusVector)
            val is_in_b = isVectorAMember(b, focusVector)
            if (is_in_a && is_in_b) throw new Exception("Mul: both scalars cannot have the focusVector")
            if (is_in_a) {
              return getVariationCoefficient(SMul(SMul(v, b), a), focusVector)
            }
            if (is_in_b) {
              return getVariationCoefficient(SMul(SMul(v, a),b), focusVector)
            }
            else ZeroMatrix()

          }

          case Dot(a, b) => {
            val is_in_a = isVectorAMember(a, focusVector)
            val is_in_b = isVectorAMember(b, focusVector)
            if (is_in_a && is_in_b) throw new Exception("Dot: both vectors cannot have the focusVector")
            if (is_in_a) {
              getVariationCoefficient(MVMul(VVMul(v, TransposeVector(b)), a), focusVector)
            }
            else if(is_in_b) {
              getVariationCoefficient(MVMul(VVMul(v, TransposeVector(a)), b), focusVector)
            }
            else ZeroMatrix()
          }

          case _ => ZeroMatrix()
        }
      }
      else {
        return ZeroMatrix()
      }
    }

    case MMul(u, v) => {
      val is_in_u = isVectorAMember(u, focusVector)
      val is_in_v = isVectorAMember(v, focusVector)
      if (is_in_u && is_in_v) throw new Exception("MMul: both matrices cannot have the focusVector")
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

    case expr: VectorExpr =>
      if (expr == focusVector) IdentityMatrix()
      else if (!isVectorAMember(expr, focusVector)) ZeroMatrix()
      else getVariationCoefficient(expr, focusVector)

    case _ => IdentityMatrix()
  }

}
