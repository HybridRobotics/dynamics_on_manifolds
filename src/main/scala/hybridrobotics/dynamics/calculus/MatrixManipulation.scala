package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.Expansion._
import hybridrobotics.dynamics.calculus.TreeProperties._

object MatrixManipulation {


  def extractVariationCoefficients(equation: VectorExpr, variables: (List[ScalarExpr], List[VectorExpr], List[MatrixExpr])): (Map[ScalarExpr, VectorExpr], Map[VectorExpr, MatrixExpr], Map[MatrixExpr, MatrixExpr]) = {
    var  eqn = equation.basicSimplify()
    //    eqn = expandVectorExpr(eqn)
    //    eqn = eqn.basicSimplify()

    // variables List(output_variables, state_variables, input_variables)
    var sclrCoeffMap: Map[ScalarExpr, VectorExpr] = Map()
    for (scalar_ <- variables._1) {
      if (!eqn.is_member(scalar_)) {
        sclrCoeffMap += scalar_ -> ZeroVector()
      }
      else {
        sclrCoeffMap += scalar_ -> getScalarCoefficient(eqn, scalar_).basicSimplify()
      }
    }

    var vecCoeffMap: Map[VectorExpr, MatrixExpr] = Map()
    for (vector_ <- variables._2) {
      val ismember = eqn.is_member(vector_)
      if (!ismember)
        vecCoeffMap += vector_ -> ZeroMatrix()
      else {
        //        var tmp = getVectorCoefficient(eqn, vector_)
        //        tmp = tmp.basicSimplify() // TODO remove the temporary variable once debug is complete
        vecCoeffMap += vector_ -> getVectorCoefficient(eqn, vector_).basicSimplify()
      }
    }

    var matCoeffMap: Map[MatrixExpr, MatrixExpr] = Map()
    for (matrix_ <- variables._3) {
      if (!eqn.is_member(matrix_)) {
        matCoeffMap += matrix_ -> ZeroMatrix()
      }
      else {
        matCoeffMap += matrix_ -> getMatrixCoefficient(eqn, matrix_).basicSimplify()
      }
    }

    return (sclrCoeffMap, vecCoeffMap, matCoeffMap)
  }

  def getVectorCoefficient(expr: Any, focusVector: VectorExpr): MatrixExpr = expr match {
    // Scalar Expr
    // TODO write case for Dot(a.b)c

    // VectorExpr
    case VAdd(u, v) => {
      val is_in_u = isVectorAMember(u, focusVector)
      val is_in_v = isVectorAMember(v, focusVector)

      if (is_in_u && is_in_v) {
        val tmp = MAdd(getVectorCoefficient(u, focusVector), getVectorCoefficient(v, focusVector))
        return tmp
      }
      else if (is_in_u) {
        return getVectorCoefficient(u, focusVector)
      }
      else if (is_in_v) {
        return getVectorCoefficient(v, focusVector)
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
        if (is_in_u) return MMul(SMMul(CrossMap(v), NumScalar(-1.0)), getVectorCoefficient(u, focusVector))
        else if (is_in_v) return MMul(CrossMap(u), getVectorCoefficient(v, focusVector))
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
          return MMul(mat, getVectorCoefficient(vec, focusVector))
      }
      else if (is_in_mat) {
        mat match {
          case MAdd(lmat, rmat) => {
            val is_in_lmat = isVectorAMember(lmat, focusVector)
            val is_in_rmat = isVectorAMember(rmat, focusVector)
            if (is_in_lmat & is_in_rmat)
              return getVectorCoefficient(MVMul(lmat, vec), focusVector) + getVectorCoefficient(MVMul(rmat, vec), focusVector)
            else if (is_in_lmat)
              return getVectorCoefficient(MVMul(lmat, vec), focusVector)
            else if (is_in_rmat)
              return getVectorCoefficient(MVMul(rmat, vec), focusVector)
            else
              ZeroMatrix()
          }
          case MMul(lmat, rmat) => {
            val is_in_lmat = isVectorAMember(lmat, focusVector)
            val is_in_rmat = isVectorAMember(rmat, focusVector)
            if (is_in_lmat) return getVectorCoefficient(MVMul(lmat, MVMul(rmat, vec)), focusVector)
            else if (is_in_rmat) return MMul(lmat, getVectorCoefficient(MVMul(rmat, vec), focusVector))
            else ZeroMatrix()
          }
          case SMMul(sub_mat, sclr) => {
            val is_in_sub_mat = isVectorAMember(sub_mat, focusVector)
            val is_in_sclr = isVectorAMember(sclr, focusVector)
            if (is_in_sub_mat && is_in_sclr) throw new Exception("SMMul: both matrix and scalar cannot have the focusVector")
            if (is_in_sub_mat)
              getVectorCoefficient(MVMul(sub_mat, SMul(vec, sclr)), focusVector)
            else if (is_in_sclr)
              MMul(sub_mat, getVectorCoefficient(SMul(vec, sclr), focusVector))
            else
              ZeroMatrix()
          }
          case VVMul(u, v) => {
            val is_in_u = isVectorAMember(u, focusVector)
            val is_in_v = isVectorAMember(v, focusVector)
            if (is_in_u && is_in_v) throw new Exception("VVMul: both vectors cannot have the focusVector")
            if (is_in_u)
              return MMul(SMMul(IdentityMatrix(), Dot(v.v, vec)), getVectorCoefficient(u, focusVector)) // getVectorCoefficient(SMul(u, Dot(v, vec)), focusVector)
            else if (is_in_v)
              return MMul(VVMul(u, TransposeVector(vec)), getVectorCoefficient(v.v, focusVector))
            else
              ZeroMatrix()
          }
          case TransposeMatrix(sub_mat) => {
            val tmp = TransposeMatrix(sub_mat).basicSimplify()
            getVectorCoefficient(MVMul(tmp, vec), focusVector)
          }
          case CrossMap(y) => {
            if (y == focusVector)
              return SMMul(CrossMap(vec), NumScalar(-1.0))
            else
              return MMul(SMMul(CrossMap(vec), NumScalar(-1.0)), getVectorCoefficient(y, focusVector))
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
        return SMMul(getVectorCoefficient(v, focusVector), s)
      }
      else if (is_in_sclr) {
        s match {
          case Add(a, b) => {
            val is_in_a = isVectorAMember(a, focusVector)
            val is_in_b = isVectorAMember(b, focusVector)
            if (is_in_a && is_in_b) {
              MAdd(getVectorCoefficient(SMul(v, a), focusVector), getVectorCoefficient(SMul(v, b), focusVector))
            }
            else if (is_in_a) {
              getVectorCoefficient(SMul(v, a), focusVector)
            }
            else if (is_in_b) {
              getVectorCoefficient(SMul(v, b), focusVector)
            }
            else ZeroMatrix()
            //            MVMul(MAdd(getVectorCoefficient(a, focusVector), getVectorCoefficient(b, focusVector)), v)
          }
          case Mul(a, b) => {
            val is_in_a = isVectorAMember(a, focusVector)
            val is_in_b = isVectorAMember(b, focusVector)
            if (is_in_a && is_in_b) throw new Exception("Mul: both scalars cannot have the focusVector")
            if (is_in_a) {
              return getVectorCoefficient(SMul(SMul(v, b), a), focusVector)
            }
            if (is_in_b) {
              return getVectorCoefficient(SMul(SMul(v, a), b), focusVector)
            }
            else ZeroMatrix()
          }
          case Dot(a, b) => {
            val is_in_a = isVectorAMember(a, focusVector)
            val is_in_b = isVectorAMember(b, focusVector)
            if (is_in_a && is_in_b) throw new Exception("Dot: both vectors cannot have the focusVector")
            if (is_in_a) {
              getVectorCoefficient(MVMul(VVMul(v, TransposeVector(b)), a), focusVector)
            }
            else if (is_in_b) {
              getVectorCoefficient(MVMul(VVMul(v, TransposeVector(a)), b), focusVector)
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
        return Matrix("fix this") // TODO I shouldn't encounter this case
      }
      else if (is_in_v) {
        return MMul(u, getVectorCoefficient(v, focusVector))
      }
      else {
        return ZeroMatrix()
      }
    }

    case expr: VectorExpr => {
      if (expr == focusVector) IdentityMatrix()
      else if (!isVectorAMember(expr, focusVector)) ZeroMatrix()
      else getVectorCoefficient(expr, focusVector)
    }

    case _ => IdentityMatrix()
  }

  def getScalarCoefficient(expr: Any): VectorExpr = { // TODO complete getScalarCoefficient
    expr match {
      // Scalar Expr

      // Vector Expr

      // Matrix Expr

      // Default
      case _ => OnesVector()
    }
  }

  def getMatrixCoefficient(expr: Any): MatrixExpr = { // TODO complete getMatrixCoefficient
    expr match {
      // Scalar Expr

      // Vector Expr

      // Matrix Expr

      // Default
      case _ => IdentityMatrix()
    }
  }

  def replaceVectorExpr(expr: Any, oldVector: VectorExpr, newVector: VectorExpr): Any = {
    // TODO replaceVectorExpr
    return expr
  }

  def replaceMatrixExpr(expr: MatrixExpr, oldMatrix: MatrixExpr, newMatrix: MatrixExpr): Any = {
    // TODO replaceMatrixExpr
    return expr
  }

  def replaceScalrExpr(expr: ScalarExpr, oldScalar: ScalarExpr, newScalr: MatrixExpr): Any = {
    // TODO may be we can make this an inbuilt vector
    return expr
  }
}
