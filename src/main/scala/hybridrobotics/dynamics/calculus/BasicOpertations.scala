package hybridrobotics.dynamics.calculus
import hybridrobotics.dynamics.data_types._

object BasicOpertations {

  def diff(exp: Any): Any = exp match {
    case exp: ScalarExpr => exp.diff()
    case exp: VectorExpr => exp.diff()
    case exp: MatrixExpr => exp.diff()
    case exp: Int => 0
    case exp: Double => 0.0
    case exp: String => "dot"+exp
    case _ => "DIFF_NOT_DEFINED"
  }

  def delta(exp: Any): Any = exp match {
    case exp: ScalarExpr => exp.delta()
    case exp: VectorExpr => exp.delta()
    case exp: MatrixExpr => exp.delta()
    case exp: Int => 0
    case exp: Double => 0.0
    case exp: String => "delta_"+exp
    case _ => "DELTA_NOT_DEFINED"
  }


}
