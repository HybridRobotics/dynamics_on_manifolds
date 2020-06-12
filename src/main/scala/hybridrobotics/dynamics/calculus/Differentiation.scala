package hybridrobotics.dynamics.calculus
import hybridrobotics.dynamics.data_types._

object Differentiation {

  def diff(e: Any): Any = e match {
    case e: ScalarExpr => e.diff()
    case e: VectorExpr => e.diff()
    case e: MatrixExpr => e.diff()
    case e: Int => 0
    case e: Double => 0.0
    case e: String => "dot"+e
    case _ => "DIFF_NOT_DEFINED"
  }


}
