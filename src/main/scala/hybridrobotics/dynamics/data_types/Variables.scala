package hybridrobotics.dynamics.data_types

trait TimeVarying {
  def diff(): Any

  def delta(): Any
}

trait Variable {
  val name: String

  val size: List[Int]

  def d: Any // Desired Variable

  def getVariation(): Any
}

trait Expr {
  def basicSimplify(): Any
}

