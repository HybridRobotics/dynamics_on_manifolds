package hybridrobotics.dynamics.data_types

trait TimeVarying {
  def diff(): Any

  def delta(): Any
}

trait Variable {
  val name: String

  val size: List[Int]

  def basicSimplify(): Any
}

