package hybridrobotics.dynamics.data_types

trait TimeVarying {
  def diff(): Any

  def delta(): Any

  def getVariation: Any
}

trait Variable {
  val name: String

  val size: List[Int]

  def d: Any // Desired Variable
}

trait Expression {
  def basicSimplify(): Any
}

trait Iterator {

}
