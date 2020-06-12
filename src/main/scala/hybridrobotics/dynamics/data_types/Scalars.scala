package hybridrobotics.dynamics.data_types


// Scalar Expression
trait ScalarExpr extends Variable with TimeVarying {
  //Wrap s:String to ScalarExpr

  import language.implicitConversions

  implicit def int2ScalarExpr(i: Int): ScalarExpr = Num(i.toDouble)

  implicit def double2ScalarExpr(d: Double): ScalarExpr = Num(d)

  implicit def str2ScalarExpr(name: String): ScalarExpr = Var(name)

  override val name: String = ""

  override val size = List(1, 1)

  override def diff(): ScalarExpr = {
    this match {
      case DeltaS(u) => DeltaS(u.diff())
      case Par(u) => Par(u.diff())
      case Mul(u, v) => Mul(u.diff(), v) + Mul(u, v.diff())
      case Add(u, v) => u.diff() + v.diff()
      case Dot(a, b) => Dot(a.diff(), b) + Dot(a, b.diff())
      case _ => this.diff()
    }
  }

  override def delta(): ScalarExpr = DeltaS(this)

  // (Algebra) Infix operators from high to low using Scala precedence
  def *(v: ScalarExpr): ScalarExpr = Mul(this, v)

  def +(v: ScalarExpr): ScalarExpr = Add(this, v)

  def -(v: ScalarExpr): ScalarExpr = Add(this, Mul(Num(-1), v)) // Subtract is stored as adding a negative

  def T: ScalarExpr = this // scalar transpose
}

// Scalar Types
case class Num(value: Double) extends ScalarExpr {
  // Numeric
  override def diff(): ScalarExpr = Num(0.0)

  override def delta(): ScalarExpr = Num(0.0)

}

case class Var(override val name: String) extends ScalarExpr {
  // Variable string
  override def diff(): ScalarExpr = Var("dot" + this.name)

  override def delta(): ScalarExpr = DeltaS(this)
}

case class Const(override val name: String) extends ScalarExpr {
  // Constant scalar
  override def diff(): ScalarExpr = Num(0.0)

  override def delta(): ScalarExpr = Num(0.0)
}

// Scalar Operation classes
case class DeltaS(u: ScalarExpr) extends ScalarExpr // delta prefix

case class Par(u: ScalarExpr) extends ScalarExpr // parentheses

case class Mul(u: ScalarExpr, v: ScalarExpr) extends ScalarExpr // u * v infix

case class Add(u: ScalarExpr, v: ScalarExpr) extends ScalarExpr // u + v infix