package hybridrobotics.dynamics.data_types

// Vector Expression
trait VectorExpr extends Variable with TimeVarying {
  //Wrap s:String to MatrixExp

  import language.implicitConversions

  implicit def str2VectorExpr(s: String, t: String): VectorExpr = Vector(s)

  override val name: String = ""

  override val size = List(3, 1)

  override def diff(): VectorExpr = {
    this match {
      case Cross(a, b) => VAdd(Cross(a.diff(), b), Cross(a, b.diff()))
      case VAdd(a, b) => VAdd(a.diff(), b.diff())
      case SMul(a, b) => SMul(a.diff(), b) + SMul(a, b.diff())
      case MVMul(a, b) => MVMul(a.diff(), b) + MVMul(a, b.diff())
      case DeltaV(v) => DeltaV(v.diff())
      case VeeMap(m) => VeeMap(m.diff())
      case TransposeVector(v) => TransposeVector(v.diff())
      //    case AVec(s, u) => AVec(s + "dot", diffV(u))
      case _ => this.diff()
    }
  }

  override def delta(): VectorExpr = DeltaV(this)

  //Infix operators
  def x(v: VectorExpr): VectorExpr = Cross(this, v)

  def *(v: ScalarExpr): VectorExpr = SMul(this, v)

  def +(v: VectorExpr): VectorExpr = VAdd(this, v)

  def -(v: VectorExpr): VectorExpr = VAdd(this, SMul(v, Num(-1)))

  def dot(v: VectorExpr): ScalarExpr = Dot(this, v)

  def T: VectorExpr = TransposeVector(this)

}

// Vector Algebra classes
case class DeltaV(u: VectorExpr) extends VectorExpr // delta prefix

case class Cross(u: VectorExpr, v: VectorExpr) extends VectorExpr // u x v infix

case class SMul(u: VectorExpr, v: ScalarExpr) extends VectorExpr // u * v infix

case class VAdd(u: VectorExpr, v: VectorExpr) extends VectorExpr // u + v infix

case class Dot(u: VectorExpr, v: VectorExpr) extends ScalarExpr // u dot v infix

case class TransposeVector(v: VectorExpr) extends VectorExpr {

  override def T: VectorExpr = this.v

  override val size: List[Int] = List(this.v.size(1), this.v.size(0))

}

case class VeeMap(m: MatrixExpr) extends VectorExpr // TODO update input to SkewSymMatrix

// Vector Types
case class Vector(override val name: String) extends VectorExpr {
  // Normal Vector
  override def diff(): VectorExpr = Vector("dot" + this.name)
}

case class UnitVector(override val name: String) extends VectorExpr with UnitNorm {
  // UnitVector
  override def diff(): VectorExpr = Vector("dot" + this.name)
}

case class ConstVector(override val name: String) extends VectorExpr with ConstantVector {
  // Constant Vector
  override def diff(): VectorExpr = SMul(ZeroVector("dot" + this.name), Num(0))
}

//case class AVec(s: String, u: VectorExpr) extends VectorExpr // holds symbolic reference for large vector

case class ZeroVector(override val name: String) extends VectorExpr {
  // ZeroVector Vector
  override def diff(): VectorExpr = SMul(ZeroVector("dot" + this.name), Num(0))
}

case class SkewMatVector(override val name: String) extends VectorExpr {
  // TODO update this vector  (or make it obsolete)
  override def diff(): VectorExpr = Vector(this.name + "dot")
}

case class S2(override val name: String) extends VectorExpr with UnitNorm with SmoothManifold {

  val variationStr: String = "xi_{" + name + "}"
  val tangentStr: String = "omega_{" + name + "}"

  def norm: ScalarExpr = Num(1.0)

  override def getTangentVector: VectorExpr = Vector(this.tangentStr)

  override def getVariationVector: VectorExpr = Vector(this.variationStr)

  override def delta(): VectorExpr = Cross(this.getVariationVector, this)

  override def diff(): VectorExpr = Cross(this.getTangentVector, this)

}