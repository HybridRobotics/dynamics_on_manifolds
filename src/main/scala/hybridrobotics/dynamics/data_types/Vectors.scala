package hybridrobotics.dynamics.data_types

// Vector Expression
trait VectorExpr extends Expression with TimeVarying {
  //Wrap s:String to MatrixExp

  import language.implicitConversions

  implicit def str2VectorExpr(s: String, t: String): VectorExpr = Vector(s)

  override def diff(): VectorExpr = {
    this match {
      case Cross(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) Cross(a, b.diff()) // Ignore differentiation of constant vector
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) Cross(a.diff(), b) // Ignore differentiation of constant vector
        else VAdd(Cross(a.diff(), b), Cross(a, b.diff()))

      case VAdd(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) b.diff() // Ignore variation of constant Vector
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) a.diff() // Ignore variation of constant Vector
        else VAdd(a.diff(), b.diff())

      case SMul(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) SMul(a, b.diff()) // Ignore variation of constant Vector
        else if (b.isInstanceOf[NumScalar] || b.isInstanceOf[ConstScalar]) SMul(a.diff(), b) // Ignore variation of constant scalar
        else SMul(a.diff(), b) + SMul(a, b.diff())

      case MVMul(a, b) =>
        if (a.isInstanceOf[ConstantMatrix]) MVMul(a, b.diff()) // Ignore variation of constant matrix
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) MVMul(a.diff(), b) // Ignore variation of constant Vector
        else MVMul(a.diff(), b) + MVMul(a, b.diff())

      case DeltaV(v) => DeltaV(v.diff())

      case VeeMap(m) => VeeMap(m.diff())

      case TransposeVector(v) => TransposeVector(v.diff())
      //    case AVec(s, u) => AVec(s + "dot", diffV(u))
      case _ => this.diff()
    }
  }

  override def delta(): VectorExpr = {
    this match {
      case VAdd(u, v) =>
        if (u.isInstanceOf[ConstantVector] || u.isInstanceOf[ZeroVector]) v.delta() // Ignore variation of constant Vector
        else if (v.isInstanceOf[ConstantVector] || v.isInstanceOf[ZeroVector]) u.delta() // Ignore variation of constant Vector
        else u.delta() + v.delta()

      case MVMul(a, b) =>
        if (a.isInstanceOf[ConstantMatrix]) MVMul(a, b.delta()) // Ignore variation of constant matrix
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) MVMul(a.delta(), b) // Ignore variation of constant Vector
        else MVMul(a.delta(), b) + MVMul(a, b.delta())

      case SMul(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) SMul(a, b.delta()) // Ignore variation of constant Vector
        else if (b.isInstanceOf[NumScalar] || b.isInstanceOf[ConstScalar]) SMul(a.delta(), b) // Ignore variation of constant scalar
        else SMul(a.delta(), b) + SMul(a, b.delta())

      case Cross(a, b) =>
        if (a.isInstanceOf[ConstantVector] || a.isInstanceOf[ZeroVector]) Cross(a, b.delta()) // Ignore variation of constant scalar
        else if (b.isInstanceOf[ConstantVector] || b.isInstanceOf[ZeroVector]) Cross(a.delta(), b) // Ignore variation of constant scalar
        else Cross(a.delta(), b) + Cross(a, b.delta())

      case TransposeVector(v) => TransposeVector(v.delta())

      case VeeMap(m) => VeeMap(m.delta())

      case _ => this.diff()
    }
  }

  override def getVariation: VectorExpr = this.delta()

  //Infix operators
  def x(v: VectorExpr): VectorExpr = Cross(this, v)

  def *(v: ScalarExpr): VectorExpr = SMul(this, v)

  def ***(v: TransposeVector): MatrixExpr = VVMul(this, v)

  def +(v: VectorExpr): VectorExpr = VAdd(this, v)

  def -(v: VectorExpr): VectorExpr = VAdd(this, SMul(v, NumScalar(-1)))

  def dot(v: VectorExpr): ScalarExpr = Dot(this, v)

  def T: VectorExpr = TransposeVector(this)

  // other functions
  override def basicSimplify(): VectorExpr = {
    this match {
      case _ => this
    }
  }

}

//
// Vector Algebra classes
//
case class DeltaV(u: VectorExpr) extends VectorExpr // delta prefix

case class Cross(u: VectorExpr, v: VectorExpr) extends VectorExpr // u x v infix

case class SMul(u: VectorExpr, v: ScalarExpr) extends VectorExpr // u * v infix

case class VAdd(u: VectorExpr, v: VectorExpr) extends VectorExpr // u + v infix

case class Dot(u: VectorExpr, v: VectorExpr) extends ScalarExpr // u dot v infix

case class TransposeVector(v: VectorExpr) extends VectorExpr {

  override def T: VectorExpr = this.v

}

case class VVMul(u: VectorExpr, v: TransposeVector) extends MatrixExpr

case class VeeMap(m: MatrixExpr) extends VectorExpr // TODO update input to SkewSymMatrix

//
// Vector Types
//
trait BaseVectorVariable extends VectorExpr with Variable {

  override val size = List(3, 1)

  override def diff(): VectorExpr = Vector("dot" + this.name)

  override def delta(): VectorExpr = DeltaV(this)

  override def d: VectorExpr = Vector(this.name + "_d")
}

case class Vector(override val name: String) extends BaseVectorVariable

case class UnitVector(override val name: String) extends BaseVectorVariable with UnitNorm {
  // UnitVector
  override def d: VectorExpr = UnitVector(this.name + "_d")
}

case class ConstVector(override val name: String) extends BaseVectorVariable with ConstantVector {
  // Constant Vector
  override def diff(): VectorExpr = SMul(this, NumScalar(0))

  override def delta(): VectorExpr = SMul(this, NumScalar(0.0))

  override def d: VectorExpr = ConstVector(this.name + "_d")

}

//case class AVec(s: String, u: VectorExpr) extends VectorExpr // holds symbolic reference for large vector

case class ZeroVector(override val name: String) extends BaseVectorVariable {
  // ZeroVector Vector
  override def diff(): VectorExpr = SMul(this, NumScalar(0))

  override def delta(): VectorExpr = SMul(this, NumScalar(0.0))

  override def d: VectorExpr = ZeroVector(this.name + "_d")
}

case class SkewMatVector(override val name: String) extends BaseVectorVariable {
  // TODO update this vector  (or make it obsolete)
  override def diff(): VectorExpr = Vector(this.name + "dot")

  override def delta(): VectorExpr = DeltaV(this)

  override def d: VectorExpr = SkewMatVector(this.name + "_d")

}

case class S2(override val name: String) extends BaseVectorVariable with UnitNorm with SmoothManifold {

  val variationStr: String = "xi_{" + this.name + "}"
  val tangentStr: String = "omega_{" + this.name + "}" // TODO find a better way to represent the variable

  def norm: ScalarExpr = NumScalar(1.0)

  override def getTangentVector: VectorExpr = Vector(this.tangentStr)

  override def getVariationVector: VectorExpr = Vector(this.variationStr)

  override def delta(): VectorExpr = Cross(this.getVariationVector, this)

  override def diff(): VectorExpr = Cross(this.getTangentVector, this)

  override def getVariation: VectorExpr = this.getVariationVector

  override def d: VectorExpr = S2(this.name + "_d")

}